#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using HighState = unitree_go::msg::dds_::SportModeState_;

constexpr const char* kLowCmdTopic = "rt/lowcmd";
constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kHighStateTopic = "rt/sportmodestate";

constexpr int kPolicyJointCount = 24;
constexpr int kR1MotorCount = 26;
constexpr double kCommandPublishDtSec = 0.002;

std::atomic_bool g_stop_requested{false};

const std::array<std::string, kPolicyJointCount> kPolicyJointNames = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
};

constexpr std::array<int, kPolicyJointCount> kJointIdxInIdl = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26
};

struct ReplayFrame
{
    double time_s = 0.0;
    std::array<float, kPolicyJointCount> target_q{};
};

template <typename T>
class DataBuffer
{
public:
    void set(const T& new_data)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = std::make_shared<T>(new_data);
    }

    std::shared_ptr<const T> get() const
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return data_ ? data_ : nullptr;
    }

private:
    mutable std::shared_mutex mutex_;
    std::shared_ptr<T> data_;
};

struct MotorStateView
{
    std::array<float, kPolicyJointCount> q{};
    std::array<float, kPolicyJointCount> dq{};
    uint8_t mode_machine = 0;
};

struct RootStateView
{
    std::array<float, 3> gyro{};
    std::array<float, 4> quat{};
    std::array<float, 3> position{};
};

struct MotorCommandView
{
    std::array<float, kPolicyJointCount> q_target{};
    std::array<float, kPolicyJointCount> kp{};
    std::array<float, kPolicyJointCount> kd{};
    std::array<float, kPolicyJointCount> tau_ff{};
};

struct ReplaySample
{
    double time_s = 0.0;
    std::array<float, 3> base_ang_vel{};
    std::array<float, 3> projected_gravity{};
    std::array<float, kPolicyJointCount> joint_pos_rel{};
    std::array<float, kPolicyJointCount> joint_vel_rel{};
    std::array<float, kPolicyJointCount> target_q{};
    std::array<float, 2> root_xy{};
    float root_yaw = 0.0f;
};

std::string timestamp_string()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&now_time, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    for (char ch : line) {
        if (ch == ',') {
            fields.push_back(current);
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    fields.push_back(current);
    return fields;
}

float quat_to_yaw(float w, float x, float y, float z)
{
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 1e-8f) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
    return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

std::array<float, 3> projected_gravity_from_quat(float w, float x, float y, float z)
{
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 1e-8f) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // Rotate gravity (0,0,-1) into body frame: R^T * g.
    const float gx = 2.0f * (x * z - w * y);
    const float gy = 2.0f * (y * z + w * x);
    const float gz = 1.0f - 2.0f * (x * x + y * y);
    return {gx, gy, -gz};
}

inline uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t crc = 0xFFFFFFFF;
    constexpr uint32_t polynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1u << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (crc & 0x80000000) {
                crc <<= 1;
                crc ^= polynomial;
            } else {
                crc <<= 1;
            }
            if (data & xbit) {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }
    return crc;
}

class ReplayLogger
{
public:
    void start(const std::filesystem::path& output_dir)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        output_dir_ = output_dir;
        samples_.clear();
        flushed_ = false;
        std::filesystem::create_directories(output_dir_);
    }

    void add(const ReplaySample& sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_) {
            return;
        }
        samples_.push_back(sample);
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_) {
            return;
        }
        flushed_ = true;
        if (samples_.empty()) {
            return;
        }

        const auto csv_path = output_dir_ / "target_replay.csv";
        std::ofstream csv(csv_path);
        csv << "time_s";
        for (int i = 0; i < 3; ++i) csv << ",base_ang_vel_" << i;
        for (int i = 0; i < 3; ++i) csv << ",projected_gravity_" << i;
        for (int i = 0; i < kPolicyJointCount; ++i) csv << ",joint_pos_rel_" << i;
        for (int i = 0; i < kPolicyJointCount; ++i) csv << ",joint_vel_rel_" << i;
        for (int i = 0; i < kPolicyJointCount; ++i) csv << ",target_q_" << i;
        csv << ",root_xy_0,root_xy_1,root_yaw\n";

        for (const auto& s : samples_) {
            csv << s.time_s;
            for (float v : s.base_ang_vel) csv << "," << v;
            for (float v : s.projected_gravity) csv << "," << v;
            for (float v : s.joint_pos_rel) csv << "," << v;
            for (float v : s.joint_vel_rel) csv << "," << v;
            for (float v : s.target_q) csv << "," << v;
            csv << "," << s.root_xy[0] << "," << s.root_xy[1] << "," << s.root_yaw << "\n";
        }
        std::cout << "[replay] wrote target replay csv to " << csv_path << std::endl;
    }

private:
    std::mutex mutex_;
    bool flushed_ = false;
    std::filesystem::path output_dir_;
    std::vector<ReplaySample> samples_;
};

std::vector<ReplayFrame> load_target_frames_from_obs_compare(const std::filesystem::path& csv_path)
{
    std::ifstream in(csv_path);
    if (!in) {
        throw std::runtime_error("Failed to open obs_compare csv: " + csv_path.string());
    }

    std::string header_line;
    if (!std::getline(in, header_line)) {
        throw std::runtime_error("Empty obs_compare csv: " + csv_path.string());
    }
    const auto headers = split_csv_line(header_line);

    std::array<int, kPolicyJointCount> target_cols{};
    target_cols.fill(-1);
    int time_col = -1;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
        if (headers[i] == "time_s") {
            time_col = i;
            continue;
        }
        const std::string prefix = "target_q_";
        if (headers[i].rfind(prefix, 0) == 0) {
            const int idx = std::stoi(headers[i].substr(prefix.size()));
            if (idx >= 0 && idx < kPolicyJointCount) {
                target_cols[idx] = i;
            }
        }
    }
    if (time_col < 0) {
        throw std::runtime_error("obs_compare csv missing time_s column: " + csv_path.string());
    }
    for (int i = 0; i < kPolicyJointCount; ++i) {
        if (target_cols[i] < 0) {
            throw std::runtime_error("obs_compare csv missing target_q_" + std::to_string(i));
        }
    }

    std::vector<ReplayFrame> frames;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = split_csv_line(line);
        ReplayFrame frame;
        frame.time_s = std::stod(fields.at(time_col));
        for (int i = 0; i < kPolicyJointCount; ++i) {
            frame.target_q[i] = std::stof(fields.at(target_cols[i]));
        }
        frames.push_back(frame);
    }
    if (frames.empty()) {
        throw std::runtime_error("No replay frames found in: " + csv_path.string());
    }
    return frames;
}

class TargetReplayDiagnostic
{
public:
    TargetReplayDiagnostic(std::string network,
                           std::filesystem::path replay_csv,
                           std::filesystem::path config_path)
    {
        load_config(config_path);
        replay_frames_ = load_target_frames_from_obs_compare(replay_csv);

        logger_.start(std::filesystem::path(__FILE__).parent_path() / "../log" /
                      ("target_replay_" + timestamp_string()));

        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        motion_switcher_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        motion_switcher_->SetTimeout(5.0f);
        motion_switcher_->Init();
        std::string form;
        std::string name;
        while (motion_switcher_->CheckMode(form, name), !name.empty()) {
            std::cout << "Releasing active motion mode: " << name << std::endl;
            motion_switcher_->ReleaseMode();
            sleep(1);
        }

        lowcmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<LowCmd>>(kLowCmdTopic);
        lowcmd_pub_->InitChannel();

        lowstate_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>(kLowStateTopic);
        lowstate_sub_->InitChannel(
            std::bind(&TargetReplayDiagnostic::handle_lowstate, this, std::placeholders::_1), 1);

        highstate_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<HighState>>(kHighStateTopic);
        highstate_sub_->InitChannel(
            std::bind(&TargetReplayDiagnostic::handle_highstate, this, std::placeholders::_1), 1);

        writer_thread_ = std::thread(&TargetReplayDiagnostic::write_loop, this);
        control_thread_ = std::thread(&TargetReplayDiagnostic::control_loop, this);
    }

    ~TargetReplayDiagnostic()
    {
        stop();
    }

    void stop()
    {
        running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        if (writer_thread_.joinable()) {
            writer_thread_.join();
        }
        logger_.flush();
    }

private:
    void load_config(const std::filesystem::path& config_path)
    {
        const YAML::Node cfg = YAML::LoadFile(config_path.string());
        step_dt_ = cfg["step_dt"].as<double>();
        const auto kp = cfg["policy_kp"].as<std::vector<float>>();
        const auto kd = cfg["policy_kd"].as<std::vector<float>>();
        const auto torque = cfg["torque_limit"].as<std::vector<float>>();
        const auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
        if (kp.size() != kPolicyJointCount || kd.size() != kPolicyJointCount ||
            torque.size() != kPolicyJointCount || default_joint_pos.size() != kPolicyJointCount) {
            throw std::runtime_error("Unexpected R1 deploy config dimensions for replay.");
        }
        for (int i = 0; i < kPolicyJointCount; ++i) {
            policy_kp_[i] = kp[i];
            policy_kd_[i] = kd[i];
            torque_limit_[i] = torque[i];
            default_joint_pos_[i] = default_joint_pos[i];
        }
    }

    void handle_lowstate(const void* message)
    {
        LowState low_state = *static_cast<const LowState*>(message);
        if (low_state.crc() != crc32_core((uint32_t*)&low_state, (sizeof(LowState) >> 2) - 1)) {
            std::cerr << "[replay] CRC error on lowstate" << std::endl;
            return;
        }

        MotorStateView state{};
        for (int i = 0; i < kPolicyJointCount; ++i) {
            const int slot = kJointIdxInIdl[i];
            state.q[i] = low_state.motor_state()[slot].q();
            state.dq[i] = low_state.motor_state()[slot].dq();
        }
        state.mode_machine = low_state.mode_machine();
        lowstate_buffer_.set(state);
    }

    void handle_highstate(const void* message)
    {
        const HighState msg = *static_cast<const HighState*>(message);
        RootStateView state{};
        state.position[0] = msg.position()[0];
        state.position[1] = msg.position()[1];
        state.position[2] = msg.position()[2];
        state.gyro[0] = msg.imu_state().gyroscope()[0];
        state.gyro[1] = msg.imu_state().gyroscope()[1];
        state.gyro[2] = msg.imu_state().gyroscope()[2];
        state.quat[0] = msg.imu_state().quaternion()[0];
        state.quat[1] = msg.imu_state().quaternion()[1];
        state.quat[2] = msg.imu_state().quaternion()[2];
        state.quat[3] = msg.imu_state().quaternion()[3];
        highstate_buffer_.set(state);
    }

    void write_loop()
    {
        while (running_) {
            LowCmd cmd{};
            cmd.mode_pr() = 0;
            cmd.mode_machine() = mode_machine_;

            auto desired = command_buffer_.get();
            if (desired) {
                for (int i = 0; i < kPolicyJointCount; ++i) {
                    const int slot = kJointIdxInIdl[i];
                    cmd.motor_cmd()[slot].mode() = 1;
                    cmd.motor_cmd()[slot].q() = desired->q_target[i];
                    cmd.motor_cmd()[slot].dq() = 0.0f;
                    cmd.motor_cmd()[slot].kp() = 0.0f;
                    cmd.motor_cmd()[slot].kd() = 0.0f;
                    cmd.motor_cmd()[slot].tau() = desired->tau_ff[i];
                }
            }

            cmd.crc() = crc32_core((uint32_t*)&cmd, (sizeof(LowCmd) >> 2) - 1);
            lowcmd_pub_->Write(cmd);
            std::this_thread::sleep_for(std::chrono::duration<double>(kCommandPublishDtSec));
        }
    }

    void control_loop()
    {
        using steady_clock = std::chrono::steady_clock;
        const auto start = steady_clock::now();
        int last_frame_index = -1;

        while (running_) {
            auto low = lowstate_buffer_.get();
            auto high = highstate_buffer_.get();
            if (!low || !high) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (mode_machine_ == 0 && low->mode_machine != 0) {
                mode_machine_ = low->mode_machine;
                std::cout << "[replay] detected mode_machine = " << unsigned(mode_machine_) << std::endl;
            }

            const double elapsed = std::chrono::duration<double>(steady_clock::now() - start).count();
            int frame_index = static_cast<int>(std::floor(elapsed / step_dt_));
            if (frame_index >= static_cast<int>(replay_frames_.size())) {
                std::cout << "[replay] completed " << replay_frames_.size() << " replay frames, stopping automatically" << std::endl;
                running_ = false;
                g_stop_requested = true;
                break;
            }
            if (frame_index < 0) {
                frame_index = 0;
            }

            MotorCommandView cmd{};
            const auto& frame = replay_frames_[frame_index];
            for (int i = 0; i < kPolicyJointCount; ++i) {
                cmd.q_target[i] = frame.target_q[i];
                cmd.kp[i] = policy_kp_[i];
                cmd.kd[i] = policy_kd_[i];
                const float tau = std::clamp(
                    policy_kp_[i] * (frame.target_q[i] - low->q[i]) + policy_kd_[i] * (-low->dq[i]),
                    -torque_limit_[i],
                    torque_limit_[i]
                );
                cmd.tau_ff[i] = tau;
            }
            command_buffer_.set(cmd);

            if (frame_index != last_frame_index) {
                last_frame_index = frame_index;
                ReplaySample sample;
                sample.time_s = frame.time_s;
                sample.base_ang_vel = high->gyro;
                sample.projected_gravity = projected_gravity_from_quat(
                    high->quat[0], high->quat[1], high->quat[2], high->quat[3]);
                for (int i = 0; i < kPolicyJointCount; ++i) {
                    sample.joint_pos_rel[i] = low->q[i] - default_joint_pos_[i];
                    sample.joint_vel_rel[i] = low->dq[i];
                    sample.target_q[i] = frame.target_q[i];
                }
                sample.root_xy[0] = high->position[0];
                sample.root_xy[1] = high->position[1];
                sample.root_yaw = quat_to_yaw(high->quat[0], high->quat[1], high->quat[2], high->quat[3]);
                logger_.add(sample);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    std::atomic_bool running_{true};
    uint8_t mode_machine_ = 0;
    double step_dt_ = 0.02;

    std::array<float, kPolicyJointCount> policy_kp_{};
    std::array<float, kPolicyJointCount> policy_kd_{};
    std::array<float, kPolicyJointCount> torque_limit_{};
    std::array<float, kPolicyJointCount> default_joint_pos_{};

    std::vector<ReplayFrame> replay_frames_;
    ReplayLogger logger_;

    DataBuffer<MotorStateView> lowstate_buffer_;
    DataBuffer<RootStateView> highstate_buffer_;
    DataBuffer<MotorCommandView> command_buffer_;

    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;
    std::shared_ptr<unitree::robot::ChannelPublisher<LowCmd>> lowcmd_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<LowState>> lowstate_sub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<HighState>> highstate_sub_;

    std::thread writer_thread_;
    std::thread control_thread_;
};

void signal_handler(int)
{
    g_stop_requested = true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4) {
        std::cout << "Usage: r1_target_replay_diag <network_interface> <obs_compare_csv> [deploy_yaml]" << std::endl;
        return 1;
    }

    const std::string network = argv[1];
    const std::filesystem::path replay_csv = argv[2];
    const std::filesystem::path config_path =
        argc >= 4 ? std::filesystem::path(argv[3])
                  : std::filesystem::path(__FILE__).parent_path() / "../config/policy/track/params/deploy.yaml";

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "Starting target_q replay diagnostic from '" << replay_csv << "'" << std::endl;
    TargetReplayDiagnostic diag(network, replay_csv, config_path);
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    diag.stop();
    return 0;
}
