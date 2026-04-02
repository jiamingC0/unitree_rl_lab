#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;

constexpr const char* kLowCmdTopic = "rt/lowcmd";
constexpr const char* kLowStateTopic = "rt/lowstate";

constexpr int kR1MotorCount = 26;
constexpr double kSettleDurationSec = 2.0;
constexpr double kTestDurationSec = 5.0;
constexpr double kPauseDurationSec = 1.0;
constexpr double kSineFrequencyHz = 0.5;

std::atomic_bool g_stop_requested{false};

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

constexpr std::array<int, kR1MotorCount> kJointIdxInIdl = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30
};

enum class ControlMode : uint8_t
{
    PR = 0,
    AB = 1,
};

// The 26 logical channels exposed by the R1 low-level example. In PR mode the ankle
// channels are interpreted as pitch/roll. In AB mode they are interpreted as B/A.
const std::array<std::string, kR1MotorCount> kJointNamesPr = {
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
    "head_pitch_joint",
    "head_yaw_joint",
};

const std::array<std::string, kR1MotorCount> kJointNamesAb = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_b_joint",
    "left_ankle_a_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_b_joint",
    "right_ankle_a_joint",
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
    "head_pitch_joint",
    "head_yaw_joint",
};

constexpr std::array<float, kR1MotorCount> kKp = {
    200, 200, 200, 200, 20, 10,
    200, 200, 200, 200, 20, 10,
    300, 300,
    100, 100, 100, 100, 50,
    100, 100, 100, 100, 50,
    50, 10
};

constexpr std::array<float, kR1MotorCount> kKd = {
    3, 3, 3, 3, 0.3f, 0.1f,
    3, 3, 3, 3, 0.3f, 0.1f,
    5, 5,
    2, 2, 2, 2, 2,
    2, 2, 2, 2, 2,
    2, 0.1f
};

// Conservative amplitudes keep the sequential test informative while avoiding violent
// whole-body motion when a single joint is isolated.
constexpr std::array<float, kR1MotorCount> kAmplitude = {
    0.25f, 0.20f, 0.20f, 0.30f, 0.18f, 0.12f,
    0.25f, 0.20f, 0.20f, 0.30f, 0.18f, 0.12f,
    0.18f, 0.18f,
    0.25f, 0.20f, 0.20f, 0.25f, 0.18f,
    0.25f, 0.20f, 0.20f, 0.25f, 0.18f,
    0.12f, 0.12f
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
    std::array<float, kR1MotorCount> q{};
    std::array<float, kR1MotorCount> dq{};
    uint8_t mode_machine = 0;
};

struct MotorCommandView
{
    std::array<float, kR1MotorCount> q_target{};
    std::array<float, kR1MotorCount> dq_target{};
    std::array<float, kR1MotorCount> kp{};
    std::array<float, kR1MotorCount> kd{};
    std::array<float, kR1MotorCount> tau_ff{};
};

struct JointDiagnosticSample
{
    double global_time_s = 0.0;
    double local_time_s = 0.0;
    float target_q = 0.0f;
    float actual_q = 0.0f;
    float actual_dq = 0.0f;
    std::array<float, kR1MotorCount> actual_q_all{};
};

class JointDiagnosticLogger
{
public:
    explicit JointDiagnosticLogger(std::array<std::string, kR1MotorCount> joint_names)
        : joint_names_(std::move(joint_names))
    {
    }

    void start(const std::filesystem::path& output_dir)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        output_dir_ = output_dir;
        flushed_ = false;
        for (auto& samples : samples_per_joint_) {
            samples.clear();
        }
        std::filesystem::create_directories(output_dir_);
    }

    void add(int joint_index, const JointDiagnosticSample& sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_ || joint_index < 0 || joint_index >= kR1MotorCount) {
            return;
        }
        samples_per_joint_[joint_index].push_back(sample);
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (flushed_) {
            return;
        }
        flushed_ = true;

        const auto repo_root = std::filesystem::weakly_canonical(
            std::filesystem::path(__FILE__).parent_path() / "../../../..");
        const auto script_path = repo_root / "scripts" / "r1" / "plot_joint_sine_diag.py";

        for (int joint_index = 0; joint_index < kR1MotorCount; ++joint_index) {
            const auto& joint_name = joint_names_[joint_index];
            const auto& samples = samples_per_joint_[joint_index];
            if (samples.empty()) {
                continue;
            }

            const auto csv_path = output_dir_ / (joint_name + ".csv");
            std::ofstream csv(csv_path);
            csv << "global_time_s,local_time_s,target_q,actual_q,actual_dq";
            for (const auto& other_joint_name : joint_names_) {
                csv << ",actual_q__" << other_joint_name;
            }
            csv << "\n";
            for (const auto& sample : samples) {
                csv << sample.global_time_s << ","
                    << sample.local_time_s << ","
                    << sample.target_q << ","
                    << sample.actual_q << ","
                    << sample.actual_dq;
                for (float q_value : sample.actual_q_all) {
                    csv << "," << q_value;
                }
                csv << "\n";
            }
            csv.close();

            std::ostringstream cmd;
            cmd << "python3 \"" << script_path.string() << "\""
                << " --csv \"" << csv_path.string() << "\""
                << " --output-dir \"" << output_dir_.string() << "\""
                << " --joint-name \"" << joint_name << "\"";
            const int ret = std::system(cmd.str().c_str());
            if (ret != 0) {
                std::cerr << "[diag] failed to generate plot for " << joint_name << std::endl;
            }
        }

        std::cout << "[diag] wrote sequential joint diagnostics to " << output_dir_ << std::endl;
    }

private:
    std::mutex mutex_;
    bool flushed_ = false;
    std::filesystem::path output_dir_;
    std::array<std::string, kR1MotorCount> joint_names_;
    std::array<std::vector<JointDiagnosticSample>, kR1MotorCount> samples_per_joint_{};
};

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

class SequentialJointDiagnostic
{
public:
    SequentialJointDiagnostic(std::string network, ControlMode mode)
        : mode_(mode),
          joint_names_(mode_ == ControlMode::PR ? kJointNamesPr : kJointNamesAb),
          logger_(joint_names_)
    {
        logger_.start(std::filesystem::path(__FILE__).parent_path() / "../log" /
                      ("joint_sine_diag_" +
                       std::string(mode_ == ControlMode::PR ? "pr" : "ab") +
                       "_" + timestamp_string()));

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
            std::bind(&SequentialJointDiagnostic::handle_lowstate, this, std::placeholders::_1), 1);

        writer_thread_ = std::thread(&SequentialJointDiagnostic::write_loop, this);
        control_thread_ = std::thread(&SequentialJointDiagnostic::control_loop, this);
    }

    ~SequentialJointDiagnostic()
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
    void handle_lowstate(const void* message)
    {
        LowState low_state = *static_cast<const LowState*>(message);
        if (low_state.crc() != crc32_core((uint32_t*)&low_state, (sizeof(LowState) >> 2) - 1)) {
            std::cerr << "[diag] CRC error on lowstate" << std::endl;
            return;
        }

        MotorStateView state{};
        for (int i = 0; i < kR1MotorCount; ++i) {
            const int slot = kJointIdxInIdl[i];
            state.q[i] = low_state.motor_state()[slot].q();
            state.dq[i] = low_state.motor_state()[slot].dq();
        }
        state.mode_machine = low_state.mode_machine();
        state_buffer_.set(state);
    }

    void write_loop()
    {
        while (running_) {
            LowCmd cmd{};
            cmd.mode_pr() = static_cast<uint8_t>(mode_);
            cmd.mode_machine() = mode_machine_;

            auto desired = command_buffer_.get();
            if (desired) {
                for (int i = 0; i < kR1MotorCount; ++i) {
                    const int slot = kJointIdxInIdl[i];
                    cmd.motor_cmd()[slot].mode() = 1;
                    cmd.motor_cmd()[slot].q() = desired->q_target[i];
                    cmd.motor_cmd()[slot].dq() = desired->dq_target[i];
                    cmd.motor_cmd()[slot].kp() = desired->kp[i];
                    cmd.motor_cmd()[slot].kd() = desired->kd[i];
                    cmd.motor_cmd()[slot].tau() = desired->tau_ff[i];
                }
            }

            cmd.crc() = crc32_core((uint32_t*)&cmd, (sizeof(LowCmd) >> 2) - 1);
            lowcmd_pub_->Write(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void control_loop()
    {
        using steady_clock = std::chrono::steady_clock;
        const auto start = steady_clock::now();
        bool baseline_initialized = false;
        std::array<float, kR1MotorCount> baseline_q{};
        int last_reported_joint = -2;

        while (running_) {
            auto state = state_buffer_.get();
            if (!state) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (mode_machine_ == 0 && state->mode_machine != 0) {
                mode_machine_ = state->mode_machine;
                std::cout << "[diag] detected mode_machine = " << unsigned(mode_machine_) << std::endl;
            }

            const double elapsed = std::chrono::duration<double>(steady_clock::now() - start).count();
            if (!baseline_initialized && elapsed >= kSettleDurationSec) {
                baseline_q = state->q;
                baseline_initialized = true;
                std::cout << "[diag] baseline posture captured, starting sequential sine test" << std::endl;
            }

            MotorCommandView cmd{};
            for (int i = 0; i < kR1MotorCount; ++i) {
                cmd.kp[i] = kKp[i];
                cmd.kd[i] = kKd[i];
                cmd.q_target[i] = baseline_initialized ? baseline_q[i] : state->q[i];
            }

            int active_joint = -1;
            double local_time = 0.0;
            if (baseline_initialized) {
                const double phase_time = elapsed - kSettleDurationSec;
                const double cycle = kTestDurationSec + kPauseDurationSec;
                const int joint_block = static_cast<int>(phase_time / cycle);
                if (joint_block < kR1MotorCount) {
                    const double time_in_cycle = phase_time - joint_block * cycle;
                    if (time_in_cycle < kTestDurationSec) {
                        active_joint = joint_block;
                        local_time = time_in_cycle;
                        cmd.q_target[active_joint] =
                            baseline_q[active_joint] +
                            kAmplitude[active_joint] *
                                std::sin(2.0 * M_PI * kSineFrequencyHz * local_time);
                    }
                } else {
                    std::cout << "[diag] completed all " << kR1MotorCount
                              << " joint tests, stopping automatically" << std::endl;
                    running_ = false;
                    g_stop_requested = true;
                    break;
                }
            }

            command_buffer_.set(cmd);

            if (active_joint != last_reported_joint) {
                last_reported_joint = active_joint;
                if (active_joint >= 0) {
                    std::cout << "[diag] testing joint " << active_joint << " : "
                              << joint_names_[active_joint] << " for " << kTestDurationSec
                              << " s, then pausing " << kPauseDurationSec << " s" << std::endl;
                } else if (baseline_initialized) {
                    std::cout << "[diag] pause / sequence complete" << std::endl;
                }
            }

            if (active_joint >= 0) {
                logger_.add(active_joint, {
                    elapsed,
                    local_time,
                    cmd.q_target[active_joint],
                    state->q[active_joint],
                    state->dq[active_joint],
                    state->q,
                });
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    ControlMode mode_;
    std::array<std::string, kR1MotorCount> joint_names_;
    std::atomic_bool running_{true};
    uint8_t mode_machine_ = 0;
    JointDiagnosticLogger logger_;

    DataBuffer<MotorStateView> state_buffer_;
    DataBuffer<MotorCommandView> command_buffer_;

    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;
    std::shared_ptr<unitree::robot::ChannelPublisher<LowCmd>> lowcmd_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<LowState>> lowstate_sub_;

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
    if (argc < 2 || argc > 3) {
        std::cout << "Usage: r1_joint_sine_diag <network_interface> [pr|ab]" << std::endl;
        return 1;
    }

    const std::string network = argv[1];
    ControlMode mode = ControlMode::PR;
    if (argc == 3) {
        const std::string arg = argv[2];
        if (arg == "ab") {
            mode = ControlMode::AB;
        } else if (arg != "pr") {
            std::cerr << "Unknown mode '" << arg << "', expected 'pr' or 'ab'." << std::endl;
            return 1;
        }
    }

    std::cout << "Starting sequential joint sine diagnostic in "
              << (mode == ControlMode::PR ? "PR" : "AB")
              << " mode. Each joint runs for 5 s, pauses 1 s, then switches automatically."
              << std::endl;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    SequentialJointDiagnostic diag(network, mode);
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    diag.stop();
    return 0;
}
