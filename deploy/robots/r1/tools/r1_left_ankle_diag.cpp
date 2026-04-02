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

#include <unitree/idl/hg/IMUState_.hpp>
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
std::atomic_bool g_stop_requested{false};

std::string timestamp_string()
{
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&now_time, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

// Logical R1 joint order used by the SDK example and by our deploy pipeline.
constexpr std::array<int, kR1MotorCount> kJointIdxInIdl = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30
};

enum class AnkleMode : uint8_t
{
    PR = 0,
    AB = 1,
};

enum JointIndex
{
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleRoll = 11,
};

constexpr std::array<float, kR1MotorCount> kKp = {
    200, 200, 200, 200, 20, 10,
    200, 200, 200, 200, 200, 200,
    300, 300,
    100, 100, 100, 100, 50,
    100, 100, 100, 100, 50,
    50, 10
};

constexpr std::array<float, kR1MotorCount> kKd = {
    3, 3, 3, 3, 0.3f, 0.1f,
    3, 3, 3, 3, 3, 3,
    5, 5,
    2, 2, 2, 2, 2,
    2, 2, 2, 2, 2,
    2, 0.1f
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

struct DiagnosticSample
{
    double time_s = 0.0;
    float target_pitch = 0.0f;
    float target_roll = 0.0f;
    float actual_pitch = 0.0f;
    float actual_roll = 0.0f;
    float dq_pitch = 0.0f;
    float dq_roll = 0.0f;
};

class DiagnosticLogger
{
public:
    void start(const std::filesystem::path& output_dir)
    {
        output_dir_ = output_dir;
        samples_.clear();
        flushed_ = false;
        std::filesystem::create_directories(output_dir_);
    }

    void add(const DiagnosticSample& sample)
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
        if (flushed_ || samples_.empty()) {
            flushed_ = true;
            return;
        }

        const auto csv_path = output_dir_ / "left_ankle_diag.csv";
        std::ofstream csv(csv_path);
        csv << "time_s,target_pitch,target_roll,actual_pitch,actual_roll,dq_pitch,dq_roll\n";
        for (const auto& s : samples_) {
            csv << s.time_s << ","
                << s.target_pitch << ","
                << s.target_roll << ","
                << s.actual_pitch << ","
                << s.actual_roll << ","
                << s.dq_pitch << ","
                << s.dq_roll << "\n";
        }
        csv.close();

        const auto repo_root = std::filesystem::weakly_canonical(
            std::filesystem::path(__FILE__).parent_path() / "../../../..");
        const auto script_path = repo_root / "scripts" / "r1" / "plot_left_ankle_diag.py";

        std::ostringstream cmd;
        cmd << "python3 \"" << script_path.string() << "\""
            << " --csv \"" << csv_path.string() << "\""
            << " --output-dir \"" << output_dir_.string() << "\"";
        const int ret = std::system(cmd.str().c_str());
        if (ret != 0) {
            std::cerr << "[diag] failed to generate plots from " << csv_path << std::endl;
        } else {
            std::cout << "[diag] wrote CSV and plots to " << output_dir_ << std::endl;
        }

        flushed_ = true;
    }

private:
    std::mutex mutex_;
    bool flushed_ = false;
    std::filesystem::path output_dir_;
    std::vector<DiagnosticSample> samples_;
};

// Same CRC helper used in the official SDK examples so the diagnostic behaves
// exactly like the shipped low-level tools.
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

class LeftAnkleDiagnostic
{
public:
    LeftAnkleDiagnostic(std::string network, AnkleMode mode)
        : mode_(mode)
    {
        const std::string mode_name = mode_ == AnkleMode::PR ? "pr" : "ab";
        logger_.start(std::filesystem::path(__FILE__).parent_path() / "../log" /
                      ("left_ankle_diag_" + mode_name + "_" + timestamp_string()));
        unitree::robot::ChannelFactory::Instance()->Init(0, network);

        // Release any active robot-side motion service so low-level control is accepted.
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
            std::bind(&LeftAnkleDiagnostic::handle_lowstate, this, std::placeholders::_1), 1);

        writer_thread_ = std::thread(&LeftAnkleDiagnostic::write_loop, this);
        control_thread_ = std::thread(&LeftAnkleDiagnostic::control_loop, this);
    }

    ~LeftAnkleDiagnostic()
    {
        stop();
    }

    void stop()
    {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }
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
        int print_counter = 0;

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

            MotorCommandView cmd{};
            for (int i = 0; i < kR1MotorCount; ++i) {
                cmd.kp[i] = kKp[i];
                cmd.kd[i] = kKd[i];
            }

            const double elapsed = std::chrono::duration<double>(steady_clock::now() - start).count();
            if (elapsed < 2.0) {
                // First settle all joints to the current posture so the ankle test starts quietly.
                for (int i = 0; i < kR1MotorCount; ++i) {
                    cmd.q_target[i] = state->q[i];
                }
            } else {
                // Only excite the left ankle. All other joints stay at their current posture.
                for (int i = 0; i < kR1MotorCount; ++i) {
                    cmd.q_target[i] = state->q[i];
                }

                const double t = elapsed - 2.0;
                if (mode_ == AnkleMode::PR) {
                    // In PR mode we command the semantic pitch/roll coordinates directly.
                    cmd.q_target[LeftAnklePitch] = 0.35f * std::sin(2.0 * M_PI * 0.5 * t);
                    cmd.q_target[LeftAnkleRoll] = 0.0f;
                } else {
                    // In AB mode we command the two parallel ankle actuators directly.
                    cmd.q_target[LeftAnkleA] = 0.35f * std::sin(2.0 * M_PI * 0.5 * t);
                    cmd.q_target[LeftAnkleB] = 0.0f;
                }
            }

            command_buffer_.set(cmd);

            if (++print_counter >= 100) {
                print_counter = 0;
                std::cout
                    << "[diag] mode=" << (mode_ == AnkleMode::PR ? "PR" : "AB")
                    << " target(L_pitch/L_roll)=(" << cmd.q_target[LeftAnklePitch] << ", "
                    << cmd.q_target[LeftAnkleRoll] << ")"
                    << " actual(L_pitch/L_roll)=(" << state->q[LeftAnklePitch] << ", "
                    << state->q[LeftAnkleRoll] << ")"
                    << " dq=(" << state->dq[LeftAnklePitch] << ", "
                    << state->dq[LeftAnkleRoll] << ")"
                    << std::endl;
            }

            logger_.add({
                elapsed,
                cmd.q_target[LeftAnklePitch],
                cmd.q_target[LeftAnkleRoll],
                state->q[LeftAnklePitch],
                state->q[LeftAnkleRoll],
                state->dq[LeftAnklePitch],
                state->dq[LeftAnkleRoll],
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    AnkleMode mode_;
    std::atomic_bool running_{true};
    uint8_t mode_machine_ = 0;
    DiagnosticLogger logger_;

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
        std::cout << "Usage: r1_left_ankle_diag <network_interface> [pr|ab]" << std::endl;
        return 1;
    }

    const std::string network = argv[1];
    AnkleMode mode = AnkleMode::PR;
    if (argc == 3) {
        const std::string arg = argv[2];
        if (arg == "ab") {
            mode = AnkleMode::AB;
        } else if (arg != "pr") {
            std::cerr << "Unknown mode '" << arg << "', expected 'pr' or 'ab'." << std::endl;
            return 1;
        }
    }

    std::cout << "Starting left ankle diagnostic in "
              << (mode == AnkleMode::PR ? "PR" : "AB")
              << " mode. Press Ctrl+C to stop." << std::endl;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    LeftAnkleDiagnostic diag(network, mode);
    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    diag.stop();
}
