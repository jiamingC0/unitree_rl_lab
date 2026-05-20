#include "State_JointStepTest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_map>

#include <spdlog/spdlog.h>
#include <unitree/common/time/time_tool.hpp>

namespace
{
double now_s()
{
    return static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1e-3;
}

std::string timestamp_string()
{
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream ss;
    ss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return ss.str();
}

template <typename T>
std::vector<T> yaml_vector_or(const YAML::Node& node, const std::vector<T>& fallback)
{
    return node ? node.as<std::vector<T>>() : fallback;
}
}

State_JointStepTest::State_JointStepTest(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    if (!cfg) {
        throw std::runtime_error("JointStepTest: missing FSM config.");
    }

    const std::string xml_file = cfg["xml_file"] ? cfg["xml_file"].as<std::string>() : "";
    const auto xml_path = resolve_xml_path(xml_file);
    joint_limits_ = load_joint_limits(xml_path);

    joint_sdk_slots_ = cfg["joint_sdk_slots"]
        ? cfg["joint_sdk_slots"].as<std::vector<int>>()
        : std::vector<int>{0, 1, 2, 3, 4, 5,
                           6, 7, 8, 9, 10, 11,
                           12, 13,
                           15, 16, 17, 18, 19,
                           22, 23, 24, 25, 26,
                           29, 30};
    if (joint_limits_.size() < joint_sdk_slots_.size()) {
        throw std::runtime_error("JointStepTest: XML has fewer ranged joints than joint_sdk_slots.");
    }
    if (joint_sdk_slots_.size() != 26) {
        throw std::runtime_error("JointStepTest: expected exactly 26 joint SDK slots.");
    }

    kp_ = yaml_vector_or<float>(cfg["kp"], param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>());
    kd_ = yaml_vector_or<float>(cfg["kd"], param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>());
    return_speed_ = cfg["return_speed"] ? cfg["return_speed"].as<float>() : 0.2f;
    zero_hold_s_ = cfg["zero_hold_s"] ? cfg["zero_hold_s"].as<float>() : 1.0f;
    hold_upper_s_ = cfg["hold_upper_s"] ? cfg["hold_upper_s"].as<float>() : 2.0f;
    hold_lower_s_ = cfg["hold_lower_s"] ? cfg["hold_lower_s"].as<float>() : 2.0f;
    hold_final_zero_s_ = cfg["hold_final_zero_s"] ? cfg["hold_final_zero_s"].as<float>() : 2.0f;
    log_dir_ = cfg["log_dir"]
        ? std::filesystem::path(cfg["log_dir"].as<std::string>())
        : std::filesystem::path("debug/joint_step_test");
    if (log_dir_.is_relative()) {
        log_dir_ = param::proj_dir / log_dir_;
    }

    if (return_speed_ <= 0.0f) {
        throw std::runtime_error("JointStepTest: return_speed must be positive.");
    }

    spdlog::info("JointStepTest: loaded {} joint limits from '{}'",
                 joint_limits_.size(), xml_path.string());
}

std::filesystem::path State_JointStepTest::resolve_xml_path(const std::string& configured_path) const
{
    if (configured_path.empty()) {
        throw std::runtime_error("JointStepTest: xml_file is required.");
    }

    std::filesystem::path path(configured_path);
    if (path.is_absolute() && std::filesystem::exists(path)) {
        return path;
    }

    std::vector<std::filesystem::path> candidates = {
        param::proj_dir / path,
        param::config_dir / path,
        path,
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }

    throw std::runtime_error("JointStepTest: XML file not found: " + configured_path);
}

std::vector<State_JointStepTest::JointLimit> State_JointStepTest::load_joint_limits(
    const std::filesystem::path& xml_path) const
{
    std::ifstream in(xml_path);
    if (!in) {
        throw std::runtime_error("JointStepTest: failed to open XML file: " + xml_path.string());
    }

    std::vector<JointLimit> limits;
    std::string line;
    const std::regex attr_re(R"xmlattr(([A-Za-z_][A-Za-z0-9_]*)="([^"]*)")xmlattr");
    while (std::getline(in, line)) {
        if (line.find("<joint") == std::string::npos || line.find("range=") == std::string::npos) {
            continue;
        }

        std::unordered_map<std::string, std::string> attrs;
        for (std::sregex_iterator it(line.begin(), line.end(), attr_re), end; it != end; ++it) {
            attrs[(*it)[1].str()] = (*it)[2].str();
        }

        const auto name_it = attrs.find("name");
        const auto range_it = attrs.find("range");
        if (name_it == attrs.end() || range_it == attrs.end()) {
            continue;
        }

        std::istringstream range_stream(range_it->second);
        JointLimit limit;
        limit.name = name_it->second;
        if (range_stream >> limit.lower >> limit.upper) {
            limits.push_back(limit);
        }
    }

    if (limits.empty()) {
        throw std::runtime_error("JointStepTest: no ranged joints found in XML.");
    }
    return limits;
}

int State_JointStepTest::ask_joint_index() const
{
    while (true) {
        std::cout << "\nJointStepTest: input joint id [1-26], or 0 to cancel:\n";
        for (size_t i = 0; i < joint_sdk_slots_.size(); ++i) {
            std::cout << "  " << (i + 1) << ": " << joint_limits_[i].name
                      << "  range [" << joint_limits_[i].lower << ", "
                      << joint_limits_[i].upper << "]  sdk_slot "
                      << joint_sdk_slots_[i] << "\n";
        }
        const std::string input = keyboard ? keyboard->getString("JointStepTest> ") : "";
        try {
            const int value = std::stoi(input);
            if (value == 0) {
                return -1;
            }
            if (value >= 1 && value <= static_cast<int>(joint_sdk_slots_.size())) {
                return value - 1;
            }
        } catch (const std::exception&) {
        }
        std::cout << "JointStepTest: invalid input '" << input << "'.\n";
    }
}

float State_JointStepTest::ramp(float from, float to, double elapsed_s) const
{
    const float distance = to - from;
    const float duration = std::fabs(distance) / return_speed_;
    if (duration <= 1e-6f || elapsed_s >= duration) {
        return to;
    }
    return from + distance * static_cast<float>(elapsed_s / duration);
}

const char* State_JointStepTest::phase_name(Phase phase) const
{
    switch (phase) {
    case Phase::Idle: return "idle";
    case Phase::ToZero: return "to_zero";
    case Phase::HoldZero: return "hold_zero";
    case Phase::StepTarget: return "step_target";
    case Phase::HoldTarget: return "hold_target";
    case Phase::StepZero: return "step_zero";
    case Phase::HoldStepZero: return "hold_step_zero";
    }
    return "unknown";
}

std::vector<float> State_JointStepTest::build_target_sequence(const JointLimit& limit) const
{
    const auto clamp = [&](float q) {
        return std::clamp(q, limit.lower, limit.upper);
    };

    const std::string& name = limit.name;
    std::vector<float> targets;
    if (name.find("_hip_") != std::string::npos) {
        targets = {0.5f, 0.0f, 1.0f, -0.5f, 0.0f};
    } else if (name.find("_knee_joint") != std::string::npos) {
        targets = {0.5f, 0.0f, 1.0f, 0.5f, 0.0f};
    } else if (name.find("_ankle_pitch_joint") != std::string::npos) {
        targets = {-0.4f, 0.0f, 0.3f, 0.0f};
    } else if (name.find("_ankle_roll_joint") != std::string::npos) {
        targets = {-0.1f, 0.0f, 0.1f, 0.0f};
    } else if (name == "waist_roll_joint") {
        targets = {-0.2f, 0.0f, 0.3f, 0.0f};
    } else if (name == "waist_yaw_joint") {
        targets = {-0.3f, 0.0f, 0.4f, 0.0f};
    } else if (name.find("_shoulder_pitch_joint") != std::string::npos) {
        targets = {-0.5f, 0.0f, 0.5f, 0.0f};
    } else if (name.find("_shoulder_roll_joint") != std::string::npos) {
        targets = {-0.3f, 0.0f, 0.3f, 0.0f};
    } else if (name.find("_shoulder_yaw_joint") != std::string::npos) {
        targets = {-0.4f, 0.0f, 0.4f, 0.0f};
    } else if (name.find("_elbow_joint") != std::string::npos) {
        targets = {0.5f, 0.0f, 1.0f, 0.5f, 0.0f};
    } else if (name.find("_wrist_roll_joint") != std::string::npos) {
        targets = {-0.4f, 0.0f, 0.4f, 0.0f};
    } else if (name.find("head_") == 0) {
        targets = {-0.2f, 0.0f, 0.2f, 0.0f};
    } else {
        targets = {-0.3f, 0.0f, 0.3f, 0.0f};
    }

    for (float& target : targets) {
        target = clamp(target);
    }
    return targets;
}

float State_JointStepTest::target_hold_duration() const
{
    if (target_sequence_.empty() || target_index_ < 0 ||
        target_index_ >= static_cast<int>(target_sequence_.size())) {
        return hold_final_zero_s_;
    }

    const bool is_zero_target = std::fabs(target_sequence_[target_index_]) < 1e-6f;
    const bool is_final_target = target_index_ + 1 >= static_cast<int>(target_sequence_.size());
    if (is_final_target) {
        return hold_final_zero_s_;
    }
    return is_zero_target ? zero_hold_s_ : hold_upper_s_;
}

std::filesystem::path State_JointStepTest::make_log_path() const
{
    return log_dir_ / ("joint_step_test_" + timestamp_string() + ".csv");
}

void State_JointStepTest::open_log()
{
    if (log_stream_.is_open()) {
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    if (ec) {
        throw std::runtime_error(
            "JointStepTest: failed to create CSV log dir '" + log_dir_.string() +
            "': " + ec.message());
    }
    log_path_ = make_log_path();
    log_stream_.open(log_path_);
    if (!log_stream_) {
        throw std::runtime_error("JointStepTest: failed to open CSV log file '" + log_path_.string() + "'");
    }

    log_start_time_ = now_s();
    log_stream_ << std::setprecision(9);
    log_stream_ << "time_s,round,joint_id,joint_name,sdk_slot,phase,"
                << "target_index,target_q,target_count,"
                << "q_cmd,q_actual,dq_actual,lower,upper\n";
}

void State_JointStepTest::log_sample(float q_cmd)
{
    if (!log_stream_.is_open() || selected_joint_ < 0 || selected_sdk_slot_ < 0) {
        return;
    }

    const auto& limit = joint_limits_[selected_joint_];
    const auto& motor_state = lowstate->msg_.motor_state()[selected_sdk_slot_];
    const int csv_target_index = target_index_ + 1;
    const float target_q = (target_index_ >= 0 && target_index_ < static_cast<int>(target_sequence_.size()))
        ? target_sequence_[target_index_]
        : q_cmd;
    log_stream_ << (now_s() - log_start_time_) << ","
                << round_ << ","
                << (selected_joint_ + 1) << ","
                << limit.name << ","
                << selected_sdk_slot_ << ","
                << phase_name(phase_) << ","
                << csv_target_index << ","
                << target_q << ","
                << target_sequence_.size() << ","
                << q_cmd << ","
                << motor_state.q() << ","
                << motor_state.dq() << ","
                << limit.lower << ","
                << limit.upper << "\n";
}

void State_JointStepTest::close_log()
{
    if (log_stream_.is_open()) {
        log_stream_.flush();
        log_stream_.close();
        spdlog::info("JointStepTest: closed log '{}'", log_path_.string());
    }
}

void State_JointStepTest::select_next_joint_after_zero()
{
    selected_joint_ = ask_joint_index();
    if (selected_joint_ < 0) {
        selected_sdk_slot_ = -1;
        phase_ = Phase::Idle;
        close_log();
        spdlog::info("JointStepTest: cancelled after returning to zero, holding current posture.");
        return;
    }

    selected_sdk_slot_ = joint_sdk_slots_[selected_joint_];
    if (selected_sdk_slot_ < 0 ||
        selected_sdk_slot_ >= static_cast<int>(lowcmd->msg_.motor_cmd().size())) {
        throw std::runtime_error("JointStepTest: selected SDK slot is out of motor_cmd range.");
    }

    const auto& limit = joint_limits_[selected_joint_];
    phase_ = Phase::ToZero;
    last_reported_phase_ = Phase::Idle;
    phase_start_time_ = now_s();
    phase_start_q_ = hold_q_[selected_sdk_slot_];
    phase_target_q_ = 0.0f;
    target_index_ = 0;
    target_sequence_ = build_target_sequence(limit);
    open_log();
    spdlog::info("JointStepTest: selected joint {} '{}' sdk_slot {} range [{:.4f}, {:.4f}], targets {}, csv '{}'",
                 selected_joint_ + 1, limit.name, selected_sdk_slot_,
                 limit.lower, limit.upper, target_sequence_.size(), log_path_.string());
}

void State_JointStepTest::enter()
{
    hold_q_.assign(lowcmd->msg_.motor_cmd().size(), 0.0f);
    for (size_t i = 0; i < hold_q_.size(); ++i) {
        hold_q_[i] = lowstate->msg_.motor_state()[i].q();
    }

    selected_joint_ = ask_joint_index();
    if (selected_joint_ < 0) {
        selected_sdk_slot_ = -1;
        phase_ = Phase::Idle;
        spdlog::info("JointStepTest: cancelled, holding current posture.");
        return;
    }

    selected_sdk_slot_ = joint_sdk_slots_[selected_joint_];
    if (selected_sdk_slot_ < 0 ||
        selected_sdk_slot_ >= static_cast<int>(lowcmd->msg_.motor_cmd().size())) {
        throw std::runtime_error("JointStepTest: selected SDK slot is out of motor_cmd range.");
    }

    for (size_t i = 0; i < hold_q_.size(); ++i) {
        auto& motor = lowcmd->msg_.motor_cmd()[i];
        motor.mode() = 1;
        motor.q() = hold_q_[i];
        motor.dq() = 0.0f;
        motor.tau() = 0.0f;
        if (i < kp_.size()) {
            motor.kp() = kp_[i];
        }
        if (i < kd_.size()) {
            motor.kd() = kd_[i];
        }
    }

    const auto& limit = joint_limits_[selected_joint_];
    phase_ = Phase::ToZero;
    last_reported_phase_ = Phase::Idle;
    phase_start_time_ = now_s();
    phase_start_q_ = hold_q_[selected_sdk_slot_];
    phase_target_q_ = 0.0f;
    target_index_ = 0;
    target_sequence_ = build_target_sequence(limit);
    round_ = 0;
    open_log();

    spdlog::info("JointStepTest: selected joint {} '{}' sdk_slot {} range [{:.4f}, {:.4f}], targets {}, csv '{}'",
                 selected_joint_ + 1, limit.name, selected_sdk_slot_,
                 limit.lower, limit.upper, target_sequence_.size(), log_path_.string());
}

void State_JointStepTest::run()
{
    if (hold_q_.empty()) {
        return;
    }

    for (size_t i = 0; i < hold_q_.size(); ++i) {
        auto& motor = lowcmd->msg_.motor_cmd()[i];
        motor.mode() = 1;
        motor.q() = hold_q_[i];
        motor.dq() = 0.0f;
        motor.tau() = 0.0f;
        if (i < kp_.size()) {
            motor.kp() = kp_[i];
        }
        if (i < kd_.size()) {
            motor.kd() = kd_[i];
        }
    }

    if (selected_joint_ < 0 || selected_sdk_slot_ < 0 || phase_ == Phase::Idle) {
        return;
    }

    if (phase_ != last_reported_phase_) {
        spdlog::info("JointStepTest: phase {}", phase_name(phase_));
        last_reported_phase_ = phase_;
    }

    const double elapsed = now_s() - phase_start_time_;
    float q = hold_q_[selected_sdk_slot_];

    if (phase_ == Phase::ToZero) {
        q = ramp(phase_start_q_, phase_target_q_, elapsed);
        if (std::fabs(q - phase_target_q_) < 1e-5f) {
            q = phase_target_q_;
            phase_ = Phase::HoldZero;
            phase_start_time_ = now_s();
        }
    } else if (phase_ == Phase::HoldZero) {
        q = 0.0f;
        if (elapsed >= zero_hold_s_) {
            phase_ = Phase::StepTarget;
        }
    } else if (phase_ == Phase::StepTarget) {
        if (target_sequence_.empty() || target_index_ >= static_cast<int>(target_sequence_.size())) {
            select_next_joint_after_zero();
            return;
        }
        q = target_sequence_[target_index_];
        round_ = target_index_ + 1;
        phase_ = Phase::HoldTarget;
        phase_start_time_ = now_s();
    } else if (phase_ == Phase::HoldTarget) {
        q = target_sequence_[target_index_];
        if (elapsed >= target_hold_duration()) {
            if (target_index_ + 1 >= static_cast<int>(target_sequence_.size())) {
                hold_q_[selected_sdk_slot_] = q;
                lowcmd->msg_.motor_cmd()[selected_sdk_slot_].q() = q;
                log_sample(q);
                select_next_joint_after_zero();
                return;
            }
            ++target_index_;
            phase_ = Phase::StepTarget;
        }
    } else if (phase_ == Phase::StepZero) {
        q = 0.0f;
        phase_ = Phase::HoldStepZero;
        phase_start_time_ = now_s();
    } else if (phase_ == Phase::HoldStepZero) {
        q = 0.0f;
        const bool final_target = target_index_ >= 2;
        const float hold_s = final_target ? hold_final_zero_s_ : zero_hold_s_;
        if (elapsed >= hold_s) {
            if (final_target) {
                hold_q_[selected_sdk_slot_] = 0.0f;
                lowcmd->msg_.motor_cmd()[selected_sdk_slot_].q() = 0.0f;
                log_sample(0.0f);
                select_next_joint_after_zero();
                return;
            }
            ++target_index_;
            phase_ = Phase::StepTarget;
        }
    }

    hold_q_[selected_sdk_slot_] = q;
    lowcmd->msg_.motor_cmd()[selected_sdk_slot_].q() = q;
    log_sample(q);
}

void State_JointStepTest::exit()
{
    close_log();
    selected_joint_ = -1;
    selected_sdk_slot_ = -1;
    phase_ = Phase::Idle;
}
