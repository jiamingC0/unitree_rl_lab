#include "State_JointTest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <iomanip>
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

State_JointTest::State_JointTest(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    if (!cfg) {
        throw std::runtime_error("JointTest: missing FSM config.");
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
        throw std::runtime_error("JointTest: XML has fewer ranged joints than joint_sdk_slots.");
    }
    if (joint_sdk_slots_.size() != 26) {
        throw std::runtime_error("JointTest: expected exactly 26 joint SDK slots.");
    }

    kp_ = yaml_vector_or<float>(cfg["kp"], param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>());
    kd_ = yaml_vector_or<float>(cfg["kd"], param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>());
    ramp_speed_ = cfg["ramp_speed"] ? cfg["ramp_speed"].as<float>() : 0.2f;
    hold_upper_s_ = cfg["hold_upper_s"] ? cfg["hold_upper_s"].as<float>() : 2.0f;
    hold_lower_s_ = cfg["hold_lower_s"] ? cfg["hold_lower_s"].as<float>() : 2.0f;
    log_dir_ = cfg["log_dir"]
        ? std::filesystem::path(cfg["log_dir"].as<std::string>())
        : std::filesystem::path("debug/joint_test");
    if (log_dir_.is_relative()) {
        log_dir_ = param::proj_dir / log_dir_;
    }

    if (ramp_speed_ <= 0.0f) {
        throw std::runtime_error("JointTest: ramp_speed must be positive.");
    }

    spdlog::info("JointTest: loaded {} joint limits from '{}'",
                 joint_limits_.size(), xml_path.string());
}

std::filesystem::path State_JointTest::resolve_xml_path(const std::string& configured_path) const
{
    if (configured_path.empty()) {
        throw std::runtime_error("JointTest: xml_file is required.");
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

    throw std::runtime_error("JointTest: XML file not found: " + configured_path);
}

std::vector<State_JointTest::JointLimit> State_JointTest::load_joint_limits(
    const std::filesystem::path& xml_path) const
{
    std::ifstream in(xml_path);
    if (!in) {
        throw std::runtime_error("JointTest: failed to open XML file: " + xml_path.string());
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
        throw std::runtime_error("JointTest: no ranged joints found in XML.");
    }
    return limits;
}

int State_JointTest::ask_joint_index() const
{
    while (true) {
        std::cout << "\nJointTest: input joint id [1-26], or 0 to cancel:\n";
        for (size_t i = 0; i < joint_sdk_slots_.size(); ++i) {
            std::cout << "  " << (i + 1) << ": " << joint_limits_[i].name
                      << "  range [" << joint_limits_[i].lower << ", "
                      << joint_limits_[i].upper << "]  sdk_slot "
                      << joint_sdk_slots_[i] << "\n";
        }
        const std::string input = keyboard ? keyboard->getString("JointTest> ") : "";
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
        std::cout << "JointTest: invalid input '" << input << "'.\n";
    }
}

float State_JointTest::ramp(float from, float to, double elapsed_s) const
{
    const float distance = to - from;
    const float duration = std::fabs(distance) / ramp_speed_;
    if (duration <= 1e-6f || elapsed_s >= duration) {
        return to;
    }
    return from + distance * static_cast<float>(elapsed_s / duration);
}

const char* State_JointTest::phase_name(Phase phase) const
{
    switch (phase) {
    case Phase::Idle: return "idle";
    case Phase::ToZero: return "to_zero";
    case Phase::ToUpper: return "to_upper";
    case Phase::HoldUpper: return "hold_upper";
    case Phase::ToLower: return "to_lower";
    case Phase::HoldLower: return "hold_lower";
    }
    return "unknown";
}

std::filesystem::path State_JointTest::make_log_path() const
{
    const std::string joint_name = selected_joint_ >= 0 ? joint_limits_[selected_joint_].name : "unknown";
    return log_dir_ / ("joint_test_" + timestamp_string() + "_" + joint_name + ".csv");
}

void State_JointTest::open_log()
{
    if (selected_joint_ < 0) {
        return;
    }
    std::filesystem::create_directories(log_dir_);
    log_path_ = make_log_path();
    log_stream_.open(log_path_);
    if (!log_stream_) {
        spdlog::warn("JointTest: failed to open log file '{}'", log_path_.string());
        return;
    }

    log_stream_ << "time_s,round,joint_id,joint_name,sdk_slot,phase,"
                << "q_cmd,q_actual,dq_actual,lower,upper\n";
    spdlog::info("JointTest: logging to '{}'", log_path_.string());
}

void State_JointTest::log_sample(float q_cmd)
{
    if (!log_stream_ || selected_joint_ < 0 || selected_sdk_slot_ < 0) {
        return;
    }

    const auto& limit = joint_limits_[selected_joint_];
    const auto& motor_state = lowstate->msg_.motor_state()[selected_sdk_slot_];
    log_stream_ << now_s() << ","
                << round_ << ","
                << (selected_joint_ + 1) << ","
                << limit.name << ","
                << selected_sdk_slot_ << ","
                << phase_name(phase_) << ","
                << q_cmd << ","
                << motor_state.q() << ","
                << motor_state.dq() << ","
                << limit.lower << ","
                << limit.upper << "\n";
}

void State_JointTest::close_log()
{
    if (log_stream_) {
        log_stream_.flush();
        log_stream_.close();
        spdlog::info("JointTest: closed log '{}'", log_path_.string());
    }
}

void State_JointTest::enter()
{
    hold_q_.assign(lowcmd->msg_.motor_cmd().size(), 0.0f);
    for (size_t i = 0; i < hold_q_.size(); ++i) {
        hold_q_[i] = lowstate->msg_.motor_state()[i].q();
    }

    selected_joint_ = ask_joint_index();
    if (selected_joint_ < 0) {
        selected_sdk_slot_ = -1;
        phase_ = Phase::Idle;
        spdlog::info("JointTest: cancelled, holding current posture.");
        return;
    }

    selected_sdk_slot_ = joint_sdk_slots_[selected_joint_];
    if (selected_sdk_slot_ < 0 ||
        selected_sdk_slot_ >= static_cast<int>(lowcmd->msg_.motor_cmd().size())) {
        throw std::runtime_error("JointTest: selected SDK slot is out of motor_cmd range.");
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
    round_ = 0;
    prompt_after_zero_ = false;
    open_log();

    spdlog::info("JointTest: selected joint {} '{}' sdk_slot {} range [{:.4f}, {:.4f}]",
                 selected_joint_ + 1, limit.name, selected_sdk_slot_,
                 limit.lower, limit.upper);
}

void State_JointTest::run()
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
        spdlog::info("JointTest: phase {}", phase_name(phase_));
        last_reported_phase_ = phase_;
    }

    const auto& limit = joint_limits_[selected_joint_];
    const double elapsed = now_s() - phase_start_time_;
    float q = hold_q_[selected_sdk_slot_];

    if (phase_ == Phase::ToZero || phase_ == Phase::ToUpper || phase_ == Phase::ToLower) {
        q = ramp(phase_start_q_, phase_target_q_, elapsed);
        if (std::fabs(q - phase_target_q_) < 1e-5f) {
            q = phase_target_q_;
            if (phase_ == Phase::ToZero) {
                hold_q_[selected_sdk_slot_] = 0.0f;
                if (prompt_after_zero_) {
                    lowcmd->msg_.motor_cmd()[selected_sdk_slot_].q() = 0.0f;
                    log_sample(0.0f);
                    prompt_after_zero_ = false;
                    selected_joint_ = ask_joint_index();
                    if (selected_joint_ < 0) {
                        selected_sdk_slot_ = -1;
                        phase_ = Phase::Idle;
                        spdlog::info("JointTest: cancelled after returning to zero, holding current posture.");
                    } else {
                        selected_sdk_slot_ = joint_sdk_slots_[selected_joint_];
                        const auto& next_limit = joint_limits_[selected_joint_];
                        phase_ = Phase::ToZero;
                        last_reported_phase_ = Phase::Idle;
                        phase_start_q_ = hold_q_[selected_sdk_slot_];
                        phase_target_q_ = 0.0f;
                        phase_start_time_ = now_s();
                        spdlog::info("JointTest: selected joint {} '{}' sdk_slot {} range [{:.4f}, {:.4f}]",
                                     selected_joint_ + 1, next_limit.name, selected_sdk_slot_,
                                     next_limit.lower, next_limit.upper);
                    }
                    return;
                } else {
                    phase_ = Phase::ToUpper;
                    phase_start_q_ = 0.0f;
                    phase_target_q_ = limit.upper;
                    phase_start_time_ = now_s();
                    ++round_;
                }
            } else if (phase_ == Phase::ToUpper) {
                phase_ = Phase::HoldUpper;
                phase_start_q_ = limit.upper;
                phase_target_q_ = limit.upper;
                phase_start_time_ = now_s();
            } else {
                phase_ = Phase::HoldLower;
                phase_start_q_ = limit.lower;
                phase_target_q_ = limit.lower;
                phase_start_time_ = now_s();
            }
        }
    } else if (phase_ == Phase::HoldUpper) {
        q = limit.upper;
        if (elapsed >= hold_upper_s_) {
            phase_ = Phase::ToLower;
            phase_start_q_ = limit.upper;
            phase_target_q_ = limit.lower;
            phase_start_time_ = now_s();
        }
    } else if (phase_ == Phase::HoldLower) {
        q = limit.lower;
        if (elapsed >= hold_lower_s_) {
            phase_ = Phase::ToZero;
            phase_start_q_ = limit.lower;
            phase_target_q_ = 0.0f;
            phase_start_time_ = now_s();
            prompt_after_zero_ = true;
        }
    }

    hold_q_[selected_sdk_slot_] = q;
    lowcmd->msg_.motor_cmd()[selected_sdk_slot_].q() = q;
    log_sample(q);
}

void State_JointTest::exit()
{
    close_log();
    selected_joint_ = -1;
    selected_sdk_slot_ = -1;
    phase_ = Phase::Idle;
    prompt_after_zero_ = false;
}
