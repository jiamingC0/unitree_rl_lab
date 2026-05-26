#include "State_Velocity.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "State_Track.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "unitree_articulation.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    static const std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}},
    };

    const auto it = key_commands.find(FSMState::keyboard->key());
    return it == key_commands.end() ? std::vector<float>{0.0f, 0.0f, 0.0f} : it->second;
}

}
}

namespace
{
std::string trim_copy(const std::string& value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string tracker_target_from_token(const std::string& token)
{
    static const std::unordered_map<std::string, std::string> aliases = {
        {"general", "GeneralTracker"},
        {"tracker", "GeneralTracker"},
        {"generaltracker", "GeneralTracker"},
        {"GeneralTracker", "GeneralTracker"},
        {"debug", "GeneralTrackerCJM"},
        {"cjm", "GeneralTrackerCJM"},
        {"general_tracker_cjm", "GeneralTrackerCJM"},
        {"generaltrackercjm", "GeneralTrackerCJM"},
        {"GeneralTrackerCJM", "GeneralTrackerCJM"},
        {"cln", "GeneralTrackerCLN"},
        {"general_tracker_cln", "GeneralTrackerCLN"},
        {"generaltrackercln", "GeneralTrackerCLN"},
        {"GeneralTrackerCLN", "GeneralTrackerCLN"},
    };
    const auto it = aliases.find(token);
    return it == aliases.end() ? "" : it->second;
}
}

bool State_Velocity::prepare_general_tracker_request()
{
    if (pending_tracker_target_state_) {
        return true;
    }
    if (State_Track::has_pending_motion_request()) {
        pending_tracker_target_state_ = "GeneralTracker";
        return true;
    }
    if (general_tracker_request_file_.empty()
        || !std::filesystem::exists(general_tracker_request_file_)) {
        return false;
    }

    std::ifstream input(general_tracker_request_file_);
    std::string line;
    std::getline(input, line);
    input.close();

    std::error_code ec;
    std::filesystem::remove(general_tracker_request_file_, ec);

    line = trim_copy(line);
    if (line.empty()) {
        spdlog::warn("Velocity: ignored empty GeneralTracker request file '{}'",
                     general_tracker_request_file_.string());
        return false;
    }

    std::istringstream ss(line);
    std::string first_token;
    ss >> first_token;
    std::string target_state = tracker_target_from_token(first_token);
    std::string motion_file;
    if (target_state.empty()) {
        target_state = "GeneralTracker";
        motion_file = line;
    } else {
        std::getline(ss, motion_file);
        motion_file = trim_copy(motion_file);
    }

    if (motion_file.empty()) {
        spdlog::warn("Velocity: ignored GeneralTracker request without motion path: '{}'", line);
        return false;
    }
    if (!FSMStringMap.right.count(target_state)) {
        spdlog::warn("Velocity: ignored GeneralTracker request for unavailable target '{}'", target_state);
        return false;
    }

    State_Track::request_motion_file(motion_file);
    pending_tracker_target_state_ = target_state;
    spdlog::info("Velocity: routed GeneralTracker request to {} with motion '{}'",
                 target_state,
                 motion_file);
    return true;
}

State_Velocity::State_Velocity(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const std::string deploy_file = cfg["deploy_file"] ? cfg["deploy_file"].as<std::string>() : "deploy.yaml";

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / deploy_file),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
            FSMState::lowstate, FSMState::highstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / policy_file);
    policy_kp_ = env->cfg["policy_kp"] ? env->cfg["policy_kp"].as<std::vector<float>>() : env->robot->data.joint_stiffness;
    policy_kd_ = env->cfg["policy_kd"] ? env->cfg["policy_kd"].as<std::vector<float>>() : env->robot->data.joint_damping;

    if (FSMStringMap.right.count("GeneralTracker")) {
        auto tracker_cfg = param::config["FSM"]["GeneralTracker"];
        const std::string request_file = tracker_cfg["request_file"]
            ? tracker_cfg["request_file"].as<std::string>()
            : "debug/general_tracker_request.txt";
        general_tracker_request_file_ = request_file;
        if (!general_tracker_request_file_.is_absolute()) {
            general_tracker_request_file_ = param::proj_dir / general_tracker_request_file_;
        }

        const std::vector<std::string> tracker_targets = {
            "GeneralTracker",
            "GeneralTrackerCJM",
            "GeneralTrackerCLN",
        };
        for (const auto& target_state : tracker_targets) {
            if (!FSMStringMap.right.count(target_state)) {
                continue;
            }
            registered_checks.push_back({
                [this, target_state]() -> bool {
                    if (!prepare_general_tracker_request()) {
                        return false;
                    }
                    if (pending_tracker_target_state_ != target_state) {
                        return false;
                    }
                    pending_tracker_target_state_.reset();
                    return true;
                },
                FSMStringMap.right.at(target_state),
                "external " + target_state + " motion request"
            });
        }
    }

    registered_checks.push_back({
        [&]() -> bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
        FSMStringMap.right.at("Passive"),
        "bad_orientation"
    });
}

void State_Velocity::enter()
{
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    const size_t joint_count = std::min({
        env->robot->data.joint_ids_map.size(),
        policy_kp_.size(),
        policy_kd_.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.dq() = 0.0f;
        motor.tau() = 0.0f;
    }

    env->robot->update();
    policy_thread_running_ = true;
    policy_thread_ = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(env->step_dt));

        auto sleep_till = clock::now() + dt;
        env->reset();

        while (policy_thread_running_) {
            env->step();
            std::this_thread::sleep_until(sleep_till);
            sleep_till += dt;
        }
    });
}

void State_Velocity::run()
{
    auto action = env->action_manager->processed_actions();
    const int motor_cmd_count = static_cast<int>(lowcmd->msg_.motor_cmd().size());
    const size_t joint_count = std::min({
        env->robot->data.joint_ids_map.size(),
        action.size(),
        policy_kp_.size(),
        policy_kd_.size()
    });

    for (size_t i = 0; i < joint_count; ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        if (sdk_slot < 0 || sdk_slot >= motor_cmd_count) {
            continue;
        }

        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = action[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.tau() = 0.0f;
    }
}

void State_Velocity::exit()
{
    policy_thread_running_ = false;
    if (policy_thread_.joinable()) {
        policy_thread_.join();
    }
}
