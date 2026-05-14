#include "State_Velocity.h"

#include <algorithm>
#include <unordered_map>

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
