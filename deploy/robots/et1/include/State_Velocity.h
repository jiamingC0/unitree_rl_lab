#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"

class State_Velocity : public FSMState
{
public:
    State_Velocity(int state_mode, std::string state_string = "Velocity");

    void enter();
    void run();
    void exit();

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::vector<float> policy_kp_;
    std::vector<float> policy_kd_;

    std::thread policy_thread_;
    bool policy_thread_running_ = false;
};

REGISTER_FSM(State_Velocity)
