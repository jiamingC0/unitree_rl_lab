// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include <chrono>
#include "BaseState.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        currentState->enter();
        last_state_run_time_ = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(currentState->run_dt()));

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        // Stop the recurrent FSM loop before tearing down state objects.
        fsm_thread_.reset();

        // Ensure the active state's background resources (e.g. Track policy thread)
        // are shut down even when the process exits via Ctrl+C without a state switch.
        if (currentState) {
            currentState->exit();
            currentState.reset();
        }

        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        currentState->pre_run();
        const auto now = std::chrono::steady_clock::now();
        const auto desired_dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(currentState->run_dt()));
        if (now - last_state_run_time_ >= desired_dt) {
            currentState->run();
            currentState->post_run();
            last_state_run_time_ = now;
        }
        
        // Check if need to change state
        int nextStateMode = 0;
        std::string transition_reason;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].condition())
            {
                nextStateMode = currentState->registered_checks[i].target_state;
                transition_reason = currentState->registered_checks[i].reason;
                break;
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    if (transition_reason.empty()) {
                        spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    } else {
                        spdlog::info("FSM: Change state from {} to {} due to {}",
                                     currentState->getStateString(),
                                     state->getStateString(),
                                     transition_reason);
                    }
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    last_state_run_time_ = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(currentState->run_dt()));
                    break;
                }
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    std::chrono::steady_clock::time_point last_state_run_time_;
    unitree::common::RecurrentThreadPtr fsm_thread_;
};
