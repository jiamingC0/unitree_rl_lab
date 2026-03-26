#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto register_keyboard_transition = [&](const std::string& key, const std::string& target_fsm) {
            if (!keyboard || !FSMStringMap.right.count(target_fsm)) {
                return;
            }

            int fsm_id = FSMStringMap.right.at(target_fsm);
            registered_checks.emplace_back(
                std::make_pair(
                    [key]()->bool {
                        return keyboard && keyboard->on_pressed && keyboard->key() == key;
                    },
                    fsm_id
                )
            );
        };

        // Keyboard fallback for Sim2Sim when no joystick is available.
        if (state_string == "Passive") {
            register_keyboard_transition("1", "FixStand");
        } else {
            register_keyboard_transition("0", "Passive");
        }

        if (state_string == "FixStand") {
            if (FSMStringMap.right.count("Velocity")) {
                register_keyboard_transition("2", "Velocity");
            } else if (FSMStringMap.right.count("Track")) {
                register_keyboard_transition("2", "Track");
            }
        } else if (state_string == "Velocity") {
            register_keyboard_transition("3", "Mimic_Dance_102");
            register_keyboard_transition("4", "Mimic_Gangnam_Style");
        } else if (state_string == "Mimic_Dance_102" || state_string == "Mimic_Gangnam_Style") {
            register_keyboard_transition("2", "Velocity");
        }

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
