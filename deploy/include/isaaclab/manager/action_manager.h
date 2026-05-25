// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/manager/manager_term_cfg.h"
#include <algorithm>
#include <deque>
#include <numeric>
#include <random>
#include <stdexcept>

namespace isaaclab
{

class ActionTerm 
{
public:
    ActionTerm(YAML::Node cfg, ManagerBasedRLEnv* env): cfg(cfg), env(env) {}

    virtual int action_dim() = 0;
    virtual std::vector<float> raw_actions() = 0;
    virtual std::vector<float> processed_actions() = 0;
    virtual void process_actions(std::vector<float> actions) = 0;
    virtual void reset(){};

protected:
    YAML::Node cfg;
    ManagerBasedRLEnv* env;
};

inline std::map<std::string, std::function<std::unique_ptr<ActionTerm>(YAML::Node, ManagerBasedRLEnv*)>>& actions_map() {
    static std::map<std::string, std::function<std::unique_ptr<ActionTerm>(YAML::Node, ManagerBasedRLEnv*)>> instance;
    return instance;
}

#define REGISTER_ACTION(name) \
    inline struct name##_registrar { \
        name##_registrar() { \
            actions_map()[#name] = [](YAML::Node cfg, ManagerBasedRLEnv* env) { \
                return std::make_unique<name>(cfg, env); \
            }; \
        } \
    } name##_registrar_instance;

class ActionManager
{
public:
    ActionManager(YAML::Node cfg, ManagerBasedRLEnv* env, YAML::Node action_delay_cfg = YAML::Node())
    : cfg(cfg), env(env)
    {
        _prepare_terms();
        _action.resize(total_action_dim(), 0.0f);
        _configure_action_delay(action_delay_cfg);
    }

    void reset()
    {
        _action.assign(total_action_dim(), 0.0f);
        _reset_action_delay();
        for(auto & term : _terms)
        {
            term->reset();
        }
    }

    std::vector<float> action()
    {
        return _action;
    }

    std::vector<float> processed_actions()
    {
        std::vector<float> actions;
        for(auto & term : _terms)
        {
            auto term_action = term->processed_actions();
            actions.insert(actions.end(), term_action.begin(), term_action.end());
        }
        return actions;
    }

    void process_action(std::vector<float> action)
    {
        if (action.size() != static_cast<size_t>(total_action_dim()))
        {
            throw std::runtime_error("Action size mismatch: got " + std::to_string(action.size())
                + ", expected " + std::to_string(total_action_dim()));
        }
        action = _apply_action_delay(action);
        _action = action;
        int idx = 0;
        for(auto & term : _terms)
        {
            auto term_action = std::vector<float>(action.begin() + idx, action.begin() + idx + term->action_dim());
            term->process_actions(term_action);
            idx += term->action_dim();
        }
    }

    int total_action_dim()
    {
        auto dims = action_dim();
        
        return std::accumulate(dims.begin(), dims.end(), 0);
    }

    std::vector<int> action_dim()
    {
        std::vector<int> dims;
        for (auto & term : _terms)
        {
            dims.push_back(term->action_dim());
        }
        return dims;
    }

    YAML::Node cfg;
    ManagerBasedRLEnv* env;

private:
    void _configure_action_delay(YAML::Node delay_cfg)
    {
        if (!delay_cfg || !delay_cfg.IsMap())
        {
            return;
        }

        const bool enabled = delay_cfg["enabled"] ? delay_cfg["enabled"].as<bool>() : true;
        _max_delay_steps = delay_cfg["max_delay_steps"] ? delay_cfg["max_delay_steps"].as<int>() : 0;
        if (!enabled || _max_delay_steps <= 0)
        {
            _max_delay_steps = 0;
            _delay_steps = 0;
            return;
        }

        if (delay_cfg["delay_steps"])
        {
            _randomize_delay_on_reset = false;
            _delay_steps = std::clamp(delay_cfg["delay_steps"].as<int>(), 0, _max_delay_steps);
        }
        else
        {
            _randomize_delay_on_reset = delay_cfg["randomize_on_reset"]
                ? delay_cfg["randomize_on_reset"].as<bool>()
                : true;
            _delay_steps = _randomize_delay_on_reset ? 0 : _max_delay_steps;
        }

        _reset_action_delay();
    }

    void _reset_action_delay()
    {
        _action_delay_buffer.clear();
        if (_max_delay_steps <= 0)
        {
            _delay_steps = 0;
            return;
        }

        if (_randomize_delay_on_reset)
        {
            std::uniform_int_distribution<int> dist(0, _max_delay_steps);
            _delay_steps = dist(_rng);
        }

        const std::vector<float> zero_action(total_action_dim(), 0.0f);
        for (int i = 0; i < _max_delay_steps + 1; ++i)
        {
            _action_delay_buffer.push_back(zero_action);
        }
    }

    std::vector<float> _apply_action_delay(const std::vector<float>& action)
    {
        if (_max_delay_steps <= 0)
        {
            return action;
        }

        if (_action_delay_buffer.empty())
        {
            _reset_action_delay();
        }

        _action_delay_buffer.push_back(action);
        while (static_cast<int>(_action_delay_buffer.size()) > _max_delay_steps + 1)
        {
            _action_delay_buffer.pop_front();
        }

        const int index = static_cast<int>(_action_delay_buffer.size()) - 1 - _delay_steps;
        return _action_delay_buffer[std::max(0, index)];
    }

    void _prepare_terms()
    {
        for(auto it = this->cfg.begin(); it != this->cfg.end(); ++it)
        {
            std::string action_name = it->first.as<std::string>();
            if(actions_map().find(action_name) == actions_map().end())
            {
                throw std::runtime_error("Action term '" + action_name + "' is not registered.");
            }

            auto term = actions_map()[action_name](it->second, env);
            _terms.push_back(std::move(term));
        }
    }

    std::vector<float> _action;
    std::vector<std::unique_ptr<ActionTerm>> _terms;
    int _max_delay_steps = 0;
    int _delay_steps = 0;
    bool _randomize_delay_on_reset = true;
    std::deque<std::vector<float>> _action_delay_buffer;
    std::mt19937 _rng{std::random_device{}()};
};

};
