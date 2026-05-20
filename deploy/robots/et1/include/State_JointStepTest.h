#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "FSM/FSMState.h"

class State_JointStepTest : public FSMState
{
public:
    State_JointStepTest(int state_mode, std::string state_string = "JointStepTest");

    double run_dt() const override { return 0.002; }
    void enter() override;
    void run() override;
    void exit() override;

private:
    struct JointLimit
    {
        std::string name;
        float lower = 0.0f;
        float upper = 0.0f;
    };

    enum class Phase
    {
        Idle,
        ToZero,
        HoldZero,
        StepTarget,
        HoldTarget,
        StepZero,
        HoldStepZero,
    };

    std::filesystem::path resolve_xml_path(const std::string& configured_path) const;
    std::vector<JointLimit> load_joint_limits(const std::filesystem::path& xml_path) const;
    int ask_joint_index() const;
    float ramp(float from, float to, double elapsed_s) const;
    const char* phase_name(Phase phase) const;
    std::vector<float> build_target_sequence(const JointLimit& limit) const;
    float target_hold_duration() const;
    std::filesystem::path make_log_path() const;
    void open_log();
    void log_sample(float q_cmd);
    void close_log();
    void select_next_joint_after_zero();

    std::vector<JointLimit> joint_limits_;
    std::vector<int> joint_sdk_slots_;
    std::vector<float> kp_;
    std::vector<float> kd_;
    std::vector<float> hold_q_;

    int selected_joint_ = -1;
    int selected_sdk_slot_ = -1;
    Phase phase_ = Phase::Idle;
    Phase last_reported_phase_ = Phase::Idle;
    double phase_start_time_ = 0.0;
    float phase_start_q_ = 0.0f;
    float phase_target_q_ = 0.0f;
    int target_index_ = 0;
    std::vector<float> target_sequence_;
    float return_speed_ = 0.2f;
    float zero_hold_s_ = 1.0f;
    float hold_upper_s_ = 2.0f;
    float hold_lower_s_ = 2.0f;
    float hold_final_zero_s_ = 2.0f;
    int round_ = 0;

    std::filesystem::path log_dir_;
    std::filesystem::path log_path_;
    std::ofstream log_stream_;
    double log_start_time_ = 0.0;
};

REGISTER_FSM(State_JointStepTest)
