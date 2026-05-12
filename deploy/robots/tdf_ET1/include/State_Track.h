#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "FSM/State_RLBase.h"

class State_Track : public FSMState
{
public:
    class ReferenceLoader
    {
    public:
        static constexpr int kJointDim = 26;
        static constexpr int kBodyCount = 27;
        static constexpr int kAnchorBodyIndex = 14; // waist_yaw_link in tdf_ET1 motion npz.
        static constexpr uint32_t kCacheVersion = 1;

        struct Header
        {
            char magic[8];
            uint32_t version;
            uint32_t array_count;
        };

        ReferenceLoader(const std::filesystem::path& motion_file, float fps);

        void reset();
        void update(float time_s, const Eigen::VectorXf& robot_joint_pos, const Eigen::Quaternionf& robot_root_quat);

        const Eigen::VectorXf& command_joint_pos() const { return joint_pos_; }
        const Eigen::VectorXf& command_joint_vel() const { return joint_vel_; }
        const Eigen::Matrix<float, 6, 1>& motion_anchor_ori_b() const { return anchor_ori_b_; }
        float duration() const { return duration_; }

    private:
        std::filesystem::path ensure_cache_file(const std::filesystem::path& motion_file) const;
        void load_cache_file(const std::filesystem::path& cache_file);
        Eigen::Quaternionf anchor_quat_w(const Eigen::Quaternionf& root_quat, const Eigen::VectorXf& joint_pos) const;

        float fps_ = 50.0f;
        float duration_ = 0.0f;
        size_t frame_count_ = 0;

        std::vector<float> joint_pos_seq_;
        std::vector<float> joint_vel_seq_;
        std::vector<float> body_pos_w_seq_;
        std::vector<float> body_quat_w_seq_;
        std::vector<float> body_lin_vel_w_seq_;
        std::vector<float> body_ang_vel_w_seq_;

        Eigen::VectorXf joint_pos_;
        Eigen::VectorXf joint_vel_;
        Eigen::Matrix<float, 6, 1> anchor_ori_b_ = Eigen::Matrix<float, 6, 1>::Zero();
    };

    State_Track(int state_mode, std::string state_string = "Track");

    double run_dt() const override { return 0.02; }
    void enter();
    void run();
    void exit();

    static std::shared_ptr<ReferenceLoader> reference;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<ReferenceLoader> reference_;
    std::vector<float> policy_kp_;
    std::vector<float> policy_kd_;
};

REGISTER_FSM(State_Track)
