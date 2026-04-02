#pragma once

#include <array>
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
        static constexpr int kJointDim = 24;
        static constexpr int kQposDim = 31;
        static constexpr int kKptCount = 14;
        static constexpr uint32_t kCacheVersion = 4;

        // A compact runtime cache generated from the original NPZ track.
        struct Header
        {
            char magic[8];
            uint32_t version;
            uint32_t array_count;
        };

        ReferenceLoader(const std::filesystem::path& motion_file, float fps);

        void reset(const Eigen::VectorXf& default_joint_pos);
        void update(float time_s,
                    bool has_current_root_xy,
                    const Eigen::Vector2f& current_root_xy,
                    float current_root_yaw);

        const Eigen::VectorXf& joint_pos_rel() const { return joint_pos_rel_; }
        float root_height() const { return root_height_; }
        const Eigen::Vector3f& root_gravity() const { return root_gravity_; }
        const Eigen::Matrix<float, 6, 1>& root_cvel_in_gv() const { return root_cvel_in_gv_; }
        const Eigen::Vector2f& yaw_cmd() const { return yaw_cmd_; }
        const Eigen::Vector2f& xy_cmd() const { return xy_cmd_; }
        float duration() const { return duration_; }

    private:
        std::filesystem::path ensure_cache_file(const std::filesystem::path& motion_file) const;
        void load_cache_file(const std::filesystem::path& cache_file);

        float fps_ = 50.0f;
        float duration_ = 0.0f;

        std::vector<float> qpos_seq_;
        std::vector<float> kpt2gv_pose_seq_;
        std::vector<float> kpt_cvel_seq_;

        Eigen::VectorXf default_joint_pos_;
        Eigen::VectorXf joint_pos_rel_;
        Eigen::Vector3f root_gravity_ = Eigen::Vector3f::Zero();
        Eigen::Matrix<float, 6, 1> root_cvel_in_gv_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Vector2f yaw_cmd_ = Eigen::Vector2f::Zero();
        Eigen::Vector2f xy_cmd_ = Eigen::Vector2f::Zero();
        float root_height_ = 0.0f;
        size_t frame_count_ = 0;

        float wrap_to_pi(float angle) const;
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
