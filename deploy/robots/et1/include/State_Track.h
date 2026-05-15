#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
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
        static constexpr uint32_t kCacheVersion = 1;

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
                    bool no_global_mode,
                    bool has_current_root_xy,
                    const Eigen::Vector2f& current_root_xy,
                    float current_root_yaw,
                    const Eigen::Quaternionf& current_root_quat,
                    bool use_motion_root_command = true,
                    bool use_motion_velocity_command = true);

        const Eigen::VectorXf& command_joint_pos() const { return joint_pos_; }
        const Eigen::VectorXf& command_joint_vel() const { return joint_vel_; }
        const Eigen::Matrix<float, 6, 1>& command_root_ori_b() const { return root_ori_b_; }
        const Eigen::Vector3f& command_xy_yaw_vel() const { return xy_yaw_vel_; }
        const Eigen::Matrix<float, 6, 1>& command_foot_support_state() const { return foot_support_state_; }
        float duration() const { return duration_; }

    private:
        std::filesystem::path ensure_cache_file(const std::filesystem::path& motion_file) const;
        void load_cache_file(const std::filesystem::path& cache_file);

        float fps_ = 50.0f;
        float duration_ = 0.0f;

        std::vector<float> joint_pos_seq_;
        std::vector<float> joint_vel_seq_;
        std::vector<float> body_pos_w_seq_;
        std::vector<float> body_quat_w_seq_;
        std::vector<float> body_lin_vel_w_seq_;
        std::vector<float> body_ang_vel_w_seq_;
        std::vector<int64_t> left_foot_contact_state_seq_;
        std::vector<int64_t> right_foot_contact_state_seq_;

        Eigen::VectorXf default_joint_pos_;
        Eigen::VectorXf joint_pos_;
        Eigen::VectorXf joint_vel_;
        Eigen::Matrix<float, 6, 1> root_ori_b_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Vector3f xy_yaw_vel_ = Eigen::Vector3f::Zero();
        Eigen::Matrix<float, 6, 1> foot_support_state_ = Eigen::Matrix<float, 6, 1>::Zero();
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
    void dump_first_frame_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                                const std::vector<float>& action,
                                const std::vector<float>& target_q);
    void write_npy_float(const std::filesystem::path& path,
                         const std::vector<float>& data,
                         const std::vector<size_t>& shape) const;
    void write_npy_int64(const std::filesystem::path& path,
                         const std::vector<int64_t>& data,
                         const std::vector<size_t>& shape) const;
    void write_npy_header(std::ofstream& out,
                          const std::string& descr,
                          const std::vector<size_t>& shape) const;
    void open_observation_dump();
    void dump_observation_frame(const std::unordered_map<std::string, std::vector<float>>& obs);
    void close_observation_dump();

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<ReferenceLoader> reference_;
    std::vector<float> policy_kp_;
    std::vector<float> policy_kd_;
    std::filesystem::path debug_dump_dir_;
    bool debug_dump_first_frame_ = false;
    bool first_frame_debug_dumped_ = false;
    bool use_motion_root_command_ = false;
    bool use_motion_velocity_command_ = false;
    bool no_global_mode_ = false;
    bool has_initial_yaw_bias_ = false;
    float initial_yaw_bias_ = 0.0f;
    bool observation_dump_enabled_ = false;
    std::filesystem::path observation_dump_file_;
    std::ofstream observation_dump_stream_;
    size_t observation_dump_frame_ = 0;
};

REGISTER_FSM(State_Track)
