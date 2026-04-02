// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <type_traits>

#include "isaaclab/assets/articulation/articulation.h"

namespace unitree
{

template <typename LowStatePtr, typename HighStatePtr = std::nullptr_t>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, HighStatePtr highstate_ = nullptr)
    : lowstate(lowstate_), highstate(highstate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        const int joint_dim = static_cast<int>(data.joint_ids_map.size());
        if (data.live_state.qpos.size() != 7 + joint_dim) {
            data.live_state.qpos = Eigen::VectorXf::Zero(7 + joint_dim);
        }
        if (data.live_state.qvel.size() != 6 + joint_dim) {
            data.live_state.qvel = Eigen::VectorXf::Zero(6 + joint_dim);
        }

        // 1) Read all lowstate-backed values in one lock region into live_state.
        data.live_state.has_lowstate = false;
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            data.live_state.root_gyro_b.x() = lowstate->msg_.imu_state().gyroscope()[0];
            data.live_state.root_gyro_b.y() = lowstate->msg_.imu_state().gyroscope()[1];
            data.live_state.root_gyro_b.z() = lowstate->msg_.imu_state().gyroscope()[2];
            data.live_state.root_quat_w = Eigen::Quaternionf(
                lowstate->msg_.imu_state().quaternion()[0],
                lowstate->msg_.imu_state().quaternion()[1],
                lowstate->msg_.imu_state().quaternion()[2],
                lowstate->msg_.imu_state().quaternion()[3]
            );

            for (int i = 0; i < joint_dim; ++i) {
                const int sdk_slot = data.policy_joint_to_sdk_slot(i);
                data.live_state.qpos[7 + i] = lowstate->msg_.motor_state()[sdk_slot].q();
                data.live_state.qvel[6 + i] = lowstate->msg_.motor_state()[sdk_slot].dq();
            }
            data.live_state.has_lowstate = true;
        }

        data.live_state.qpos.segment<4>(3) << data.live_state.root_quat_w.w(),
            data.live_state.root_quat_w.x(),
            data.live_state.root_quat_w.y(),
            data.live_state.root_quat_w.z();
        data.live_state.qvel.segment<3>(3) = data.live_state.root_gyro_b;

        // 2) Read all highstate-backed values in one lock region into live_state.
        data.live_state.has_highstate = false;
        if constexpr (!std::is_same_v<HighStatePtr, std::nullptr_t>) {
            if (highstate && !highstate->isTimeout()) {
                std::lock_guard<std::mutex> lock(highstate->mutex_);
                data.live_state.root_pos_w.x() = highstate->msg_.position()[0];
                data.live_state.root_pos_w.y() = highstate->msg_.position()[1];
                data.live_state.root_pos_w.z() = highstate->msg_.position()[2];
                data.live_state.root_lin_vel_w.x() = highstate->msg_.velocity()[0];
                data.live_state.root_lin_vel_w.y() = highstate->msg_.velocity()[1];
                data.live_state.root_lin_vel_w.z() = highstate->msg_.velocity()[2];
                data.live_state.has_highstate = true;
            }
        }

        if (data.live_state.has_highstate) {
            data.live_state.qpos.segment<3>(0) = data.live_state.root_pos_w;
            data.live_state.qvel.segment<3>(0) = data.live_state.root_lin_vel_w;
        } else {
            data.live_state.qpos.segment<3>(0).setZero();
            data.live_state.qvel.segment<3>(0).setZero();
        }

        // 3) Derive all articulation state from live_state only.
        data.root_quat_w = data.live_state.root_quat_w;
        data.root_ang_vel_b = data.live_state.root_gyro_b;
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        for (int i = 0; i < joint_dim; ++i) {
            data.joint_pos[i] = data.live_state.qpos[7 + i];
            data.joint_vel[i] = data.live_state.qvel[6 + i];
        }
    }

    LowStatePtr lowstate;
    HighStatePtr highstate;
};

}
