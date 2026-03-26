#pragma once

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>

#include "unitree/dds_wrapper/common/Publisher.h"
#include "unitree/dds_wrapper/common/Subscription.h"
#include "unitree/dds_wrapper/common/crc.h"
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

namespace unitree
{
namespace robot
{
namespace r1
{
namespace subscription
{

class LowCmd : public SubscriptionBase<unitree_hg::msg::dds_::LowCmd_>
{
public:
    using SharedPtr = std::shared_ptr<LowCmd>;

    LowCmd(std::string topic = "rt/lowcmd") : SubscriptionBase<MsgType>(topic) {}
};

class LowState : public SubscriptionBase<unitree_hg::msg::dds_::LowState_>
{
public:
    using SharedPtr = std::shared_ptr<LowState>;

    LowState(std::string topic = "rt/lowstate") : SubscriptionBase<MsgType>(topic) {}

    void update()
    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        if (std::all_of(this->msg_.wireless_remote().begin(),
                        this->msg_.wireless_remote().end(),
                        [](uint8_t value) { return value == 0; })) {
            auto now = std::chrono::system_clock::now();
            auto elapsed = now - last_joystick_time_;
            if (elapsed > std::chrono::milliseconds(joystick_timeout_ms_)) {
                isJoystickTimeout_ = true;
            }
        } else {
            last_joystick_time_ = std::chrono::system_clock::now();
            isJoystickTimeout_ = false;
        }

        unitree::common::REMOTE_DATA_RX key{};
        std::memcpy(&key, this->msg_.wireless_remote().data(), sizeof(unitree::common::REMOTE_DATA_RX));
        joystick.extract(key);
    }

    bool isJoystickTimeout() const { return isJoystickTimeout_; }
    unitree::common::UnitreeJoystick joystick;

private:
    uint32_t joystick_timeout_ms_ = 3000;
    bool isJoystickTimeout_ = false;
    std::chrono::time_point<std::chrono::system_clock> last_joystick_time_;
};

} // namespace subscription

namespace publisher
{

class LowCmd : public RealTimePublisher<unitree_hg::msg::dds_::LowCmd_>
{
public:
    using SharedPtr = std::shared_ptr<LowCmd>;

    LowCmd(std::string topic = "rt/lowcmd") : RealTimePublisher<MsgType>(topic) {}

private:
    void pre_communication() override
    {
        msg_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&msg_), (sizeof(MsgType) >> 2) - 1);
    }
};

} // namespace publisher
} // namespace r1
} // namespace robot
} // namespace unitree

using LowCmd_t = unitree::robot::r1::publisher::LowCmd;
using LowState_t = unitree::robot::r1::subscription::LowState;
