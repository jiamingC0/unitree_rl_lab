#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Velocity.h"
#include "State_Track.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<HighState_t> FSMState::highstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::et1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if (!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        std::exit(-1);
    }

    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    FSMState::highstate = std::make_shared<HighState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     ET1 Controller \n";

    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    if (param::config["controller"]) {
        auto controller = param::config["controller"];
        if (controller["mode_pr"]) {
            FSMState::lowcmd->msg_.mode_pr() = controller["mode_pr"].as<int>();
        }
        if (controller["mode_machine"]) {
            FSMState::lowcmd->msg_.mode_machine() = controller["mode_machine"].as<int>();
        }
    }

    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Keyboard: [1] FixStand, [2] Velocity, [3] GeneralTracker, [4] Dance1/Wind Summer, [5] Dance2/PokerFace, [0] Passive.\n";
    std::cout << "Joystick: FixStand [LT+Up], Velocity [RB+X], GeneralTracker [LT(2s)+Up], Dance1 [LT(2s)+Down], Dance2 [LT(2s)+Right].\n";

    while (true)
    {
        sleep(1);
    }

    return 0;
}
