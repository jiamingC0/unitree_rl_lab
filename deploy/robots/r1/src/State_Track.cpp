#include "State_Track.h"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <spdlog/spdlog.h>

#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

std::shared_ptr<State_Track::ReferenceLoader> State_Track::reference = nullptr;

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(ref_joint_pos_rel)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_joint_pos_rel.");
    }
    auto & loader = State_Track::reference;
    const auto & data = loader->joint_pos_rel();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(ref_root_height)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_root_height.");
    }
    return {State_Track::reference->root_height()};
}

REGISTER_OBSERVATION(ref_root_gravity)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_root_gravity.");
    }
    const auto & data = State_Track::reference->root_gravity();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(ref_root_cvel_in_gv)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing ref_root_cvel_in_gv.");
    }
    const auto & data = State_Track::reference->root_cvel_in_gv();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(yaw_command)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing yaw_command.");
    }
    const auto & data = State_Track::reference->yaw_cmd();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(xy_command)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing xy_command.");
    }
    const auto & data = State_Track::reference->xy_cmd();
    return std::vector<float>(data.data(), data.data() + data.size());
}

}
}

State_Track::ReferenceLoader::ReferenceLoader(const std::filesystem::path& motion_file, float fps)
    : fps_(fps)
{
    spdlog::info("Track: initializing reference loader from '{}' at {} FPS", motion_file.string(), fps_);
    const auto cache_file = ensure_cache_file(motion_file);
    spdlog::info("Track: using cache file '{}'", cache_file.string());
    load_cache_file(cache_file);
    duration_ = frame_count_ > 0 ? static_cast<float>(frame_count_ - 1) / fps_ : 0.0f;
    spdlog::info("Track: reference loaded with {} frames, duration {:.3f}s", frame_count_, duration_);
}

void State_Track::ReferenceLoader::reset(const Eigen::VectorXf& default_joint_pos)
{
    default_joint_pos_ = default_joint_pos;
    joint_pos_rel_ = Eigen::VectorXf::Zero(kJointDim);
    update(0.0f);
}

void State_Track::ReferenceLoader::update(float time_s)
{
    if (frame_count_ == 0) {
        return;
    }

    // Loop the reference so tracking can run continuously in sim.
    const float loop_time = duration_ > 0.0f ? std::fmod(std::max(time_s, 0.0f), duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(loop_time * fps_)), frame_count_ - 1);

    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_rel_[i] = joint_pos_[frame_index * kJointDim + i] - default_joint_pos_[i];
    }

    root_height_ = root_height_seq_[frame_index];

    for (int i = 0; i < 3; ++i) {
        root_gravity_[i] = root_gravity_seq_[frame_index * 3 + i];
    }
    for (int i = 0; i < 6; ++i) {
        root_cvel_in_gv_[i] = root_cvel_seq_[frame_index * 6 + i];
    }
    for (int i = 0; i < 2; ++i) {
        yaw_cmd_[i] = yaw_cmd_seq_[frame_index * 2 + i];
        xy_cmd_[i] = xy_cmd_seq_[frame_index * 2 + i];
    }
}

std::filesystem::path State_Track::ReferenceLoader::ensure_cache_file(const std::filesystem::path& motion_file) const
{
    if (motion_file.extension() != ".npz") {
        spdlog::info("Track: motion file '{}' is already in cache format", motion_file.string());
        return motion_file;
    }

    auto cache_file = motion_file;
    cache_file.replace_extension(".r1trk");

    const bool regenerate = !std::filesystem::exists(cache_file)
        || std::filesystem::last_write_time(cache_file) < std::filesystem::last_write_time(motion_file);
    if (!regenerate) {
        spdlog::info("Track: reusing existing cache '{}'", cache_file.string());
        return cache_file;
    }

    const auto repo_root = std::filesystem::weakly_canonical(param::proj_dir / "../../..");
    const auto script_path = repo_root / "scripts" / "r1" / "convert_track_npz.py";

    std::ostringstream cmd;
    cmd << "python3 \"" << script_path.string() << "\""
        << " --input \"" << motion_file.string() << "\""
        << " --output \"" << cache_file.string() << "\"";

    spdlog::info("Track: converting NPZ '{}' -> '{}'", motion_file.string(), cache_file.string());
    const int ret = std::system(cmd.str().c_str());
    if (ret != 0 || !std::filesystem::exists(cache_file)) {
        throw std::runtime_error("Failed to convert track NPZ to runtime cache: " + motion_file.string());
    }
    spdlog::info("Track: cache generated successfully");
    return cache_file;
}

void State_Track::ReferenceLoader::load_cache_file(const std::filesystem::path& cache_file)
{
    spdlog::info("Track: loading cache file '{}'", cache_file.string());
    std::ifstream in(cache_file, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open track cache file: " + cache_file.string());
    }

    Header header{};
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!in || std::string(header.magic, header.magic + 7) != "R1TRK01") {
        throw std::runtime_error("Invalid track cache header: " + cache_file.string());
    }
    if (header.joint_dim != kJointDim) {
        throw std::runtime_error("Unexpected joint dimension in track cache: " + cache_file.string());
    }

    frame_count_ = header.frame_count;
    spdlog::info("Track: cache header ok, frame_count={}, joint_dim={}", frame_count_, header.joint_dim);
    joint_pos_.resize(frame_count_ * kJointDim);
    root_height_seq_.resize(frame_count_);
    root_gravity_seq_.resize(frame_count_ * 3);
    root_cvel_seq_.resize(frame_count_ * 6);
    yaw_cmd_seq_.resize(frame_count_ * 2);
    xy_cmd_seq_.resize(frame_count_ * 2);

    in.read(reinterpret_cast<char*>(joint_pos_.data()), joint_pos_.size() * sizeof(float));
    in.read(reinterpret_cast<char*>(root_height_seq_.data()), root_height_seq_.size() * sizeof(float));
    in.read(reinterpret_cast<char*>(root_gravity_seq_.data()), root_gravity_seq_.size() * sizeof(float));
    in.read(reinterpret_cast<char*>(root_cvel_seq_.data()), root_cvel_seq_.size() * sizeof(float));
    in.read(reinterpret_cast<char*>(yaw_cmd_seq_.data()), yaw_cmd_seq_.size() * sizeof(float));
    in.read(reinterpret_cast<char*>(xy_cmd_seq_.data()), xy_cmd_seq_.size() * sizeof(float));

    if (!in) {
        throw std::runtime_error("Failed to read complete track cache file: " + cache_file.string());
    }
}

State_Track::State_Track(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    spdlog::info("Track: constructing state '{}'", state_string);
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if (!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }
    spdlog::info("Track: resolved motion file '{}'", motion_file.string());
    reference_ = std::make_shared<ReferenceLoader>(motion_file, cfg["fps"].as<float>());
    reference = reference_;
    spdlog::info("Track: reference pointer initialized");

    spdlog::info("Track: loading deploy config '{}'", (policy_dir / "params" / "deploy.yaml").string());
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    policy_kp_ = env->cfg["policy_kp"].as<std::vector<float>>();
    policy_kd_ = env->cfg["policy_kd"].as<std::vector<float>>();
    torque_limit_ = env->cfg["torque_limit"].as<std::vector<float>>();
    spdlog::info("Track: deploy config loaded, constructing ONNX session '{}'", (policy_dir / "exported" / "policy.onnx").string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    spdlog::info("Track: ONNX session created successfully");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_Track::enter()
{
    spdlog::info("Track: enter");
    // Match training-time semantics: the policy outputs target joint angles,
    // while torques are computed locally from current q/dq and then clipped.
    for (int i = 0; i < lowcmd->msg_.motor_cmd().size(); ++i)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].kd() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].dq() = 0;
        lowcmd->msg_.motor_cmd()[i].tau() = 0;
    }

    reference = reference_;
    reference_->reset(env->robot->data.default_joint_pos);
    spdlog::info("Track: reference reset with default joint pose of size {}", env->robot->data.default_joint_pos.size());
    env->reset();
    spdlog::info("Track: environment reset complete");

    policy_thread_running = true;
    policy_thread = std::thread([this]{
        using clock = std::chrono::high_resolution_clock;
        const std::chrono::duration<double> desired_duration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desired_duration);
        auto sleep_till = clock::now() + dt;

        while (policy_thread_running)
        {
            env->robot->update();
            reference_->update(env->episode_length * env->step_dt);
            env->step();

            std::this_thread::sleep_until(sleep_till);
            sleep_till += dt;
        }
    });
}

void State_Track::run()
{
    auto target_q = env->action_manager->processed_actions();
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        const float q = lowstate->msg_.motor_state()[sdk_slot].q();
        const float dq = lowstate->msg_.motor_state()[sdk_slot].dq();
        const float tau = std::clamp(
            policy_kp_[i] * (target_q[i] - q) + policy_kd_[i] * (-dq),
            -torque_limit_[i],
            torque_limit_[i]
        );

        auto & motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.q() = target_q[i];
        motor.tau() = tau;
    }
}

void State_Track::exit()
{
    policy_thread_running = false;
    if (policy_thread.joinable()) {
        policy_thread.join();
    }
}
