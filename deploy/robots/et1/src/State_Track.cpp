#include "State_Track.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <spdlog/spdlog.h>

#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

std::shared_ptr<State_Track::ReferenceLoader> State_Track::reference = nullptr;

namespace
{
enum class CacheDType : uint32_t
{
    Float32 = 1,
    Float64 = 2,
    Bool = 3,
    Int32 = 4,
    Int64 = 5,
    UInt8 = 6,
};

float quat_to_yaw(float qw, float qx, float qy, float qz)
{
    const float norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-8f) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
    return std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}
}

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(command_root_ori_b)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_root_ori_b.");
    }
    const auto & data = State_Track::reference->command_root_ori_b();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_xy_yaw_vel)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_xy_yaw_vel.");
    }
    const auto & data = State_Track::reference->command_xy_yaw_vel();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_jnt_pos)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_jnt_pos.");
    }
    const auto & data = State_Track::reference->command_joint_pos();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(command_jnt_vel)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing command_jnt_vel.");
    }
    const auto & data = State_Track::reference->command_joint_vel();
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(motion_command)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing motion_command.");
    }
    const auto& pos = State_Track::reference->command_joint_pos();
    const auto& vel = State_Track::reference->command_joint_vel();
    std::vector<float> data;
    data.reserve(pos.size() + vel.size());
    data.insert(data.end(), pos.data(), pos.data() + pos.size());
    data.insert(data.end(), vel.data(), vel.data() + vel.size());
    return data;
}

REGISTER_OBSERVATION(motion_anchor_ori_b)
{
    if (!State_Track::reference) {
        throw std::runtime_error("State_Track::reference is null while computing motion_anchor_ori_b.");
    }
    const auto& data = State_Track::reference->command_root_ori_b();
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
    joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    update(0.0f, false, false, Eigen::Vector2f::Zero(), 0.0f, Eigen::Quaternionf::Identity());
}

void State_Track::ReferenceLoader::update(float time_s,
                                          bool no_global_mode,
                                          bool has_current_root_xy,
                                          const Eigen::Vector2f& current_root_xy,
                                          float current_root_yaw,
                                          const Eigen::Quaternionf& current_root_quat,
                                          bool use_motion_root_command,
                                          bool use_motion_velocity_command)
{
    if (frame_count_ == 0) {
        return;
    }

    // Loop the reference so tracking can run continuously in sim.
    const float loop_time = duration_ > 0.0f ? std::fmod(std::max(time_s, 0.0f), duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(loop_time * fps_)), frame_count_ - 1);

    const size_t joint_offset = frame_index * kJointDim;
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_[i] = joint_pos_seq_[joint_offset + i];
        joint_vel_[i] = joint_vel_seq_[joint_offset + i];
    }

    const size_t root_body_offset = frame_index * kBodyCount;
    const size_t root_quat_offset = root_body_offset * 4;
    Eigen::Quaternionf ref_root_q(
        body_quat_w_seq_[root_quat_offset + 0],
        body_quat_w_seq_[root_quat_offset + 1],
        body_quat_w_seq_[root_quat_offset + 2],
        body_quat_w_seq_[root_quat_offset + 3]
    );
    ref_root_q.normalize();

    if (use_motion_root_command) {
        Eigen::Quaternionf robot_root_q = current_root_quat.normalized();
        const Eigen::Matrix3f root_rot_b = (robot_root_q.conjugate() * ref_root_q).toRotationMatrix();
        root_ori_b_ << root_rot_b(0, 0), root_rot_b(0, 1),
                       root_rot_b(1, 0), root_rot_b(1, 1),
                       root_rot_b(2, 0), root_rot_b(2, 1);
    } else {
        root_ori_b_ << 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f;
    }

    const float yaw_ref = quat_to_yaw(ref_root_q.w(), ref_root_q.x(), ref_root_q.y(), ref_root_q.z());
    const Eigen::Quaternionf ref_yaw_q =
        Eigen::AngleAxisf(yaw_ref, Eigen::Vector3f::UnitZ()) * Eigen::Quaternionf::Identity();
    const size_t root_lin_vel_offset = root_body_offset * 3;
    const size_t root_ang_vel_offset = root_body_offset * 3;
    const Eigen::Vector3f ref_lin_vel_w(
        body_lin_vel_w_seq_[root_lin_vel_offset + 0],
        body_lin_vel_w_seq_[root_lin_vel_offset + 1],
        body_lin_vel_w_seq_[root_lin_vel_offset + 2]
    );
    const Eigen::Vector3f ref_ang_vel_w(
        body_ang_vel_w_seq_[root_ang_vel_offset + 0],
        body_ang_vel_w_seq_[root_ang_vel_offset + 1],
        body_ang_vel_w_seq_[root_ang_vel_offset + 2]
    );
    if (use_motion_velocity_command) {
        const Eigen::Vector3f ref_lin_vel_navi = ref_yaw_q.conjugate() * ref_lin_vel_w;
        const Eigen::Vector3f ref_ang_vel_navi = ref_yaw_q.conjugate() * ref_ang_vel_w;
        xy_yaw_vel_ << ref_lin_vel_navi.x(), ref_lin_vel_navi.y(), ref_ang_vel_navi.z();
    } else {
        xy_yaw_vel_.setZero();
    }
}

std::filesystem::path State_Track::ReferenceLoader::ensure_cache_file(const std::filesystem::path& motion_file) const
{
    if (motion_file.extension() != ".npz") {
        spdlog::info("Track: motion file '{}' is already in cache format", motion_file.string());
        return motion_file;
    }

    auto cache_file = motion_file;
    cache_file.replace_extension(".et1trk");

    bool regenerate = !std::filesystem::exists(cache_file)
        || std::filesystem::last_write_time(cache_file) < std::filesystem::last_write_time(motion_file);

    if (!regenerate) {
        std::ifstream in(cache_file, std::ios::binary);
        Header header{};
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        const bool header_ok = static_cast<bool>(in) && std::string(header.magic, header.magic + 7) == "ET1TRK1";
        if (!header_ok || header.version < kCacheVersion) {
            regenerate = true;
            spdlog::info("Track: cache '{}' is stale/incompatible, regenerating", cache_file.string());
        }
    }

    if (!regenerate) {
        spdlog::info("Track: reusing existing cache '{}'", cache_file.string());
        return cache_file;
    }

    const auto repo_root = std::filesystem::weakly_canonical(param::proj_dir / "../../..");
    const auto script_path = repo_root / "scripts" / "et1" / "convert_track_npz.py";

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
    if (!in || std::string(header.magic, header.magic + 7) != "ET1TRK1") {
        throw std::runtime_error("Invalid track cache header: " + cache_file.string());
    }
    if (header.version < kCacheVersion) {
        throw std::runtime_error("Track cache version is too old; regenerate cache for: " + cache_file.string());
    }

    auto dtype_item_size = [](CacheDType dtype) -> size_t {
        switch (dtype) {
            case CacheDType::Float32: return sizeof(float);
            case CacheDType::Float64: return sizeof(double);
            case CacheDType::Bool: return sizeof(bool);
            case CacheDType::Int32: return sizeof(int32_t);
            case CacheDType::Int64: return sizeof(int64_t);
            case CacheDType::UInt8: return sizeof(uint8_t);
        }
        throw std::runtime_error("Unknown cache dtype");
    };

    bool found_joint_pos = false;
    bool found_joint_vel = false;
    bool found_body_pos = false;
    bool found_body_quat = false;
    bool found_body_lin_vel = false;
    bool found_body_ang_vel = false;

    for (uint32_t array_idx = 0; array_idx < header.array_count; ++array_idx) {
        uint32_t name_len = 0;
        uint32_t dtype_code = 0;
        uint32_t ndim = 0;
        in.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
        if (!in) {
            throw std::runtime_error("Failed to read cache array name length: " + cache_file.string());
        }

        std::string name(name_len, '\0');
        in.read(name.data(), name_len);
        in.read(reinterpret_cast<char*>(&dtype_code), sizeof(dtype_code));
        in.read(reinterpret_cast<char*>(&ndim), sizeof(ndim));
        if (!in) {
            throw std::runtime_error("Failed to read cache array header: " + cache_file.string());
        }

        std::vector<uint32_t> dims(ndim, 0);
        if (ndim > 0) {
            in.read(reinterpret_cast<char*>(dims.data()), sizeof(uint32_t) * ndim);
            if (!in) {
                throw std::runtime_error("Failed to read cache array dims: " + cache_file.string());
            }
        }

        uint64_t byte_count = 0;
        in.read(reinterpret_cast<char*>(&byte_count), sizeof(byte_count));
        if (!in) {
            throw std::runtime_error("Failed to read cache array byte count: " + cache_file.string());
        }

        const auto dtype = static_cast<CacheDType>(dtype_code);
        const size_t item_size = dtype_item_size(dtype);
        size_t element_count = 1;
        for (uint32_t dim : dims) {
            element_count *= dim;
        }
        if (element_count * item_size != byte_count) {
            throw std::runtime_error("Cache array byte size mismatch for '" + name + "': " + cache_file.string());
        }

        std::vector<char> raw(byte_count);
        if (byte_count > 0) {
            in.read(raw.data(), static_cast<std::streamsize>(byte_count));
            if (!in) {
                throw std::runtime_error("Failed to read cache array payload for '" + name + "': " + cache_file.string());
            }
        }

        auto convert_to_float = [&](std::vector<float>& out) {
            out.resize(element_count);
            if (dtype == CacheDType::Float32) {
                std::memcpy(out.data(), raw.data(), byte_count);
            } else if (dtype == CacheDType::Float64) {
                const auto* src = reinterpret_cast<const double*>(raw.data());
                for (size_t i = 0; i < element_count; ++i) {
                    out[i] = static_cast<float>(src[i]);
                }
            } else {
                throw std::runtime_error("Unsupported dtype for float conversion in array '" + name + "'");
            }
        };

        if (name == "joint_pos") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_pos shape in cache: " + cache_file.string());
            }
            frame_count_ = dims[0];
            convert_to_float(joint_pos_seq_);
            found_joint_pos = true;
        } else if (name == "joint_vel") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_vel shape in cache: " + cache_file.string());
            }
            convert_to_float(joint_vel_seq_);
            found_joint_vel = true;
        } else if (name == "body_pos_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_pos_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_pos_w_seq_);
            found_body_pos = true;
        } else if (name == "body_quat_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 4) {
                throw std::runtime_error("Unexpected body_quat_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_quat_w_seq_);
            found_body_quat = true;
        } else if (name == "body_lin_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_lin_vel_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_lin_vel_w_seq_);
            found_body_lin_vel = true;
        } else if (name == "body_ang_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_ang_vel_w shape in cache: " + cache_file.string());
            }
            convert_to_float(body_ang_vel_w_seq_);
            found_body_ang_vel = true;
        }
    }

    if (!found_joint_pos || !found_joint_vel || !found_body_pos || !found_body_quat
        || !found_body_lin_vel || !found_body_ang_vel) {
        throw std::runtime_error("ET1 track cache missing required motion arrays: " + cache_file.string());
    }
}

float State_Track::ReferenceLoader::wrap_to_pi(float angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

State_Track::State_Track(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    spdlog::info("Track: constructing state '{}'", state_string);
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    no_global_mode_ = cfg["no_global_mode"].as<bool>(false);
    spdlog::info("Track: no_global_mode = {}", no_global_mode_ ? "true" : "false");
    use_motion_root_command_ = cfg["use_motion_root_command"].as<bool>(false);
    use_motion_velocity_command_ = cfg["use_motion_velocity_command"].as<bool>(false);
    spdlog::info("Track: use_motion_root_command = {}, use_motion_velocity_command = {}",
                 use_motion_root_command_ ? "true" : "false",
                 use_motion_velocity_command_ ? "true" : "false");
    debug_dump_first_frame_ = cfg["debug_dump_first_frame"].as<bool>(false);
    const std::string default_debug_dir = "debug/et1_track_first_frame";
    debug_dump_dir_ = cfg["debug_dump_dir"]
        ? std::filesystem::path(cfg["debug_dump_dir"].as<std::string>())
        : std::filesystem::path(default_debug_dir);
    if (!debug_dump_dir_.is_absolute()) {
        debug_dump_dir_ = param::proj_dir / debug_dump_dir_;
    }
    if (debug_dump_first_frame_) {
        spdlog::info("Track: first-frame debug dump enabled at '{}'", debug_dump_dir_.string());
    }
    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const std::string deploy_file = cfg["deploy_file"] ? cfg["deploy_file"].as<std::string>() : "deploy.yaml";
    const auto policy_path = policy_dir / "exported" / policy_file;

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if (!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }
    spdlog::info("Track: resolved motion file '{}'", motion_file.string());
    reference_ = std::make_shared<ReferenceLoader>(motion_file, cfg["fps"].as<float>());
    reference = reference_;
    spdlog::info("Track: reference pointer initialized");

    spdlog::info("Track: loading deploy config '{}'", (policy_dir / "params" / deploy_file).string());
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / deploy_file),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
            FSMState::lowstate, FSMState::highstate)
    );
    policy_kp_ = env->cfg["policy_kp"].as<std::vector<float>>();
    policy_kd_ = env->cfg["policy_kd"].as<std::vector<float>>();
    spdlog::info("Track: deploy config loaded, constructing ONNX session '{}'", policy_path.string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_path.string());
    spdlog::info("Track: ONNX session created successfully");

    // this->registered_checks.emplace_back(
    //     std::make_pair(
    //         [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
    //         FSMStringMap.right.at("Passive")
    //     )
    // );
    // Temporarily disable bad_orientation-triggered state switch in Track.
}

void State_Track::enter()
{
    spdlog::info("Track: enter");
    has_initial_yaw_bias_ = false;
    initial_yaw_bias_ = 0.0f;
    first_frame_debug_dumped_ = false;
    for (int i = 0; i < lowcmd->msg_.motor_cmd().size(); ++i)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].kd() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
    }
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        lowcmd->msg_.motor_cmd()[env->robot->data.policy_joint_to_sdk_slot(i)].mode() = 1;
    }

    reference = reference_;
    reference_->reset(env->robot->data.default_joint_pos);
    spdlog::info("Track: reference reset with default joint pose of size {}", env->robot->data.default_joint_pos.size());
    env->reset();
    spdlog::info("Track: environment reset complete");

    if (no_global_mode_) {
        env->robot->update();
        const auto& live_state = env->robot->data.live_state;
        initial_yaw_bias_ = quat_to_yaw(
            live_state.root_quat_w.w(),
            live_state.root_quat_w.x(),
            live_state.root_quat_w.y(),
            live_state.root_quat_w.z()
        );
        has_initial_yaw_bias_ = true;
        spdlog::info("Track: no_global_mode yaw-zero bias initialized: {:.6f} rad", initial_yaw_bias_);
    }
}

void State_Track::run()
{
    // One Track::run() call is one full 50Hz high-level cycle.
    env->robot->update();
    const auto& live_state = env->robot->data.live_state;
    const float current_root_yaw = quat_to_yaw(
        live_state.root_quat_w.w(),
        live_state.root_quat_w.x(),
        live_state.root_quat_w.y(),
        live_state.root_quat_w.z()
    );
    float current_root_yaw_used = current_root_yaw;
    if (no_global_mode_) {
        if (!has_initial_yaw_bias_) {
            initial_yaw_bias_ = current_root_yaw;
            has_initial_yaw_bias_ = true;
        }
        current_root_yaw_used = std::atan2(
            std::sin(current_root_yaw - initial_yaw_bias_),
            std::cos(current_root_yaw - initial_yaw_bias_)
        );
    }
    const bool has_current_root_xy = (!no_global_mode_) && live_state.has_highstate;
    Eigen::Vector2f current_root_xy = Eigen::Vector2f::Zero();
    if (has_current_root_xy) {
        current_root_xy = live_state.root_pos_w.head<2>();
    }
    reference_->update((env->episode_length + 1) * env->step_dt,
                       no_global_mode_,
                       has_current_root_xy,
                       current_root_xy,
                       current_root_yaw_used,
                       live_state.root_quat_w,
                       use_motion_root_command_,
                       use_motion_velocity_command_);
    env->episode_length += 1;
    env->robot->update();
    const auto obs = env->observation_manager->compute();
    const auto action = env->alg->act(obs);
    env->action_manager->process_action(action);
    auto target_q = env->action_manager->processed_actions();
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);

        auto & motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = target_q[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.tau() = 0.0f;
    }

    if (debug_dump_first_frame_ && !first_frame_debug_dumped_) {
        dump_first_frame_debug(obs, action, target_q);
        first_frame_debug_dumped_ = true;
    }
}

void State_Track::exit()
{
    spdlog::info("Track: exit");
}

void State_Track::dump_first_frame_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                                         const std::vector<float>& action,
                                         const std::vector<float>& target_q)
{
    try {
        std::filesystem::create_directories(debug_dump_dir_);

        auto write_obs = [&](const std::string& name, const std::vector<size_t>& preferred_shape) {
            auto it = obs.find(name);
            if (it == obs.end()) {
                spdlog::warn("Track debug: observation '{}' is absent, skip dump", name);
                return;
            }
            write_npy_float(debug_dump_dir_ / (name + ".npy"), it->second, preferred_shape);
        };

        write_obs("obs_current", {1, obs.count("obs_current") ? obs.at("obs_current").size() : 0});
        if (obs.count("obs_history") && obs.at("obs_history").size() % 25 == 0) {
            write_obs("obs_history", {1, 25, obs.at("obs_history").size() / 25});
        } else {
            write_obs("obs_history", {1, obs.count("obs_history") ? obs.at("obs_history").size() : 0});
        }

        write_npy_float(debug_dump_dir_ / "action.npy", action, {1, action.size()});
        write_npy_float(debug_dump_dir_ / "target_joint_pos.npy", target_q, {1, target_q.size()});

        std::vector<float> current_joint_pos(env->robot->data.joint_pos.data(),
                                             env->robot->data.joint_pos.data() + env->robot->data.joint_pos.size());
        std::vector<float> current_joint_vel(env->robot->data.joint_vel.data(),
                                             env->robot->data.joint_vel.data() + env->robot->data.joint_vel.size());
        write_npy_float(debug_dump_dir_ / "deploy_joint_pos.npy", current_joint_pos, {1, current_joint_pos.size()});
        write_npy_float(debug_dump_dir_ / "deploy_joint_vel.npy", current_joint_vel, {1, current_joint_vel.size()});

        const auto& live_state = env->robot->data.live_state;
        const std::vector<float> root_quat = {
            live_state.root_quat_w.w(),
            live_state.root_quat_w.x(),
            live_state.root_quat_w.y(),
            live_state.root_quat_w.z(),
        };
        write_npy_float(debug_dump_dir_ / "deploy_root_quat_w.npy", root_quat, {1, 4});

        const size_t motor_count = lowcmd->msg_.motor_cmd().size();
        std::vector<float> lowcmd_q(motor_count, 0.0f);
        std::vector<float> lowcmd_dq(motor_count, 0.0f);
        std::vector<float> lowcmd_kp(motor_count, 0.0f);
        std::vector<float> lowcmd_kd(motor_count, 0.0f);
        std::vector<float> lowcmd_tau(motor_count, 0.0f);
        std::vector<int64_t> lowcmd_mode(motor_count, 0);
        for (size_t i = 0; i < motor_count; ++i) {
            const auto& motor = lowcmd->msg_.motor_cmd()[i];
            lowcmd_q[i] = motor.q();
            lowcmd_dq[i] = motor.dq();
            lowcmd_kp[i] = motor.kp();
            lowcmd_kd[i] = motor.kd();
            lowcmd_tau[i] = motor.tau();
            lowcmd_mode[i] = motor.mode();
        }
        write_npy_float(debug_dump_dir_ / "lowcmd_q_idl.npy", lowcmd_q, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_dq_idl.npy", lowcmd_dq, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_kp_idl.npy", lowcmd_kp, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_kd_idl.npy", lowcmd_kd, {1, motor_count});
        write_npy_float(debug_dump_dir_ / "lowcmd_tau_idl.npy", lowcmd_tau, {1, motor_count});
        write_npy_int64(debug_dump_dir_ / "lowcmd_mode_idl.npy", lowcmd_mode, {1, motor_count});

        write_npy_int64(debug_dump_dir_ / "motion_time_steps.npy", {env->episode_length}, {1});
        write_npy_int64(debug_dump_dir_ / "motion_clip_id.npy", {0}, {1});

        spdlog::info("Track debug: dumped first frame to '{}'", debug_dump_dir_.string());
    } catch (const std::exception& e) {
        spdlog::error("Track debug: failed to dump first frame: {}", e.what());
    }
}

void State_Track::write_npy_float(const std::filesystem::path& path,
                                  const std::vector<float>& data,
                                  const std::vector<size_t>& shape) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open npy file for write: " + path.string());
    }
    write_npy_header(out, "<f4", shape);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(float)));
}

void State_Track::write_npy_int64(const std::filesystem::path& path,
                                  const std::vector<int64_t>& data,
                                  const std::vector<size_t>& shape) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open npy file for write: " + path.string());
    }
    write_npy_header(out, "<i8", shape);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(int64_t)));
}

void State_Track::write_npy_header(std::ofstream& out,
                                   const std::string& descr,
                                   const std::vector<size_t>& shape) const
{
    std::ostringstream shape_ss;
    shape_ss << "(";
    for (size_t i = 0; i < shape.size(); ++i) {
        if (i > 0) {
            shape_ss << ", ";
        }
        shape_ss << shape[i];
    }
    if (shape.size() == 1) {
        shape_ss << ",";
    }
    shape_ss << ")";

    std::string header = "{'descr': '" + descr + "', 'fortran_order': False, 'shape': "
        + shape_ss.str() + ", }";
    const size_t preamble_size = 10;
    const size_t padding = 16 - ((preamble_size + header.size() + 1) % 16);
    header.append(padding, ' ');
    header.push_back('\n');

    if (header.size() > std::numeric_limits<uint16_t>::max()) {
        throw std::runtime_error("NPY header is too large.");
    }

    const char magic[] = "\x93NUMPY";
    out.write(magic, 6);
    const char version[2] = {1, 0};
    out.write(version, 2);
    const uint16_t header_len = static_cast<uint16_t>(header.size());
    out.write(reinterpret_cast<const char*>(&header_len), sizeof(header_len));
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
}
