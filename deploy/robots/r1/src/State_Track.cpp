#include "State_Track.h"

#include <cmath>
#include <cstring>
#include <cstdlib>
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
    update(0.0f, false, false, Eigen::Vector2f::Zero(), 0.0f);
}

void State_Track::ReferenceLoader::update(float time_s,
                                          bool no_global_mode,
                                          bool has_current_root_xy,
                                          const Eigen::Vector2f& current_root_xy,
                                          float current_root_yaw)
{
    if (frame_count_ == 0) {
        return;
    }

    // Loop the reference so tracking can run continuously in sim.
    const float loop_time = duration_ > 0.0f ? std::fmod(std::max(time_s, 0.0f), duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(loop_time * fps_)), frame_count_ - 1);

    const size_t qpos_offset = frame_index * kQposDim;
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_rel_[i] = qpos_seq_[qpos_offset + 7 + i] - default_joint_pos_[i];
    }

    const size_t pose_offset = frame_index * kKptCount * 16;
    root_height_ = kpt2gv_pose_seq_[pose_offset + 11];  // pelvis(0), row=2, col=3
    for (int i = 0; i < 3; ++i) {
        // root_gravity_[i] = -kpt2gv_pose_seq_[pose_offset + i * 4 + 2];
        root_gravity_[i] = -kpt2gv_pose_seq_[pose_offset + 8 + i];
    }

    const size_t cvel_offset = frame_index * kKptCount * 6;
    for (int i = 0; i < 6; ++i) {
        root_cvel_in_gv_[i] = kpt_cvel_seq_[cvel_offset + i];
    }

    // Keep command semantics unchanged with old cache: absolute yaw/xy from reference root pose.
    const float yaw_ref = quat_to_yaw(
        qpos_seq_[qpos_offset + 3],
        qpos_seq_[qpos_offset + 4],
        qpos_seq_[qpos_offset + 5],
        qpos_seq_[qpos_offset + 6]
    );
    const float yaw_cmd_raw = 0.0f;
    const float yaw_target = yaw_ref + yaw_cmd_raw;
    const float yaw_d = wrap_to_pi(yaw_target - current_root_yaw);
    yaw_cmd_[0] = std::cos(yaw_d);
    yaw_cmd_[1] = std::sin(yaw_d);

    // No-global mode for real robot deployment:
    // keep yaw command from IMU yaw, but disable XY global-position command.
    if (no_global_mode || !has_current_root_xy) {
        xy_cmd_.setZero();
        return;
    }

    const Eigen::Vector2f xy_ref(
        qpos_seq_[qpos_offset + 0],
        qpos_seq_[qpos_offset + 1]
    );
    const Eigen::Vector2f xy_cmd_raw = Eigen::Vector2f::Zero();
    const float c_cmd = std::cos(yaw_cmd_raw);
    const float s_cmd = std::sin(yaw_cmd_raw);
    const Eigen::Matrix2f R_cmd = (Eigen::Matrix2f() << c_cmd, -s_cmd, s_cmd, c_cmd).finished();
    const Eigen::Vector2f xy_target = R_cmd * (xy_cmd_raw + xy_ref);
    Eigen::Vector2f xy_d = xy_target - current_root_xy;

    const float c_curr = std::cos(-current_root_yaw);
    const float s_curr = std::sin(-current_root_yaw);
    const Eigen::Matrix2f R_curr = (Eigen::Matrix2f() << c_curr, -s_curr, s_curr, c_curr).finished();
    xy_d = R_curr * xy_d;
    xy_cmd_ = xy_d;
}

std::filesystem::path State_Track::ReferenceLoader::ensure_cache_file(const std::filesystem::path& motion_file) const
{
    if (motion_file.extension() != ".npz") {
        spdlog::info("Track: motion file '{}' is already in cache format", motion_file.string());
        return motion_file;
    }

    auto cache_file = motion_file;
    cache_file.replace_extension(".r1trk");

    bool regenerate = !std::filesystem::exists(cache_file)
        || std::filesystem::last_write_time(cache_file) < std::filesystem::last_write_time(motion_file);

    if (!regenerate) {
        std::ifstream in(cache_file, std::ios::binary);
        Header header{};
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        const bool header_ok = static_cast<bool>(in) && std::string(header.magic, header.magic + 7) == "R1TRK01";
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

    bool found_qpos = false;
    bool found_pose = false;
    bool found_cvel = false;

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

        if (name == "qpos") {
            if (dims.size() != 2 || dims[1] != kQposDim) {
                throw std::runtime_error("Unexpected qpos shape in cache: " + cache_file.string());
            }
            frame_count_ = dims[0];
            convert_to_float(qpos_seq_);
            found_qpos = true;
        } else if (name == "kpt2gv_pose") {
            if (dims.size() != 4 || dims[1] != kKptCount || dims[2] != 4 || dims[3] != 4) {
                throw std::runtime_error("Unexpected kpt2gv_pose shape in cache: " + cache_file.string());
            }
            convert_to_float(kpt2gv_pose_seq_);
            found_pose = true;
        } else if (name == "kpt_cvel_in_gv") {
            if (dims.size() != 3 || dims[1] != kKptCount || dims[2] != 6) {
                throw std::runtime_error("Unexpected kpt_cvel_in_gv shape in cache: " + cache_file.string());
            }
            convert_to_float(kpt_cvel_seq_);
            found_cvel = true;
        }
    }

    if (!found_qpos || !found_pose || !found_cvel) {
        throw std::runtime_error("Track cache missing required arrays (qpos/kpt2gv_pose/kpt_cvel_in_gv): " + cache_file.string());
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
    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const auto policy_path = policy_dir / "exported" / policy_file;

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
    for (int i = 0; i < lowcmd->msg_.motor_cmd().size(); ++i)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].kd() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
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
    reference_->update(env->episode_length * env->step_dt,
                       no_global_mode_,
                       has_current_root_xy,
                       current_root_xy,
                       current_root_yaw_used);
    env->step();

    auto target_q = env->action_manager->processed_actions();
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);

        auto & motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.q() = target_q[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.tau() = 0.0f;
    }
}

void State_Track::exit()
{
    spdlog::info("Track: exit");
}
