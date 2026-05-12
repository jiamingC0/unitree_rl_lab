#include "State_Track.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <spdlog/spdlog.h>

#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "unitree_articulation.h"

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
}

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(motion_command)
{
    if (!State_Track::reference) {
        throw std::runtime_error("tdf_ET1 Track reference is null while computing motion_command.");
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
        throw std::runtime_error("tdf_ET1 Track reference is null while computing motion_anchor_ori_b.");
    }
    const auto& data = State_Track::reference->motion_anchor_ori_b();
    return std::vector<float>(data.data(), data.data() + data.size());
}

}
}

State_Track::ReferenceLoader::ReferenceLoader(const std::filesystem::path& motion_file, float fps)
    : fps_(fps)
{
    spdlog::info("tdf_ET1 Track: initializing reference loader from '{}' at {} FPS", motion_file.string(), fps_);
    const auto cache_file = ensure_cache_file(motion_file);
    spdlog::info("tdf_ET1 Track: using cache file '{}'", cache_file.string());
    load_cache_file(cache_file);
    duration_ = frame_count_ > 0 ? static_cast<float>(frame_count_ - 1) / fps_ : 0.0f;
    joint_pos_ = Eigen::VectorXf::Zero(kJointDim);
    joint_vel_ = Eigen::VectorXf::Zero(kJointDim);
    spdlog::info("tdf_ET1 Track: reference loaded with {} frames, duration {:.3f}s", frame_count_, duration_);
}

void State_Track::ReferenceLoader::reset()
{
    update(0.0f, Eigen::VectorXf::Zero(kJointDim), Eigen::Quaternionf::Identity());
}

void State_Track::ReferenceLoader::update(float time_s,
                                          const Eigen::VectorXf& robot_joint_pos,
                                          const Eigen::Quaternionf& robot_root_quat)
{
    if (frame_count_ == 0) {
        return;
    }

    const float loop_time = duration_ > 0.0f ? std::fmod(std::max(time_s, 0.0f), duration_) : 0.0f;
    const size_t frame_index = std::min(static_cast<size_t>(std::round(loop_time * fps_)), frame_count_ - 1);

    const size_t joint_offset = frame_index * kJointDim;
    for (int i = 0; i < kJointDim; ++i) {
        joint_pos_[i] = joint_pos_seq_[joint_offset + i];
        joint_vel_[i] = joint_vel_seq_[joint_offset + i];
    }

    const size_t anchor_quat_offset = (frame_index * kBodyCount + kAnchorBodyIndex) * 4;
    Eigen::Quaternionf ref_anchor_q(
        body_quat_w_seq_[anchor_quat_offset + 0],
        body_quat_w_seq_[anchor_quat_offset + 1],
        body_quat_w_seq_[anchor_quat_offset + 2],
        body_quat_w_seq_[anchor_quat_offset + 3]
    );
    ref_anchor_q.normalize();

    const Eigen::Quaternionf robot_anchor_q = anchor_quat_w(robot_root_quat, robot_joint_pos);
    const Eigen::Matrix3f anchor_rot_b = (robot_anchor_q.conjugate() * ref_anchor_q).toRotationMatrix();
    anchor_ori_b_ << anchor_rot_b(0, 0), anchor_rot_b(0, 1),
                     anchor_rot_b(1, 0), anchor_rot_b(1, 1),
                     anchor_rot_b(2, 0), anchor_rot_b(2, 1);
}

Eigen::Quaternionf State_Track::ReferenceLoader::anchor_quat_w(const Eigen::Quaternionf& root_quat,
                                                               const Eigen::VectorXf& joint_pos) const
{
    Eigen::Quaternionf q = root_quat.normalized();
    if (joint_pos.size() >= 14) {
        q = q
            * Eigen::AngleAxisf(joint_pos[12], Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(joint_pos[13], Eigen::Vector3f::UnitZ());
    }
    q.normalize();
    return q;
}

std::filesystem::path State_Track::ReferenceLoader::ensure_cache_file(const std::filesystem::path& motion_file) const
{
    if (motion_file.extension() != ".npz") {
        return motion_file;
    }

    auto cache_file = motion_file;
    cache_file.replace_extension(".tdf_ET1trk");

    bool regenerate = !std::filesystem::exists(cache_file)
        || std::filesystem::last_write_time(cache_file) < std::filesystem::last_write_time(motion_file);

    if (!regenerate) {
        std::ifstream in(cache_file, std::ios::binary);
        Header header{};
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        const bool header_ok = static_cast<bool>(in) && std::string(header.magic, header.magic + 7) == "ET1TRK1";
        if (!header_ok || header.version < kCacheVersion) {
            regenerate = true;
        }
    }

    if (!regenerate) {
        return cache_file;
    }

    const auto repo_root = std::filesystem::weakly_canonical(param::proj_dir / "../../..");
    const auto script_path = repo_root / "scripts" / "et1" / "convert_track_npz.py";

    std::ostringstream cmd;
    cmd << "python3 \"" << script_path.string() << "\""
        << " --input \"" << motion_file.string() << "\""
        << " --output \"" << cache_file.string() << "\"";

    spdlog::info("tdf_ET1 Track: converting NPZ '{}' -> '{}'", motion_file.string(), cache_file.string());
    const int ret = std::system(cmd.str().c_str());
    if (ret != 0 || !std::filesystem::exists(cache_file)) {
        throw std::runtime_error("Failed to convert tdf_ET1 track NPZ to runtime cache: " + motion_file.string());
    }
    return cache_file;
}

void State_Track::ReferenceLoader::load_cache_file(const std::filesystem::path& cache_file)
{
    std::ifstream in(cache_file, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open tdf_ET1 track cache file: " + cache_file.string());
    }

    Header header{};
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!in || std::string(header.magic, header.magic + 7) != "ET1TRK1") {
        throw std::runtime_error("Invalid tdf_ET1 track cache header: " + cache_file.string());
    }
    if (header.version < kCacheVersion) {
        throw std::runtime_error("tdf_ET1 track cache version is too old: " + cache_file.string());
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
        std::string name(name_len, '\0');
        in.read(name.data(), name_len);
        in.read(reinterpret_cast<char*>(&dtype_code), sizeof(dtype_code));
        in.read(reinterpret_cast<char*>(&ndim), sizeof(ndim));

        std::vector<uint32_t> dims(ndim, 0);
        if (ndim > 0) {
            in.read(reinterpret_cast<char*>(dims.data()), sizeof(uint32_t) * ndim);
        }
        uint64_t byte_count = 0;
        in.read(reinterpret_cast<char*>(&byte_count), sizeof(byte_count));
        if (!in) {
            throw std::runtime_error("Failed to read tdf_ET1 cache array header: " + cache_file.string());
        }

        const auto dtype = static_cast<CacheDType>(dtype_code);
        const size_t item_size = dtype_item_size(dtype);
        size_t element_count = 1;
        for (uint32_t dim : dims) {
            element_count *= dim;
        }
        if (element_count * item_size != byte_count) {
            throw std::runtime_error("tdf_ET1 cache array byte size mismatch for '" + name + "'");
        }

        std::vector<char> raw(byte_count);
        if (byte_count > 0) {
            in.read(raw.data(), static_cast<std::streamsize>(byte_count));
        }
        if (!in) {
            throw std::runtime_error("Failed to read tdf_ET1 cache payload for '" + name + "'");
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
                throw std::runtime_error("Unsupported dtype for float conversion in tdf_ET1 cache array '" + name + "'");
            }
        };

        if (name == "joint_pos") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_pos shape in tdf_ET1 cache: " + cache_file.string());
            }
            frame_count_ = dims[0];
            convert_to_float(joint_pos_seq_);
            found_joint_pos = true;
        } else if (name == "joint_vel") {
            if (dims.size() != 2 || dims[1] != kJointDim) {
                throw std::runtime_error("Unexpected joint_vel shape in tdf_ET1 cache: " + cache_file.string());
            }
            convert_to_float(joint_vel_seq_);
            found_joint_vel = true;
        } else if (name == "body_pos_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_pos_w shape in tdf_ET1 cache: " + cache_file.string());
            }
            convert_to_float(body_pos_w_seq_);
            found_body_pos = true;
        } else if (name == "body_quat_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 4) {
                throw std::runtime_error("Unexpected body_quat_w shape in tdf_ET1 cache: " + cache_file.string());
            }
            convert_to_float(body_quat_w_seq_);
            found_body_quat = true;
        } else if (name == "body_lin_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_lin_vel_w shape in tdf_ET1 cache: " + cache_file.string());
            }
            convert_to_float(body_lin_vel_w_seq_);
            found_body_lin_vel = true;
        } else if (name == "body_ang_vel_w") {
            if (dims.size() != 3 || dims[1] != kBodyCount || dims[2] != 3) {
                throw std::runtime_error("Unexpected body_ang_vel_w shape in tdf_ET1 cache: " + cache_file.string());
            }
            convert_to_float(body_ang_vel_w_seq_);
            found_body_ang_vel = true;
        }
    }

    if (!found_joint_pos || !found_joint_vel || !found_body_pos || !found_body_quat
        || !found_body_lin_vel || !found_body_ang_vel) {
        throw std::runtime_error("tdf_ET1 track cache missing required motion arrays: " + cache_file.string());
    }
}

State_Track::State_Track(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const std::string policy_file = cfg["policy_file"] ? cfg["policy_file"].as<std::string>() : "policy.onnx";
    const auto policy_path = policy_dir / "exported" / policy_file;

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if (!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }
    reference_ = std::make_shared<ReferenceLoader>(motion_file, cfg["fps"].as<float>());
    reference = reference_;

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr, HighState_t::SharedPtr>>(
            FSMState::lowstate, FSMState::highstate)
    );
    policy_kp_ = env->cfg["policy_kp"].as<std::vector<float>>();
    policy_kd_ = env->cfg["policy_kd"].as<std::vector<float>>();
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_path.string());
    spdlog::info("tdf_ET1 Track: loaded ONNX '{}'", policy_path.string());
}

void State_Track::enter()
{
    spdlog::info("tdf_ET1 Track: enter");
    for (int i = 0; i < lowcmd->msg_.motor_cmd().size(); ++i) {
        lowcmd->msg_.motor_cmd()[i].kp() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].kd() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
    }
    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        lowcmd->msg_.motor_cmd()[env->robot->data.policy_joint_to_sdk_slot(i)].mode() = 1;
    }

    reference = reference_;
    reference_->reset();
    env->reset();
}

void State_Track::run()
{
    env->robot->update();
    reference_->update(env->episode_length * env->step_dt,
                       env->robot->data.joint_pos,
                       env->robot->data.live_state.root_quat_w);

    env->episode_length += 1;
    env->robot->update();
    const auto obs = env->observation_manager->compute();
    const auto action = env->alg->act(obs);
    env->action_manager->process_action(action);
    const auto target_q = env->action_manager->processed_actions();

    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const int sdk_slot = env->robot->data.policy_joint_to_sdk_slot(i);
        auto& motor = lowcmd->msg_.motor_cmd()[sdk_slot];
        motor.mode() = 1;
        motor.q() = target_q[i];
        motor.dq() = 0.0f;
        motor.kp() = policy_kp_[i];
        motor.kd() = policy_kd_[i];
        motor.tau() = 0.0f;
    }
}

void State_Track::exit()
{
    spdlog::info("tdf_ET1 Track: exit");
}
