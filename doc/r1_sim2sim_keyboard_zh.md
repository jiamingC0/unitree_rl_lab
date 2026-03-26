# R1 Sim2Sim 键盘操作流程

本文档整理了使用 `unitree_mujoco` 和 `unitree_rl_lab` 运行 `R1` tracking 策略的 Sim2Sim 全流程，适用于没有手柄、只使用键盘操作的场景。

## 适用范围

- 机器人：`r1`
- 控制器：`deploy/robots/r1`
- 策略：`Track` 策略
- 控制方式：`r1_ctrl` 键盘状态切换
- 参考轨迹格式：`.npz`，会自动转换成 `.r1trk` 运行时缓存

## 1. 前置依赖

安装原生依赖：

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

安装 `unitree_sdk2`：

```bash
git clone git@github.com:unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build
cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

编译 `R1` 控制器：

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1
mkdir -p build
cd build
cmake ..
make -j4
```

`unitree_mujoco` 需要单独按它自己的安装文档完成构建，并且已经接入 `R1Bridge`。

## 2. 准备策略文件

把以下文件放到固定位置：

- ONNX 权重：
  [policy.onnx](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/exported/policy.onnx)
- 参考轨迹：
  [reference.npz](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/params/reference.npz)
- 部署配置：
  [deploy.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/params/deploy.yaml)

说明：

- 第一次运行时，控制器会自动把 `reference.npz` 转成同目录下的 `reference.r1trk`
- `deploy.yaml` 里已经支持：
  - `joint_ids_map`
  - `sdk_joint_ids_map`
  - `default_joint_pos`
  - `policy_kp`
  - `policy_kd`
  - `torque_limit`

## 3. 配置 unitree_mujoco

编辑 [config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)，确认至少包含：

```yaml
robot: "r1"
domain_id: 0
interface: "lo"
use_joystick: 0
enable_elastic_band: 1
```

说明：

- `use_joystick: 0` 表示当前不使用物理手柄
- `interface: "lo"` 表示控制器和仿真器通过本机回环通信
- `domain_id` 两边必须一致，当前控制器使用的是 `0`

## 4. 启动仿真

终端 1：

```bash
cd /home/galbot/Deploy/unitree_mujoco/simulate/build
./unitree_mujoco
```

正常情况应该看到：

- MuJoCo 窗口正常打开
- `R1` 场景正常加载
- 程序没有在启动后崩溃

## 5. 启动控制器

终端 2：

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/build
./r1_ctrl -n lo
```

正常情况应该看到：

```text
[info] Waiting for connection to robot...
[info] Connected to robot.
```

并且随后初始化：

- `State_Passive`
- `State_FixStand`
- `State_Track`

## 6. 键盘操作说明

### 在 `r1_ctrl` 终端里按

这些键是给控制器状态机用的：

- `1`：进入 `FixStand`
- `2`：进入 `Track`
- `0`：回到 `Passive`

注意：

- 这些键必须在运行 `./r1_ctrl -n lo` 的终端里按
- 如果焦点在 MuJoCo 窗口里，`1/2/0` 不会生效

### 在 MuJoCo 窗口里按

这些键是给仿真器窗口本身的：

- `8`：让双脚接地
- `9`：关闭 elastic band

注意：

- `8/9` 必须在 MuJoCo 窗口里按
- 如果焦点还在控制器终端里，`8/9` 不会生效

## 7. 推荐操作顺序

1. 启动 `unitree_mujoco`
2. 启动 `r1_ctrl -n lo`
3. 点击 `r1_ctrl` 终端，按 `1`，进入 `FixStand`
4. 点击 MuJoCo 窗口，按 `8`，让双脚接地
5. 再点击 `r1_ctrl` 终端，按 `2`，进入 `Track`
6. 再点击 MuJoCo 窗口，按 `9`，关闭 elastic band

## 8. Track 控制语义

当前 `R1 Track` 部署代码已经按训练侧语义实现：

```text
target_q = nominal_qpos + nn_action * 0.125
tau = kp * (target_q - qpos) + kd * (-qvel)
tau = clip(tau, -torque_limit, torque_limit)
```

其中：

- `nominal_qpos` 来自 `deploy.yaml` 中的 `default_joint_pos`
- `nn_action * 0.125` 来自 `actions.JointPositionAction.scale`
- `kp/kd/torque_limit` 来自：
  - `policy_kp`
  - `policy_kd`
  - `torque_limit`

## 9. 常见问题

### 控制器启动后在 `Initializing State_Track ...` 处崩溃

优先检查：

- `policy.onnx` 是否在：
  [exported/policy.onnx](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/exported/policy.onnx)
- `reference.npz` 是否在：
  [params/reference.npz](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/params/reference.npz)

### 切到 `Track` 后报 ONNX shape 错误

当前部署已经支持动态 batch 维度输入。  
如果还有问题，请把完整 `OrtRunner` 日志贴出来。

### `reference.r1trk` 没生成

优先检查：

- 系统里是否有 `python3`
- 是否能正常导入 `numpy`
- 参考轨迹文件是否是合法的 `.npz`

运行时使用的转换脚本在：
[convert_track_npz.py](/home/galbot/Deploy/unitree_rl_lab/scripts/r1/convert_track_npz.py)

### 键盘按了没反应

原因通常是焦点在错误窗口：

- `1/2/0` 在 `r1_ctrl` 终端里按
- `8/9` 在 MuJoCo 窗口里按

## 10. 关键文件位置

- 控制器入口：[main.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/main.cpp)
- `R1` 类型包装：[Types.h](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/include/Types.h)
- `Track` 状态定义：[State_Track.h](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/include/State_Track.h)
- `Track` 状态实现：[State_Track.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/src/State_Track.cpp)
- `R1` 控制器配置：[config.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/config.yaml)
- `Track` 部署配置：[deploy.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/r1/config/policy/track/params/deploy.yaml)
- 参考轨迹转换脚本：[convert_track_npz.py](/home/galbot/Deploy/unitree_rl_lab/scripts/r1/convert_track_npz.py)
- MuJoCo 配置：[config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)
