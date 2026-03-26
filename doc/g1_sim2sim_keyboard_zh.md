# G1 Sim2Sim 键盘操作流程

本文档整理了使用 `unitree_mujoco` 和 `unitree_rl_lab` 运行 `G1-29dof` 的 Sim2Sim 全流程，适用于没有手柄、使用键盘操作的场景。

## 适用范围

- 机器人：`g1`
- 控制器：`deploy/robots/g1_29dof`
- 策略：仓库内置 `Velocity` 策略
- 控制方式：`g1_ctrl` 键盘 fallback

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

编译 G1 控制器：

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof
mkdir -p build
cd build
cmake ..
make -j4
```

`unitree_mujoco` 需要单独按它自己的安装文档完成构建。

## 2. 本地可直接使用的 G1 策略

当前仓库内已经带了以下 `g1_29dof` 策略：

- `Velocity`
- `Mimic_Dance_102`
- `Mimic_Gangnam_Style`

如果只是做标准 Sim2Sim 行走测试，优先使用 `Velocity`。

## 3. 配置 unitree_mujoco

编辑 [config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)，确认至少包含以下内容：

```yaml
robot: "g1"
domain_id: 0
interface: "lo"
use_joystick: 0
enable_elastic_band: 1
```

说明：

- `use_joystick: 0` 表示当前不使用物理手柄。
- `interface: "lo"` 表示仿真器和控制器通过本机回环接口通信。
- `domain_id` 两边必须一致。当前 `g1_ctrl` 使用的是 `0`。

## 4. 启动仿真

终端 1：

```bash
cd /home/galbot/Deploy/unitree_mujoco/simulate/build
./unitree_mujoco
```

正常情况应该看到：

- MuJoCo 窗口正常打开
- G1 场景正常加载
- 程序没有在启动后崩溃

如果 `use_joystick: 1` 但系统里没有手柄，会看到：

```text
Error: Joystick open failed.
```

这时把 `use_joystick` 改成 `0` 后重新启动即可。

## 5. 启动控制器

终端 2：

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl -n lo
```

这里的 `-n lo` 很重要，它会强制控制器使用和 `unitree_mujoco` 相同的 DDS 接口。

正常情况应该看到：

```text
[info] Waiting for connection to robot...
[info] Connected to robot.
```

如果持续打印：

```text
[warning] Waiting for connection rt/lowstate
```

说明仿真器和控制器还没有连上。请重点检查：

- `unitree_mujoco` 是否还在运行
- `robot` 是否为 `"g1"`
- `domain_id` 是否为 `0`
- `interface` 是否为 `"lo"`
- `g1_ctrl` 是否使用了 `./g1_ctrl -n lo`

## 6. 键盘控制说明

现在这套 `g1_29dof` 控制器已经支持键盘 fallback，包括状态切换和速度命令。

下面这些键需要在 `g1_ctrl` 所在终端里按：

### 状态切换

- `1`：进入 `FixStand`
- `2`：进入 `Velocity`
- `0`：切回 `Passive`
- `3`：进入 `Mimic_Dance_102`
- `4`：进入 `Mimic_Gangnam_Style`

### 速度控制

- `w`：前进
- `s`：后退
- `a`：向左平移
- `d`：向右平移
- `q`：左转
- `e`：右转

下面这些键需要在 MuJoCo 窗口里按：

- `8`：让双脚接触地面
- `9`：关闭 elastic band

## 7. 推荐操作顺序

1. 启动 `unitree_mujoco`。
2. 启动 `g1_ctrl -n lo`。
3. 在 `g1_ctrl` 终端里按 `1`，进入 `FixStand`。
4. 点击 MuJoCo 窗口，按 `8`，让双脚先接地。
5. 回到 `g1_ctrl` 终端，按 `2`，进入 `Velocity`。
6. 再点击 MuJoCo 窗口，按 `9`，关闭 elastic band。
7. 回到 `g1_ctrl` 终端，使用 `w/s/a/d/q/e` 控制运动。

## 8. 最小操作检查表

在按 `2` 进入 `Velocity` 之前，请确认：

- 机器人已经进入 `FixStand`
- 机器人双脚已经接地
- MuJoCo 场景处于稳定状态

在开始发送速度命令前，请确认：

- 已经用 `9` 关闭 elastic band
- 当前键盘焦点在 `g1_ctrl` 终端，而不是 MuJoCo 窗口

## 9. 常见问题

### `Error: Joystick open failed.`

原因：

- 启用了 `use_joystick`，但系统里没有手柄设备

解决方式：

- 将 [config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml) 中的 `use_joystick` 改为 `0`

### `Waiting for connection rt/lowstate`

原因：

- DDS 接口或 domain 不一致，或者仿真器没有正确发布消息

解决方式：

- 保持 MuJoCo 正在运行
- MuJoCo 配置中使用 `interface: "lo"`
- 控制器使用 `./g1_ctrl -n lo`
- 两边都使用 `domain_id: 0`

### MuJoCo 已经打开，但键盘控制没有反应

原因：

- 键盘焦点在错误的窗口中

解决方式：

- `1/2/0/3/4/w/s/a/d/q/e` 要在 `g1_ctrl` 终端中按
- `8/9` 要在 MuJoCo 窗口中按

### 控制器提示 lowcmd 被别的进程占用

原因：

- 之前的控制器进程还在运行

解决方式：

- 先结束旧的控制器进程，再重新启动 `g1_ctrl`

## 10. 关键文件位置

- 控制器入口：[main.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/main.cpp)
- G1 控制器配置：[config.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/config/config.yaml)
- Velocity 策略参数：[deploy.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml)
- FSM 键盘 fallback：[FSMState.h](/home/galbot/Deploy/unitree_rl_lab/deploy/include/FSM/FSMState.h)
- Velocity 键盘映射：[State_RLBase.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/src/State_RLBase.cpp)
- MuJoCo 配置：[config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)
