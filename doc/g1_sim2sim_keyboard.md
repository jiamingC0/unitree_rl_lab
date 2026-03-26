# G1 Sim2Sim Keyboard Workflow

This document records the end-to-end workflow for running `G1-29dof` Sim2Sim with `unitree_mujoco` and `unitree_rl_lab` without a joystick.

## Scope

- Robot: `g1`
- Controller: `deploy/robots/g1_29dof`
- Policy: local built-in `Velocity` policy
- Control method: keyboard fallback in `g1_ctrl`

## 1. Prerequisites

Install the native dependencies:

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

Install `unitree_sdk2`:

```bash
git clone git@github.com:unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build
cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

Build the G1 controller:

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof
mkdir -p build
cd build
cmake ..
make -j4
```

Build `unitree_mujoco` separately by following its installation instructions.

## 2. Available Local G1 Policies

The repository already contains these local `g1_29dof` policies:

- `Velocity`
- `Mimic_Dance_102`
- `Mimic_Gangnam_Style`

For standard Sim2Sim walking, use `Velocity`.

## 3. Configure unitree_mujoco

Edit [simulate/config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml) and make sure it contains:

```yaml
robot: "g1"
domain_id: 0
interface: "lo"
use_joystick: 0
enable_elastic_band: 1
```

Notes:

- `use_joystick: 0` is required when there is no physical gamepad on the machine.
- `interface: "lo"` means simulator and controller communicate over localhost.
- `domain_id` must match on both sides. `g1_ctrl` uses domain `0`.

## 4. Start the Simulation

Terminal 1:

```bash
cd /home/galbot/Deploy/unitree_mujoco/simulate/build
./unitree_mujoco
```

Expected behavior:

- MuJoCo window opens
- G1 scene loads
- no crash after startup

If `use_joystick: 1` is still enabled and no device exists, MuJoCo will print:

```text
Error: Joystick open failed.
```

Set `use_joystick: 0` and restart.

## 5. Start the Controller

Terminal 2:

```bash
cd /home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl -n lo
```

The `-n lo` argument is important. It forces the controller to use the same DDS interface as `unitree_mujoco`.

Expected behavior:

```text
[info] Waiting for connection to robot...
[info] Connected to robot.
```

If it keeps printing:

```text
[warning] Waiting for connection rt/lowstate
```

then the simulator and controller are not connected yet. Re-check:

- `unitree_mujoco` is still running
- `robot: "g1"`
- `domain_id: 0`
- `interface: "lo"`
- `g1_ctrl` was started with `-n lo`

## 6. Keyboard Controls

The controller now supports keyboard fallback for state switching and velocity commands.

These keys must be pressed in the `g1_ctrl` terminal:

### State switching

- `1`: enter `FixStand`
- `2`: enter `Velocity`
- `0`: return to `Passive`
- `3`: enter `Mimic_Dance_102`
- `4`: enter `Mimic_Gangnam_Style`

### Velocity commands

- `w`: forward
- `s`: backward
- `a`: move left
- `d`: move right
- `q`: turn left
- `e`: turn right

These keys must be pressed in the MuJoCo window:

- `8`: make the feet touch the ground
- `9`: disable the elastic band

## 7. Recommended Run Order

1. Start `unitree_mujoco`.
2. Start `g1_ctrl -n lo`.
3. In the `g1_ctrl` terminal, press `1` to enter `FixStand`.
4. Click the MuJoCo window and press `8` to place the feet on the ground.
5. Return to the `g1_ctrl` terminal and press `2` to enter `Velocity`.
6. Click the MuJoCo window and press `9` to disable the elastic band.
7. Return to the `g1_ctrl` terminal and use `w/s/a/d/q/e` to command motion.

## 8. Minimal Operator Checklist

Before pressing `2` for `Velocity`, confirm:

- the robot has already entered `FixStand`
- the feet have touched the ground
- the MuJoCo scene is stable

Before commanding motion, confirm:

- elastic band has been disabled with `9`
- you are typing in the `g1_ctrl` terminal, not the MuJoCo window

## 9. Common Issues

### `Error: Joystick open failed.`

Cause:

- `use_joystick` is enabled but no joystick device exists

Fix:

- set `use_joystick: 0` in [simulate/config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)

### `Waiting for connection rt/lowstate`

Cause:

- DDS mismatch or simulator not publishing to the same interface/domain

Fix:

- keep MuJoCo running
- use `interface: "lo"` in MuJoCo config
- start controller with `./g1_ctrl -n lo`
- keep `domain_id: 0` on both sides

### MuJoCo is open but keyboard commands do nothing

Cause:

- keyboard focus is on the wrong window

Fix:

- `1/2/0/3/4/w/s/a/d/q/e` go to the `g1_ctrl` terminal
- `8/9` go to the MuJoCo window

### The controller says another process is using lowcmd

Cause:

- another controller process is still running

Fix:

- stop the old controller process and restart `g1_ctrl`

## 10. Key File References

- Controller entry: [main.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/main.cpp)
- G1 controller config: [config.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/config/config.yaml)
- Velocity policy params: [deploy.yaml](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml)
- FSM keyboard fallback: [FSMState.h](/home/galbot/Deploy/unitree_rl_lab/deploy/include/FSM/FSMState.h)
- Velocity keyboard mapping: [State_RLBase.cpp](/home/galbot/Deploy/unitree_rl_lab/deploy/robots/g1_29dof/src/State_RLBase.cpp)
- MuJoCo config: [config.yaml](/home/galbot/Deploy/unitree_mujoco/simulate/config.yaml)
