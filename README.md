# Damiao Arm SDK (OpenArm CAN Submodule + USB-CAN Bridge + ros2_control)

C++17 SDK for **Damiao DM-J4310-2EC (24V)** motors and robotic arms, integrating [`enactic/openarm_can`](https://github.com/enactic/openarm_can) as an unmodified Git submodule (`third_party/openarm_can`), providing a transparent **Linux SocketCAN (`vcan0`) <-> Damiao USB-CAN (`/dev/ttyACM0`) Bridge**, and a thin **ROS 2 Jazzy `ros2_control`** hardware adaptation (`ros2/`).

## Phase 1 Architecture: Transparent Driver Adaptation

```
┌────────────────────────────────────────────────────────┐
│            Unmodified OpenArm CAN Submodule            │
│       OpenArm("vcan0", false) -> DM4310 (0x01/0x11)    │
└───────────────────────────┬────────────────────────────┘
                            │ Linux SocketCAN (vcan0)
                            ▼
┌────────────────────────────────────────────────────────┐
│                  SocketCanUsbBridge                    │
│  - Loopback=1, RecvOwnMsgs=0 (zero self-echo loopback) │
│  - Optional Read-Only Filter (--read-only)             │
│  - Partial write retry, disconnect & auto-reconnect    │
│  - Strict CAN ID / DLC / error-frame filtering         │
└───────────────────────────┬────────────────────────────┘
                            │ Damiao USB-CDC (921600)
                            ▼
                 /dev/ttyACM0 -> J4310 Motor
```

## Repository Structure

```
damiao_arm_sdk/
├── include/damiao_arm_sdk/
│   ├── arm.hpp                    # 1..6 active joints (+ optional gripper), state machine & safety
│   ├── damiao_usb_transport.hpp   # Damiao USB-CAN (/dev/ttyACM*) 30B TX / 16B RX transport
│   ├── socketcan_bridge.hpp       # SocketCAN <-> USB-CAN bridge
│   └── types.hpp                  # ArmConfig, JointConfig, ArmState, JointState
├── src/
│   ├── arm.cpp
│   ├── damiao_usb_transport.cpp
│   └── socketcan_bridge.cpp
├── ros2/                          # ROS 2 Jazzy ros2_control adaptation (pure SDK root stays ROS-free)
│   ├── damiao_arm_hardware/       # hardware_interface::SystemInterface plugin
│   └── damiao_arm_bringup/        # URDF/xacro, 200Hz controllers YAML, launch
├── examples/
│   ├── damiao_socketcan_bridge.cpp
│   └── openarm_single_motor_test.cpp
├── tests/
│   ├── test_arm.cpp
│   └── test_sdk.cpp
└── third_party/
    └── openarm_can/               # Unmodified enactic/openarm_can submodule
```

## Build & Run (Pure C++ SDK)

```bash
# 1. Setup virtual CAN interface vcan0
./scripts/setup_vcan.sh vcan0

# 2. Configure & build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

# 3. Run unit & safety tests
ctest --test-dir build --output-on-failure
```

## Single J3507 configuration (pure C++ Arm)

`ArmConfig::joint_count` defaults to six and accepts 1..6. Only the first
`joint_count` entries of the six-element joint/command/state arrays are active;
inactive entries are ignored, and single-joint APIs reject inactive indices.
The optional gripper is a separate device at index `joint_count`.
YAML accepts optional `joint_count` and requires exactly that many `joints`
entries (six when omitted). This does not change the six-axis ROS 2 robot model.

```cpp
auto cfg = damiao::ArmConfig::defaultJ4310SixAxis("vcan0");
cfg.joint_count = 1;
auto& joint = cfg.joints[0];
joint.motor_type = damiao::MotorType::DM3507;
joint.send_can_id = 1;
joint.recv_can_id = 0; // Standard CAN receive ID zero is valid
joint.control_mode = damiao::ControlMode::MIT;
joint.pos_min = -12.5;
joint.pos_max = 12.5;
joint.vel_max = 50.0;
joint.tau_max = 5.0;
damiao::Arm arm(cfg);
arm.init(); // No motion; refresh and inspect feedback before enabling
```

These are protocol limits, not safe experimental motion targets. Use much
smaller motion/gain limits for an unloaded, secured motor and provide an
independent stop. `motor_timeout_reg == 0` leaves motor TIMEOUT unchanged:
a motor already configured with TIMEOUT=0 has no hardware communication-timeout
protection.

The SDK commit does not change OpenArm's motor range tables. Verify that motor
RID21/22/23 match the dependency's position/velocity/torque scaling before motion.
The tested J3507 reports PMAX=12.566, whereas the pinned OpenArm uses 12.5; that
dependency mismatch is not resolved by this SDK change. `JointConfig` limits
only validate commands and do not alter protocol encoding or feedback decoding.

`Arm` exposes three motion APIs: MIT (`sendMIT`/`sendMITAll`), POS_VEL
(`sendPosVel`/`sendPosVelAll`), and VEL (`sendVel`/`sendVelAll`). Each requires
the matching configured and actual motor mode. `enable()` synchronizes and
verifies non-MIT modes; MIT requires the motor to already be in MIT mode.
Lower-level OpenArm additionally supports POS_FORCE, a fourth mode not exposed
as an Arm motion API. A single-motor check cannot validate simultaneous six-axis
behavior, bus loading, mechanical coupling, or the ROS 2 six-axis integration.
This configuration example is not a claim of completed hardware validation.


## Build & Run (ROS 2 Jazzy `ros2_control`)

```bash
cd ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 1. Fake hardware smoke test
ros2 launch damiao_arm_bringup damiao_arm.launch.py use_fake_hardware:=true

# 2. Virtual CAN (vcan0) simulation example (e.g. testing RID 9 write/readback with dummy raw value 2000)
ros2 launch damiao_arm_bringup damiao_arm.launch.py \
  can_interface:=vcan0 has_gripper:=true motor_timeout_reg:=2000

# 3. Real hardware via embedded USB-CAN bridge (replace placeholder with raw uint32 verified via motor tool!)
ros2 launch damiao_arm_bringup damiao_arm.launch.py \
  can_interface:=vcan0 use_usb_bridge:=true serial_port:=/dev/ttyACM0 \
  command_timeout_ms:=100 motor_timeout_reg:=<经上位机确认的RID9原始uint32值>
```

- 默认导出每轴 `position`、`velocity`、`effort`、`kp`、`kd` 命令接口，支持：
  - `joint_trajectory_controller`、`forward_position_controller`、`forward_velocity_controller`、`forward_effort_controller`、`gripper_position_controller`
  - 单轴原子 5 通道 MIT 控制：`joint1_mit_controller` .. `joint6_mit_controller`（`MultiInterfaceForwardCommandController`，`interface_names: [position, velocity, effort, kp, kd]`）；多轴整臂原子阻抗控制可由自定义 `ControllerInterface` 直接 claim 全部 6 轴的 5 个接口。
- `on_activate` 先通过 `refreshUntilAllFresh` 等待全部配置电机（6 轴 + 可选夹爪）返回新鲜非 `stale` 状态并锁存当前关节角（原地保持），再使能电机，不执行自动回零。

## Arm 安全行为

- `disable()` 在 50 ms 内等待全部配置电机（`joint_count` 个关节 + 可选夹爪）的新鲜失能反馈，并检查总线、故障码和 enabled 状态；写入失败、缺失反馈或仍使能均锁定 `FAULT` 并抛异常，不会报告 `DISARMED`。原有 `FAULT` 不由 `disable()` 清除。
- 使能、失能和故障恢复共用 50 ms 状态握手：不仅等待响应计数增长，还等待期望的 enabled 状态，避免在较早的排队反馈到达后提前结束握手。
- 非 MIT 模式在 `enable()` 内写入 `CTRL_MODE` 后，按反馈 CAN ID、电机 ID、RID 和 `0x33` 回读确认实际模式（每台最多 50 ms）；排队状态帧及 `0x55` 写应答不交给状态解析器。缺失或不符的回读拒绝使能并进入 `FAULT`。
- **双模式后台线程**：
  - **Standalone 空闲模式（`command_timeout_ms == 0`，默认）**：`ARMED` 时后台线程每 `min(100 ms, state_timeout_ms / 3)` 查询全部电机状态，上层可不发运动命令；反馈过期或硬件故障进入 `FAULT`。
  - **ROS 周期控制看门狗模式（`command_timeout_ms > 0`，`ros2_control` 默认 `100 ms`）**：正常控制周期内不发冗余 `0xCC` 查询帧；一旦上层控制循环卡死超过 `command_timeout_ms` 未写入控制帧，后台线程独立触发 `enterFault()` 失能全部电机，且不会发送 `0xCC` 误喂电机端看门狗。
- **电机端 `RID::TIMEOUT` (RID 9)**：`motor_timeout_reg` 为写入电机寄存器 RID 9 的原始 `uint32` 值（其物理单位与生效方式由电机固件决定，请先通过达妙上位机或参数工具实测确认，勿凭猜测估算）。
  - `motor_timeout_reg == 0`（默认）：SDK **不启用/不写入**硬件掉线超时，仅在 `on_configure` 读取当前 RID 9 并告警；此时主机进程崩溃（如 `SIGKILL`）或断电没有硬件掉线保护，除非电机此前已写入并固化了超时值。
  - `motor_timeout_reg > 0`：`on_configure`（`DISARMED` 状态）向每台电机 RAM 写入 `RID::TIMEOUT`（`0x55`，不自动写 Flash `0xAA`）并回读 `0x33` 校验，任一电机未确认即拒绝配置。
- SDK 状态访问与收发共用一把锁；`rawOpenArm()` 仅用于非 `ARMED` 状态的单线程诊断。
- 析构停止监控线程，再尽力执行确认失能；断线、信号终止、崩溃及断电不保证物理停机，实际机械臂仍须硬件急停与电机通信超时保护。
