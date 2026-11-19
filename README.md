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

## 六轴混合电机配置与限位标定

`config/arm.yaml` 显式配置 `joint_count: 6`、`has_gripper: false`，总线仍为 `vcan0`，
六轴均使用 MIT 模式，`kp_max: 500`、`kd_max: 5` 不变：

|关节|电机|发送 CAN ID|主机/反馈 CAN ID|速度上限 (rad/s)|力矩上限 (N·m)|
|---|---|---|---|---|---|
|joint1|DM4310|0x01|0x11|30|10|
|joint2|DM4310|0x02|0x12|30|10|
|joint3|DM4310|0x03|0x13|30|10|
|joint4|DM3507|0x04|0x14|50|5|
|joint5|DM3507|0x05|0x15|50|5|
|joint6|DM3507|0x06|0x16|50|5|

**当前六轴位置上下限 ±3.1416 均为 UNCALIBRATED 临时默认值，不是已知机械限位；
本配置不代表完成了实机标定。** 速度、力矩范围也须与实际电机固件及 SDK 协议范围一致，
不能直接作为安全运动目标。

标定采用**失能状态下人工示教**，不自动搜索硬限位。先停止 ROS 控制器及所有其他发送进程，
确保全部六轴在线且失能，并可靠托住/支撑重力负载机械臂，防止失能后坠落、夹伤或连带转动。
人工移动时不要冲撞机械止挡。工具只读取状态和参数，不驱动、不使能、不失能、不设零、
不写 Flash；若电机仍使能，应在工具之外安全失能后再操作。

必须先核对电机 RID21/22/23 与 SDK/OpenArm 的位置、速度、力矩编解码范围：
当前固定版本 OpenArm 的 DM3507 PMAX 为 12.5，而此前实测电机为 12.566，
不匹配时工具可能拒绝标定；应先解决协议范围一致性，不能绕过检查或把结果视为真实角度。
`JointConfig` 的限位不改变协议编码/解码；此处不修改 `third_party/openarm_can`。
标定后不得改变现有物理零点/零偏，否则限位失效，必须重新标定。

在仓库根目录执行（脚本构建 `calibrate_joint_limits`，默认读取仓库根目录的
`config/arm.yaml`，所有参数原样传给工具）：

```bash
./scripts/calibrate_limits.sh --config config/arm.yaml --interface vcan0 \
  --output config/arm.calibrated.yaml

# 仅当已确认是 SDK 支持的达妙 USB-CAN 适配器时，增加串口参数：
./scripts/calibrate_limits.sh --config config/arm.yaml --interface vcan0 \
  --output config/arm.calibrated.yaml --serial /dev/ttyACM0

# 原生 SocketCAN：仅在外部已按真实总线速率正确配置 can0 后使用：
./scripts/calibrate_limits.sh --config config/arm.yaml --interface can0 \
  --output config/arm.calibrated.yaml

# 只检查连接、六轴状态及协议范围，不进行端点示教或保存：
./scripts/calibrate_limits.sh --config config/arm.yaml --interface vcan0 --check-only
```

串口路径仅为示例，应替换为已确认的受支持适配器；`ttyUSB*`/`ttyACM*` 节点存在
并不能证明适配器兼容达妙 CDC 协议，不能仅凭设备节点名称传入 `--serial`。
`vcan0` 本身不连接实物，必须有正确连接的桥接通路；不要同时运行重复桥接/其他发送者。

交互默认每端点采样 10 次、周期 20 ms，并向内预留 3° 裕量。按提示输入 `READY`，
依次对六个关节手动到达安全端点 A、按 Enter 采样，再到安全端点 B、按 Enter 采样；
检查全部结果后输入 `SAVE` 才保存。输入 `q`、EOF 或 Ctrl+C 均中止。
输出必须是不存在的新文件，不覆盖已有文件，不自动应用、不提交或覆盖原配置。
人工审阅输出及机构安全余量后，才能有意地把选定限位复制到原配置。

SDK YAML **不会自动载入 ROS xacro**。ROS 模型已合入导出的真实几何及后三轴 DM3507 配置，
位置范围采用 CAD 导出值 ±3.14，仍未实机标定；控制前必须同步审阅后的 URDF 与 ros2_control 限位。
ROS 默认关闭夹爪，仅连接这六台电机；需要额外夹爪时再显式传入 `has_gripper:=true` 并配置第七轴。


## 真实机械臂模型：先 RViz 与 Fake Hardware

`ros2/damiao_arm_bringup/urdf/arm_geometry.urdf` 来自用户提供的
`总装实验_URDF.zip`：保留七个连杆的网格、碰撞、质量、惯量、关节原点和轴向。
七个 STL 原样保存在 `meshes/`，使用 `package://damiao_arm_bringup/meshes/` 引用，
不做毫米/米缩放。兼容坐标链为 `world → base_link → base → link1 … link6`。
CAD 中第 4/5/6 轴分别为 `+Y/-Z/+X`，不再使用旧占位圆柱模型的轴向。

现有 `joint1`～`joint6`、控制器名和 ros2_control 接口保持不变，CAN/反馈 ID 及
前三轴 DM4310、后三轴 DM3507 映射不变。当前 URDF/硬件参数位置范围为 ±3.14 rad、
速度上限 3 rad/s；后三轴力矩上限从 CAD 的 10 N·m 保守限制为电机的 5 N·m。
这些值不是实机标定结果，也未加入电机方向或零位补偿。可选夹爪默认关闭；
压缩包没有提供夹爪模型，保留的夹爪显示只是旧占位几何。

```bash
cd ~/damiao_arm_sdk/ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select damiao_arm_hardware damiao_arm_bringup \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 安全展示入口：固定使用 Fake Hardware，不打开 CAN/USB，不使能实机。
ros2 launch damiao_arm_bringup display.launch.py

# 无图形桌面时，仅运行 Fake 控制链路：
ros2 launch damiao_arm_bringup display.launch.py start_rviz:=false
```

`display.launch.py` 固定 `mock_components/GenericSystem`、关闭 USB 桥和夹爪，
启动 `joint_state_broadcaster`、`joint_trajectory_controller`；
它在内部固定 Fake，即使传入同名硬件参数也不会切换成真实硬件。RViz 固定坐标系为
`world`，订阅 `robot_description` 与控制链路生成的 TF，不用独立滑块节点伪造关节反馈。
原来的 `damiao_arm.launch.py` 仍保留原有真实硬件启动接口，
可选 `start_rviz:=true`；本阶段不要使用其默认真实硬件模式。

Fake 模式用于确认模型资源、轨迹 action、关节反馈和 TF，不验证动力学、碰撞避障、
线缆、减速比、真实编码器零位或电机极性。接入真实机械臂前应逐轴核对关节编号、
正方向、零位和安全软限位，再审阅重力支撑、急停和电机通信超时设置。

本次 Jazzy 验证：两个 ROS 包构建通过；RViz 的 RobotModel/各连杆 TF 状态为 OK；
GenericSystem + 轨迹控制器完成 14 个目标（含六轴分别运动、归零和展示姿态），反馈与
原始 CAD URDF 的正运动学 TF 一致。未打开 CAN/USB 或驱动实机。测试时未获得 FIFO
实时调度权限，并出现偶发 200 Hz 控制周期超时警告，因此不代表实机实时性验收通过。

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

# 3. Real hardware: verified 100 ms timeout (RID 9 raw value 2000), periodic commands required.
ros2 launch damiao_arm_bringup damiao_arm.launch.py \
  can_interface:=vcan0 use_usb_bridge:=true serial_port:=/dev/ttyACM0 \
  command_timeout_ms:=100 motor_timeout_reg:=2000
```

- 默认导出每轴 `position`、`velocity`、`effort`、`kp`、`kd` 命令接口，支持：
  - `joint_trajectory_controller`、`forward_position_controller`、`forward_velocity_controller`、`forward_effort_controller`、`gripper_position_controller`
  - 单轴原子 5 通道 MIT 控制：`joint1_mit_controller` .. `joint6_mit_controller`（`MultiInterfaceForwardCommandController`，`interface_names: [position, velocity, effort, kp, kd]`）；多轴整臂原子阻抗控制可由自定义 `ControllerInterface` 直接 claim 全部 6 轴的 5 个接口。
- `on_activate` 先通过 `refreshUntilAllFresh` 等待全部配置电机（6 轴 + 可选夹爪）返回新鲜非 `stale` 状态并锁存当前关节角（原地保持），再使能电机，不执行自动回零。

## 已验证的 100 ms 通信保护

本机六轴配置现使用 `command_timeout_ms: 100`、`motor_timeout_reg: 2000`。
本批电机的 RID 9 计数单位为 [50 µs](https://jia-xie.github.io/python-damiao-driver/dev/concept/registers/)，2000 对应 100 ms；不是写入数值 100。
2 号电机 RID 23（TMAX）已从 20 改为 10，与 DM4310 协议编解码范围一致；
此参数不是关节安全力矩上限 `tau_max`，不能混为一谈。

实机逐轴、近零力矩验证：80 ms 断流仍使能；120 ms 断流已报 `0xD` 通信丢失并失能。
20 ms 间隔轮询首次观察到失能约在 108 ms，这不是定时器精确触发时刻。
正常六轴控制帧流量约 197 Hz，7200 个正常控制帧均收到反馈，最大目标轴发包间隔约 5.87 ms。
测试结束六轴均失能、无故障。此测试不代替负载工况、机械支撑和硬件急停验收。

**实测 `0xCC` 读取查询不喂本批电机的硬件看门狗。** 开启 100 ms 硬件保护后，
不能使用旧的“使能后只查询、不写控制帧”的空闲策略来保持使能；当前配置要求上层
周期发送控制帧，停发时允许保护触发，不自动重发旧动作。
工厂函数 `defaultJ4310SixAxis()` 的旧空闲默认值仍为 0；本机请加载
`config/arm.yaml` 或显式设置 `cfg.command_timeout_ms = 100`。

ROS 硬件插件在 `on_configure`（失能状态）写入并回读超时；纯 C++ 的 `Arm::init()`
仍保持只读，不自动写寄存器，调用方须在使能前执行并检查
`arm.configureMotorTimeout(cfg.motor_timeout_reg)` 的返回值。超时配置不会自动重写 TMAX。
CAN `0x55` 仅写 RAM；固化需要额外的 `0xAA`，并会保存当前全部寄存器。
掉电后必须核对实际 TMAX/TIMEOUT；尤其不能把 RAM 回读当作掉电持久化验证。

2026-11-19 经现场确认，在六轴失能状态下各发送一次 Flash 保存，均收到 `0xAA`
保存应答；随后回读六轴 TIMEOUT=2000、2 号 TMAX=10，均失能无故障。
用户完成电机重启后，未启动自动参数配置节点，仅用只读查询再次核对六轴 ID、
TMAX、TIMEOUT、控制模式与保存记录全部一致，重启后参数保留验证通过。

## Arm 安全行为

- `disable()` 在 50 ms 内等待全部配置电机（`joint_count` 个关节 + 可选夹爪）的新鲜失能反馈，并检查总线、故障码和 enabled 状态；写入失败、缺失反馈或仍使能均锁定 `FAULT` 并抛异常，不会报告 `DISARMED`。原有 `FAULT` 不由 `disable()` 清除。
- 使能、失能和故障恢复共用 50 ms 状态握手：不仅等待响应计数增长，还等待期望的 enabled 状态，避免在较早的排队反馈到达后提前结束握手。
- 非 MIT 模式在 `enable()` 内写入 `CTRL_MODE` 后，按反馈 CAN ID、电机 ID、RID 和 `0x33` 回读确认实际模式（每台最多 50 ms）；排队状态帧及 `0x55` 写应答不交给状态解析器。缺失或不符的回读拒绝使能并进入 `FAULT`。
- **双模式后台线程**：
  - **Standalone 空闲模式（`command_timeout_ms == 0`，工厂配置保留）**：`ARMED` 时后台线程每 `min(100 ms, state_timeout_ms / 3)` 查询全部电机状态；这些读取不能维持本批电机的 100 ms 硬件看门狗，不适用于当前机械臂的持续使能。反馈过期或硬件故障进入 `FAULT`。
  - **周期控制看门狗模式（当前 YAML 与 `ros2_control` 默认 `100 ms`）**：正常控制周期内不发冗余 `0xCC`；上层停发超过 `command_timeout_ms` 时后台线程独立触发 `enterFault()` 失能全部电机。它不以读取查询或重发旧动作掩盖控制停发；软件检查存在调度延迟，电机端硬件计时器才是约 100 ms 的掉线保护。
- **电机端 `RID::TIMEOUT` (RID 9)**：`motor_timeout_reg` 为写入电机寄存器 RID 9 的原始 `uint32` 值（其物理单位与生效方式由电机固件决定，请先通过达妙上位机或参数工具实测确认，勿凭猜测估算）。
  - `motor_timeout_reg == 0`：表示不改写 RID 9，不会把已有的非零硬件超时清零；若电机当前为 0，则仍没有硬件掉线保护。ROS 配置阶段会读取并告警。
  - `motor_timeout_reg > 0`（当前 YAML/ROS 默认 `2000`）：ROS 的 `on_configure`（`DISARMED` 状态）向每台电机 RAM 写入 RID 9（`0x55`，不自动写 Flash `0xAA`）并回读校验，任一电机未确认即拒绝配置。纯 C++ 调用方须显式使用 `configureMotorTimeout()`。
- SDK 状态访问与收发共用一把锁；`rawOpenArm()` 仅用于非 `ARMED` 状态的单线程诊断。
- 析构停止监控线程，再尽力执行确认失能；断线、信号终止、崩溃及断电不保证物理停机，实际机械臂仍须硬件急停与电机通信超时保护。
