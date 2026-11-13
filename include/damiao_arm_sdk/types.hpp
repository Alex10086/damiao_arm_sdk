#pragma once

#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace damiao {

inline constexpr std::size_t NUM_JOINTS = 6;

using MotorType = openarm::damiao_motor::MotorType;
using ControlMode = openarm::damiao_motor::ControlMode;
using MITParam = openarm::damiao_motor::MITParam;
using PosVelParam = openarm::damiao_motor::PosVelParam;
using VelParam = openarm::damiao_motor::VelParam;
using PosForceParam = openarm::damiao_motor::PosForceParam;

enum class ArmStatus {
    DISCONNECTED, // 尚未连接
    DISARMED,     // 已连接，但未允许运动
    ARMED,        // 已使能，允许控制
    FAULT         // 发生异常，禁止正常控制
};

inline const char* toString(ArmStatus status) noexcept {
    switch (status) {
        case ArmStatus::DISCONNECTED: return "DISCONNECTED";
        case ArmStatus::DISARMED:     return "DISARMED";
        case ArmStatus::ARMED:        return "ARMED";
        case ArmStatus::FAULT:        return "FAULT";
        default:                      return "UNKNOWN";
    }
}

// 单个关节的配置与限位
struct JointConfig {
    std::string name{"joint"};
    MotorType motor_type{MotorType::DM4310};
    uint32_t send_can_id{0x01};
    uint32_t recv_can_id{0x11};
    ControlMode control_mode{ControlMode::MIT};

    // 关节机械/指令安全限位
    double pos_min{-12.5}; // rad
    double pos_max{12.5};  // rad
    double vel_max{30.0};  // rad/s
    double tau_max{10.0};  // Nm
    double kp_max{500.0};
    double kd_max{5.0};
};

// 六轴机械臂（+可选夹爪）统一配置
struct ArmConfig {
    std::string can_interface{"vcan0"};
    bool enable_fd{false};
    uint32_t state_timeout_ms{500};   // 反馈新鲜度超时阈值 (ms)
    uint32_t command_timeout_ms{0};   // 周期控制看门狗超时 (ms): 0=空闲保活(0xCC), >0=控制停发超时即FAULT
    uint32_t motor_timeout_reg{0};    // 电机端 RID::TIMEOUT (RID 9) 原始 uint32 值: 0=不写入(仅诊断读取), >0=配置阶段写入RAM并回读校验
    int recv_timeout_us{1000};        // 单次 recv_all 首包等待时间 (us)
    std::array<JointConfig, NUM_JOINTS> joints{};

    // 可选第 7 轴夹爪电机配置（默认关闭，开启后与 6 轴统一管理）
    bool has_gripper{false};
    JointConfig gripper{
        "gripper_joint",
        MotorType::DM4310,
        0x07,
        0x17,
        ControlMode::MIT,
        -12.5,
        12.5,
        30.0,
        10.0,
        500.0,
        5.0
    };

    std::size_t activeMotorCount() const noexcept {
        return has_gripper ? (NUM_JOINTS + 1) : NUM_JOINTS;
    }

    // 默认 6 轴 J4310 配置（关节 0~5 对应 CAN ID 0x01~0x06，反馈 ID 0x11~0x16）
    static ArmConfig defaultJ4310SixAxis(const std::string& can_if = "vcan0");

    // 从 YAML 配置文件加载（如 config/arm.yaml）
    static ArmConfig fromYaml(const std::string& yaml_path);

    // 校验关节 ID 唯一性、限位合法性
    void validate() const;
};

// 单个关节的实时状态反馈
struct JointState {
    uint32_t send_can_id{0};
    uint32_t recv_can_id{0};

    double position{0.0}; // rad
    double velocity{0.0}; // rad/s
    double torque{0.0};   // Nm
    int t_mos{0};         // °C
    int t_rotor{0};       // °C

    bool enabled{false};
    uint8_t error_code{0}; // 0x0: 失能, 0x1: 使能, >=0x8: 故障
    bool has_fault{false};

    // 通信链路状态
    uint64_t commands_sent{0};
    uint64_t responses{0};
    uint32_t malformed_frames{0};
    uint32_t rejected_commands{0};
    bool online{false};
    bool stale{true};
};

// 六轴机械臂（+可选夹爪）统一状态
struct ArmState {
    ArmStatus status{ArmStatus::DISCONNECTED};
    std::array<JointState, NUM_JOINTS> joints{};
    bool has_gripper{false};
    JointState gripper{};
    bool bus_healthy{true};
    bool all_online{false};
    bool any_fault{false};
    std::string fault_reason{};
    std::chrono::steady_clock::time_point timestamp{};
};

} // namespace damiao
