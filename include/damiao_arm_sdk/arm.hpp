#pragma once

#include "damiao_arm_sdk/types.hpp"

#include <openarm/can/socket/openarm.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace damiao {

// Arm：封装 OpenArm CAN，提供统一初始化、状态读取、安全检查和控制指令接口
class Arm {
public:
    explicit Arm(const ArmConfig& config = ArmConfig::defaultJ4310SixAxis());
    ~Arm() noexcept;

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;

    // 1. 初始化与连接（只读安全连接，禁止在 ARMED 或 FAULT 状态直接重初始化）
    void init();
    bool isConnected() const { std::lock_guard<std::recursive_mutex> lock(mutex_); return status_ != ArmStatus::DISCONNECTED; }

    // 2. 安全状态转换：DISARMED <-> ARMED / FAULT
    void enable();     // 检查全部电机在线、新鲜、无故障并轮询确认全部使能成功后进入 ARMED
    void disable();    // 确认全部电机新鲜失能反馈后回到 DISARMED；失败进入 FAULT 并抛异常
    void clearFault(); // 严格恢复：仅在总线健康且无电机故障/超时时退出 FAULT 回到 DISARMED

    // 3. 状态刷新与读取（只读安全）
    ArmState refresh(int timeout_us = -1);            // 发送 0xCC 刷新并接收更新所有关节状态
    bool refreshUntilAllFresh(int timeout_ms = 50);   // 发送 0xCC 并在窗口内轮询直到全部电机返回新鲜反馈
    void recv(int timeout_us = -1);                   // 接收并更新状态
    ArmState getState() const { std::lock_guard<std::recursive_mutex> lock(mutex_); return state_; }
    ArmStatus status() const { std::lock_guard<std::recursive_mutex> lock(mutex_); return status_; }
    const ArmConfig& config() const noexcept { return config_; }

    // 3b. 电机端硬件超时保护 (RID::TIMEOUT / RID 9) 查询与 RAM 写入校验（仅 DISARMED 状态可用）
    std::vector<std::optional<uint32_t>> queryMotorTimeouts(int timeout_ms = 100);
    bool configureMotorTimeout(uint32_t raw_timeout_reg, int timeout_ms = 100);

    // 4. 安全检查与故障接管
    bool checkHealth();
    bool validateCommand(std::size_t joint_idx, const MITParam& cmd) const;
    bool validateCommand(const std::array<MITParam, NUM_JOINTS>& cmds) const;
    bool validateGripperCommand(const MITParam& cmd) const;
    bool validateCommand(std::size_t joint_idx, const PosVelParam& cmd) const;
    bool validateCommand(const std::array<PosVelParam, NUM_JOINTS>& cmds) const;
    bool validateCommand(std::size_t joint_idx, const VelParam& cmd) const;
    bool validateCommand(const std::array<VelParam, NUM_JOINTS>& cmds) const;
    void enterFault(const std::string& reason);

    // 5. 控制指令接口（要求处于 ARMED 状态且通过 checkHealth + validateCommand + 发送结果校验）
    void sendMIT(std::size_t joint_idx, const MITParam& cmd);
    void sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds);
    void sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds, const MITParam& gripper_cmd);
    void sendGripperMIT(const MITParam& cmd);

    void sendPosVel(std::size_t joint_idx, const PosVelParam& cmd);
    void sendPosVelAll(const std::array<PosVelParam, NUM_JOINTS>& cmds);

    void sendVel(std::size_t joint_idx, const VelParam& cmd);
    void sendVelAll(const std::array<VelParam, NUM_JOINTS>& cmds);

    // 仅限 DISARMED 下单线程诊断；返回引用不受 SDK 锁保护，禁止与其他 SDK 调用并发使用。
    openarm::can::socket::OpenArm& rawOpenArm();

private:
    void syncStateFromOpenArm();
    void monitorLoop();
    bool pollUntilAllRespond(const std::array<uint64_t, NUM_JOINTS>& baseline_responses,
                             uint64_t gripper_baseline,
                             int total_timeout_ms,
                             bool check_enabled,
                             bool expected_enabled);
    int resolveDeviceIndex(std::size_t joint_idx) const;
    void verifySendSucceeded(const char* api_name,
                             uint64_t prev_writes_ok,
                             std::size_t expected_writes,
                             const std::array<uint32_t, NUM_JOINTS>& prev_rejected,
                             uint32_t prev_gripper_rejected);

    ArmConfig config_;
    ArmStatus status_{ArmStatus::DISCONNECTED};
    ArmState state_{};
    std::array<int, NUM_JOINTS> joint_to_dev_idx_{0, 1, 2, 3, 4, 5};
    int gripper_dev_idx_{-1};
    std::chrono::steady_clock::time_point last_command_time_{};
    mutable std::recursive_mutex mutex_;
    std::condition_variable_any monitor_cv_;
    bool stopping_{false};
    std::thread monitor_;
    std::unique_ptr<openarm::can::socket::OpenArm> openarm_;
};

} // namespace damiao
