#pragma once

#include "damiao_arm_sdk/types.hpp"

#include <openarm/can/socket/openarm.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace damiao {

// Arm：封装 OpenArm CAN，提供统一初始化、状态读取、安全检查和控制指令接口
class Arm {
public:
    explicit Arm(const ArmConfig& config = ArmConfig::defaultJ4310SixAxis());
    ~Arm() = default;

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;

    // 1. 初始化与连接（只读安全连接，不向电机写寄存器或使能）
    void init();
    bool isConnected() const noexcept { return status_ != ArmStatus::DISCONNECTED; }

    // 2. 安全状态转换：DISARMED <-> ARMED / FAULT
    void enable();     // 使能六台电机，进入 ARMED 状态（若在需要时同步控制模式）
    void disable();    // 失能六台电机，回到 DISARMED 状态
    void clearFault(); // 尝试清除故障并回到 DISARMED 状态

    // 3. 状态刷新与读取（只读安全）
    ArmState refresh(int timeout_us = -1);  // 发送 0xCC 刷新并接收更新所有关节状态
    void recv(int timeout_us = -1);         // 仅接收并更新状态（适用于控制指令发送后）
    ArmState getState() const noexcept { return state_; }
    ArmStatus status() const noexcept { return status_; }
    const ArmConfig& config() const noexcept { return config_; }

    // 4. 安全检查与故障接管
    bool checkHealth();
    bool validateCommand(std::size_t joint_idx, const MITParam& cmd) const;
    bool validateCommand(const std::array<MITParam, NUM_JOINTS>& cmds) const;
    bool validateCommand(std::size_t joint_idx, const PosVelParam& cmd) const;
    bool validateCommand(const std::array<PosVelParam, NUM_JOINTS>& cmds) const;
    bool validateCommand(std::size_t joint_idx, const VelParam& cmd) const;
    bool validateCommand(const std::array<VelParam, NUM_JOINTS>& cmds) const;
    void enterFault(const std::string& reason);

    // 5. 控制指令接口（要求处于 ARMED 状态且通过 checkHealth + validateCommand）
    void sendMIT(std::size_t joint_idx, const MITParam& cmd);
    void sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds);

    void sendPosVel(std::size_t joint_idx, const PosVelParam& cmd);
    void sendPosVelAll(const std::array<PosVelParam, NUM_JOINTS>& cmds);

    void sendVel(std::size_t joint_idx, const VelParam& cmd);
    void sendVelAll(const std::array<VelParam, NUM_JOINTS>& cmds);

    // 底层 OpenArm 访问（供高级诊断使用）
    openarm::can::socket::OpenArm& rawOpenArm();

private:
    void syncStateFromOpenArm();
    int resolveDeviceIndex(std::size_t joint_idx) const;

    ArmConfig config_;
    ArmStatus status_{ArmStatus::DISCONNECTED};
    ArmState state_{};
    std::array<int, NUM_JOINTS> joint_to_dev_idx_{0, 1, 2, 3, 4, 5};
    std::unique_ptr<openarm::can::socket::OpenArm> openarm_;
};

} // namespace damiao
