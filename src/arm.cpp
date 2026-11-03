#include "damiao_arm_sdk/arm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace damiao {

namespace {

std::string trim(const std::string& s) {
    auto first = s.find_first_not_of(" \t\r\n\"'");
    if (first == std::string::npos) return "";
    auto last = s.find_last_not_of(" \t\r\n\"'");
    return s.substr(first, last - first + 1);
}

MotorType parseMotorType(const std::string& s) {
    if (s == "DM3507")     return MotorType::DM3507;
    if (s == "DM4310" || s == "DM_J4310_2EC" || s == "J4310") return MotorType::DM4310;
    if (s == "DM4310_48V") return MotorType::DM4310_48V;
    if (s == "DM4340" || s == "DM_J4340_2EC" || s == "J4340") return MotorType::DM4340;
    if (s == "DM4340_48V") return MotorType::DM4340_48V;
    if (s == "DM6006")     return MotorType::DM6006;
    if (s == "DM8006")     return MotorType::DM8006;
    if (s == "DM8009" || s == "DM_J8009" || s == "J8009")     return MotorType::DM8009;
    if (s == "DM10010L")   return MotorType::DM10010L;
    if (s == "DM10010")    return MotorType::DM10010;
    if (s == "DMH3510")    return MotorType::DMH3510;
    if (s == "DMH6215")    return MotorType::DMH6215;
    if (s == "DMG6220")    return MotorType::DMG6220;
    throw std::invalid_argument("ArmConfig: unknown motor_type '" + s + "'");
}

ControlMode parseControlMode(const std::string& s) {
    if (s == "MIT")       return ControlMode::MIT;
    if (s == "POS_VEL")   return ControlMode::POS_VEL;
    if (s == "VEL")       return ControlMode::VEL;
    if (s == "POS_FORCE") return ControlMode::POS_FORCE;
    throw std::invalid_argument("ArmConfig: unknown control_mode '" + s + "'");
}

void applyJointField(JointConfig& j, const std::string& key, const std::string& val) {
    if (key == "name")              j.name = val;
    else if (key == "motor_type")   j.motor_type = parseMotorType(val);
    else if (key == "send_can_id")  j.send_can_id = static_cast<uint32_t>(std::stoul(val, nullptr, 0));
    else if (key == "recv_can_id")  j.recv_can_id = static_cast<uint32_t>(std::stoul(val, nullptr, 0));
    else if (key == "control_mode") j.control_mode = parseControlMode(val);
    else if (key == "pos_min")      j.pos_min = std::stod(val);
    else if (key == "pos_max")      j.pos_max = std::stod(val);
    else if (key == "vel_max")      j.vel_max = std::stod(val);
    else if (key == "tau_max")      j.tau_max = std::stod(val);
    else if (key == "kp_max")       j.kp_max = std::stod(val);
    else if (key == "kd_max")       j.kd_max = std::stod(val);
}

} // namespace

// ============================================================================
// ArmConfig 实现
// ============================================================================

ArmConfig ArmConfig::defaultJ4310SixAxis(const std::string& can_if) {
    ArmConfig cfg;
    cfg.can_interface = can_if;
    cfg.enable_fd = false;
    cfg.state_timeout_ms = 500;
    cfg.recv_timeout_us = 1000;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        JointConfig j;
        j.name = "joint" + std::to_string(i + 1);
        j.motor_type = MotorType::DM4310;
        j.send_can_id = static_cast<uint32_t>(i + 1);
        j.recv_can_id = static_cast<uint32_t>(i + 1 + 0x10);
        j.control_mode = ControlMode::MIT;
        j.pos_min = -12.5;
        j.pos_max = 12.5;
        j.vel_max = 30.0;
        j.tau_max = 10.0;
        j.kp_max = 500.0;
        j.kd_max = 5.0;
        cfg.joints[i] = j;
    }
    return cfg;
}

ArmConfig ArmConfig::fromYaml(const std::string& yaml_path) {
    std::ifstream file(yaml_path);
    if (!file.is_open()) {
        throw std::runtime_error("ArmConfig::fromYaml: failed to open '" + yaml_path + "'");
    }

    ArmConfig cfg = defaultJ4310SixAxis();
    bool in_joints = false;
    int current_joint = -1;

    std::string line;
    while (std::getline(file, line)) {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        std::string t = trim(line);
        if (t.empty()) continue;

        if (t == "joints:") {
            in_joints = true;
            continue;
        }

        if (in_joints && t.rfind("- ", 0) == 0) {
            current_joint++;
            if (current_joint >= static_cast<int>(NUM_JOINTS)) {
                throw std::runtime_error("ArmConfig::fromYaml: more than 6 joints in '" + yaml_path + "'");
            }
            t = trim(t.substr(2));
            if (t.empty()) continue;
        }

        auto colon = t.find(':');
        if (colon == std::string::npos) continue;

        std::string key = trim(t.substr(0, colon));
        std::string val = trim(t.substr(colon + 1));
        if (val.empty()) continue;

        if (!in_joints) {
            if (key == "can_interface")         cfg.can_interface = val;
            else if (key == "enable_fd")        cfg.enable_fd = (val == "true" || val == "1");
            else if (key == "state_timeout_ms") cfg.state_timeout_ms = static_cast<uint32_t>(std::stoul(val));
            else if (key == "recv_timeout_us")  cfg.recv_timeout_us = std::stoi(val);
        } else if (current_joint >= 0 && current_joint < static_cast<int>(NUM_JOINTS)) {
            applyJointField(cfg.joints[static_cast<std::size_t>(current_joint)], key, val);
        }
    }

    if (in_joints && current_joint + 1 != static_cast<int>(NUM_JOINTS)) {
        throw std::runtime_error("ArmConfig::fromYaml: expected exactly 6 joints, got " +
                                 std::to_string(current_joint + 1));
    }

    cfg.validate();
    return cfg;
}

void ArmConfig::validate() const {
    if (can_interface.empty()) {
        throw std::invalid_argument("ArmConfig: can_interface cannot be empty");
    }

    std::set<uint32_t> send_ids;
    std::set<uint32_t> recv_ids;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& j = joints[i];
        if (j.send_can_id == 0 || j.send_can_id > 0x7FF) {
            throw std::invalid_argument("ArmConfig: invalid send_can_id at joint " + std::to_string(i));
        }
        if (j.recv_can_id == 0 || j.recv_can_id > 0x7FF) {
            throw std::invalid_argument("ArmConfig: invalid recv_can_id at joint " + std::to_string(i));
        }
        if (!send_ids.insert(j.send_can_id).second) {
            throw std::invalid_argument("ArmConfig: duplicate send_can_id at joint " + std::to_string(i));
        }
        if (!recv_ids.insert(j.recv_can_id).second) {
            throw std::invalid_argument("ArmConfig: duplicate recv_can_id at joint " + std::to_string(i));
        }
        if (!(std::isfinite(j.pos_min) && std::isfinite(j.pos_max) && j.pos_max >= j.pos_min &&
              std::isfinite(j.vel_max) && j.vel_max > 0.0 &&
              std::isfinite(j.tau_max) && j.tau_max > 0.0 &&
              std::isfinite(j.kp_max) && j.kp_max > 0.0 &&
              std::isfinite(j.kd_max) && j.kd_max > 0.0)) {
            throw std::invalid_argument("ArmConfig: invalid limits at joint " + std::to_string(i));
        }
    }
}

// ============================================================================
// Arm 实现
// ============================================================================

Arm::Arm(const ArmConfig& config) : config_(config) {
    config_.validate();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        state_.joints[i].send_can_id = config_.joints[i].send_can_id;
        state_.joints[i].recv_can_id = config_.joints[i].recv_can_id;
    }
}

void Arm::init() {
    // 3. 禁止危险重初始化：ARMED 和 FAULT 状态不能直接重新 init()
    if (status_ == ArmStatus::ARMED || status_ == ArmStatus::FAULT) {
        throw std::logic_error("Arm::init: forbidden while arm status is " +
                               std::string(toString(status_)) +
                               " (must be DISCONNECTED or DISARMED)");
    }

    config_.validate();

    std::vector<MotorType> types(NUM_JOINTS);
    std::vector<uint32_t> send_ids(NUM_JOINTS);
    std::vector<uint32_t> recv_ids(NUM_JOINTS);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        types[i] = config_.joints[i].motor_type;
        send_ids[i] = config_.joints[i].send_can_id;
        recv_ids[i] = config_.joints[i].recv_can_id;
    }

    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(config_.can_interface, config_.enable_fd);

    // P0 安全：初始化时不传 control_modes，避免在只读初始化阶段向电机写寄存器 (0x55)
    openarm_->init_arm_motors(types, send_ids, recv_ids);
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

    // 建立关节索引 i -> OpenArm 内部设备索引的映射（因为 OpenArm 内部 map 按 recv_can_id 排序）
    auto motors = openarm_->get_arm().get_motors();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        for (std::size_t dev_i = 0; dev_i < motors.size(); ++dev_i) {
            if (motors[dev_i].get_recv_can_id() == config_.joints[i].recv_can_id) {
                joint_to_dev_idx_[i] = static_cast<int>(dev_i);
                break;
            }
        }
    }

    status_ = ArmStatus::DISARMED;
    state_.status = status_;
    state_.fault_reason.clear();
    syncStateFromOpenArm();
}

int Arm::resolveDeviceIndex(std::size_t joint_idx) const {
    if (joint_idx >= NUM_JOINTS) {
        throw std::out_of_range("Arm: joint_idx out of range [0, 5]");
    }
    return joint_to_dev_idx_[joint_idx];
}

void Arm::syncStateFromOpenArm() {
    if (!openarm_) {
        state_.status = ArmStatus::DISCONNECTED;
        return;
    }

    const auto timeout = std::chrono::milliseconds(config_.state_timeout_ms);
    bool all_online = true;
    bool any_fault = false;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        int dev_idx = joint_to_dev_idx_[i];
        const auto motor = openarm_->get_arm().get_motor(dev_idx);
        const auto& link = openarm_->get_arm().get_link_stats(dev_idx);

        JointState& js = state_.joints[i];
        js.send_can_id = config_.joints[i].send_can_id;
        js.recv_can_id = config_.joints[i].recv_can_id;

        js.position = motor.get_position();
        js.velocity = motor.get_velocity();
        js.torque = motor.get_torque();
        js.t_mos = motor.get_state_tmos();
        js.t_rotor = motor.get_state_trotor();

        js.enabled = motor.is_enabled();
        js.error_code = motor.get_error_code();
        js.has_fault = motor.has_error();

        js.commands_sent = link.commands_sent;
        js.responses = link.responses;
        js.malformed_frames = link.malformed_frames;
        js.rejected_commands = link.rejected_commands;
        js.online = link.ever_responded();
        js.stale = link.is_stale(timeout);

        if (!js.online || js.stale) {
            all_online = false;
        }
        if (js.has_fault) {
            any_fault = true;
        }
    }

    state_.bus_healthy = openarm_->is_bus_healthy();
    state_.all_online = all_online;
    state_.any_fault = any_fault;
    state_.status = status_;
    state_.timestamp = std::chrono::steady_clock::now();
}

void Arm::enterFault(const std::string& reason) {
    status_ = ArmStatus::FAULT;
    state_.status = ArmStatus::FAULT;
    state_.fault_reason = reason;

    // 发生故障时立即尝试发送失能指令保护机械臂
    if (openarm_) {
        try {
            openarm_->disable_all();
        } catch (...) {
        }
    }
}

bool Arm::checkHealth() {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        return false;
    }

    syncStateFromOpenArm();

    if (status_ == ArmStatus::FAULT) {
        return false;
    }

    if (!state_.bus_healthy) {
        enterFault("CAN bus unhealthy");
        return false;
    }

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& js = state_.joints[i];
        if (js.has_fault) {
            enterFault("Motor hardware fault on " + config_.joints[i].name +
                       " (code=" + std::to_string(js.error_code) + ")");
            return false;
        }
        // 在已使能 (ARMED) 状态下，所有电机必须保持在线、有新鲜反馈且处于 enabled 状态
        if (status_ == ArmStatus::ARMED) {
            if (!js.online || js.stale) {
                enterFault("Communication timeout on " + config_.joints[i].name);
                return false;
            }
            if (!js.enabled) {
                enterFault("Motor unexpectedly disabled on " + config_.joints[i].name);
                return false;
            }
        }
    }

    return true;
}

void Arm::enable() {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::enable: arm is DISCONNECTED, call init() first");
    }
    if (status_ == ArmStatus::FAULT) {
        throw std::logic_error("Arm::enable: arm is in FAULT (" + state_.fault_reason + "), call clearFault() first");
    }

    // 1. 严格使能前检查：六台电机必须全部在线、有新鲜反馈且无故障
    syncStateFromOpenArm();
    if (!state_.bus_healthy) {
        enterFault("Arm::enable: CAN bus unhealthy before enable");
        throw std::runtime_error(state_.fault_reason);
    }
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& js = state_.joints[i];
        if (js.has_fault) {
            enterFault("Arm::enable: motor fault on " + config_.joints[i].name +
                       " (code=" + std::to_string(js.error_code) + ")");
            throw std::runtime_error(state_.fault_reason);
        }
        if (!js.online || js.stale) {
            throw std::runtime_error("Arm::enable: joint " + config_.joints[i].name +
                                     " is offline or feedback is stale (refresh state before enable)");
        }
    }

    // 若配置了非 MIT 模式，在使能前同步对应关节的控制模式并检查写入成功
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (config_.joints[i].control_mode != ControlMode::MIT) {
            uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
            openarm_->get_arm().set_control_mode_one(resolveDeviceIndex(i), config_.joints[i].control_mode);
            if (openarm_->get_bus_status().writes_ok != prev_writes + 1) {
                enterFault("Arm::enable: failed to write control mode on " + config_.joints[i].name);
                throw std::runtime_error(state_.fault_reason);
            }
        }
    }

    // 发送使能指令并检查 6 帧全部成功写入 SocketCAN
    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint64_t, NUM_JOINTS> prev_responses{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        prev_responses[i] = state_.joints[i].responses;
    }

    openarm_->enable_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + NUM_JOINTS) {
        enterFault("Arm::enable: CAN write failed during enable_all");
        throw std::runtime_error(state_.fault_reason);
    }

    openarm_->recv_all(config_.recv_timeout_us);
    syncStateFromOpenArm();

    // 确认六台电机全部返回了使能后的新反馈、无故障且均已进入 enabled (0x1) 状态
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& js = state_.joints[i];
        if (js.has_fault) {
            enterFault("Arm::enable: motor fault reported during enable on " + config_.joints[i].name);
            throw std::runtime_error(state_.fault_reason);
        }
        if (js.responses <= prev_responses[i] || js.stale || !js.enabled) {
            enterFault("Arm::enable: joint " + config_.joints[i].name + " did not confirm enabled state");
            throw std::runtime_error(state_.fault_reason);
        }
    }

    status_ = ArmStatus::ARMED;
    state_.status = ArmStatus::ARMED;
}

void Arm::disable() {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        return;
    }
    openarm_->disable_all();
    openarm_->recv_all(config_.recv_timeout_us);

    if (status_ != ArmStatus::FAULT) {
        status_ = ArmStatus::DISARMED;
    }
    syncStateFromOpenArm();
}

void Arm::clearFault() {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::clearFault: arm is DISCONNECTED");
    }

    // 2. 严格故障恢复：先清除总线错误计数并发送失能指令刷新全部电机状态，
    // 若故障条件（总线异常、电机离线/反馈过期、电机硬件故障或仍处于使能）未消除，禁止退出 FAULT！
    openarm_->clear_bus_status();
    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    openarm_->disable_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + NUM_JOINTS) {
        enterFault("Arm::clearFault: CAN write failed while disabling motors");
        throw std::runtime_error(state_.fault_reason);
    }

    openarm_->recv_all(config_.recv_timeout_us);
    syncStateFromOpenArm();

    if (!state_.bus_healthy) {
        enterFault("Arm::clearFault: CAN bus remains unhealthy");
        throw std::runtime_error(state_.fault_reason);
    }

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& js = state_.joints[i];
        if (js.has_fault) {
            enterFault("Arm::clearFault: fault still active on " + config_.joints[i].name +
                       " (code=" + std::to_string(js.error_code) + ")");
            throw std::runtime_error(state_.fault_reason);
        }
        if (!js.online || js.stale) {
            enterFault("Arm::clearFault: joint " + config_.joints[i].name +
                       " is offline or feedback is stale");
            throw std::runtime_error(state_.fault_reason);
        }
        if (js.enabled) {
            enterFault("Arm::clearFault: joint " + config_.joints[i].name +
                       " failed to confirm disabled state");
            throw std::runtime_error(state_.fault_reason);
        }
    }

    status_ = ArmStatus::DISARMED;
    state_.status = ArmStatus::DISARMED;
    state_.fault_reason.clear();
}

ArmState Arm::refresh(int timeout_us) {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::refresh: arm is DISCONNECTED");
    }
    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    openarm_->refresh_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + NUM_JOINTS) {
        enterFault("Arm::refresh: CAN write failed during refresh_all");
        throw std::runtime_error(state_.fault_reason);
    }
    recv(timeout_us);
    return state_;
}

void Arm::recv(int timeout_us) {
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::recv: arm is DISCONNECTED");
    }
    int wait_us = (timeout_us >= 0) ? timeout_us : config_.recv_timeout_us;
    openarm_->recv_all(wait_us);
    checkHealth();
}

// ============================================================================
// 指令校验 validateCommand
// ============================================================================

bool Arm::validateCommand(std::size_t joint_idx, const MITParam& cmd) const {
    if (joint_idx >= NUM_JOINTS) return false;
    const auto& jc = config_.joints[joint_idx];
    if (jc.control_mode != ControlMode::MIT) return false;

    if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
        !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd) || !std::isfinite(cmd.tau)) {
        return false;
    }
    if (cmd.q < jc.pos_min || cmd.q > jc.pos_max) return false;
    if (std::fabs(cmd.dq) > jc.vel_max) return false;
    if (std::fabs(cmd.tau) > jc.tau_max) return false;
    if (cmd.kp < 0.0 || cmd.kp > jc.kp_max) return false;
    if (cmd.kd < 0.0 || cmd.kd > jc.kd_max) return false;
    return true;
}

bool Arm::validateCommand(const std::array<MITParam, NUM_JOINTS>& cmds) const {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!validateCommand(i, cmds[i])) return false;
    }
    return true;
}

bool Arm::validateCommand(std::size_t joint_idx, const PosVelParam& cmd) const {
    if (joint_idx >= NUM_JOINTS) return false;
    const auto& jc = config_.joints[joint_idx];
    if (jc.control_mode != ControlMode::POS_VEL) return false;

    if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq)) return false;
    if (cmd.q < jc.pos_min || cmd.q > jc.pos_max) return false;
    if (std::fabs(cmd.dq) > jc.vel_max) return false;
    return true;
}

bool Arm::validateCommand(const std::array<PosVelParam, NUM_JOINTS>& cmds) const {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!validateCommand(i, cmds[i])) return false;
    }
    return true;
}

bool Arm::validateCommand(std::size_t joint_idx, const VelParam& cmd) const {
    if (joint_idx >= NUM_JOINTS) return false;
    const auto& jc = config_.joints[joint_idx];
    if (jc.control_mode != ControlMode::VEL) return false;

    if (!std::isfinite(cmd.dq)) return false;
    if (std::fabs(cmd.dq) > jc.vel_max) return false;
    return true;
}

bool Arm::validateCommand(const std::array<VelParam, NUM_JOINTS>& cmds) const {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!validateCommand(i, cmds[i])) return false;
    }
    return true;
}

// ============================================================================
// 控制指令发送与底层写入异常检查
// ============================================================================

void Arm::verifySendSucceeded(const char* api_name,
                              uint64_t prev_writes_ok,
                              std::size_t expected_writes,
                              const std::array<uint32_t, NUM_JOINTS>& prev_rejected) {
    // 检查是否有指令被 OpenArm 内部因模式不匹配而拒绝
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        uint32_t cur_rej = openarm_->get_arm().get_link_stats(resolveDeviceIndex(i)).rejected_commands;
        if (cur_rej > prev_rejected[i]) {
            enterFault(std::string(api_name) + ": command rejected by device on " + config_.joints[i].name);
            syncStateFromOpenArm();
            throw std::runtime_error(state_.fault_reason);
        }
    }

    // 检查底层 SocketCAN 是否全部写入成功（如部分写入失败、ENOBUFS、ENETDOWN）
    uint64_t actual_writes = openarm_->get_bus_status().writes_ok - prev_writes_ok;
    if (actual_writes != expected_writes || !openarm_->is_bus_healthy()) {
        int err = openarm_->get_bus_status().last_write_errno;
        std::string err_str = err ? std::string(std::strerror(err)) : "bus write error";
        enterFault(std::string(api_name) + ": CAN send failed (" +
                   std::to_string(actual_writes) + "/" + std::to_string(expected_writes) +
                   " frames written, " + err_str + ")");
        syncStateFromOpenArm();
        throw std::runtime_error(state_.fault_reason);
    }

    syncStateFromOpenArm();
}

void Arm::sendMIT(std::size_t joint_idx, const MITParam& cmd) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendMIT: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendMIT: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(joint_idx, cmd)) {
        throw std::invalid_argument("Arm::sendMIT: invalid MIT command on joint " + std::to_string(joint_idx));
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    openarm_->get_arm().mit_control_one(resolveDeviceIndex(joint_idx), cmd);
    verifySendSucceeded("Arm::sendMIT", prev_writes, 1, prev_rej);
}

void Arm::sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendMITAll: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendMITAll: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(cmds)) {
        throw std::invalid_argument("Arm::sendMITAll: invalid MIT commands");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        openarm_->get_arm().mit_control_one(resolveDeviceIndex(i), cmds[i]);
    }
    verifySendSucceeded("Arm::sendMITAll", prev_writes, NUM_JOINTS, prev_rej);
}

void Arm::sendPosVel(std::size_t joint_idx, const PosVelParam& cmd) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendPosVel: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendPosVel: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(joint_idx, cmd)) {
        throw std::invalid_argument("Arm::sendPosVel: invalid PosVel command on joint " + std::to_string(joint_idx));
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    openarm_->get_arm().posvel_control_one(resolveDeviceIndex(joint_idx), cmd);
    verifySendSucceeded("Arm::sendPosVel", prev_writes, 1, prev_rej);
}

void Arm::sendPosVelAll(const std::array<PosVelParam, NUM_JOINTS>& cmds) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendPosVelAll: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendPosVelAll: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(cmds)) {
        throw std::invalid_argument("Arm::sendPosVelAll: invalid PosVel commands");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        openarm_->get_arm().posvel_control_one(resolveDeviceIndex(i), cmds[i]);
    }
    verifySendSucceeded("Arm::sendPosVelAll", prev_writes, NUM_JOINTS, prev_rej);
}

void Arm::sendVel(std::size_t joint_idx, const VelParam& cmd) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendVel: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendVel: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(joint_idx, cmd)) {
        throw std::invalid_argument("Arm::sendVel: invalid Vel command on joint " + std::to_string(joint_idx));
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    openarm_->get_arm().vel_control_one(resolveDeviceIndex(joint_idx), cmd);
    verifySendSucceeded("Arm::sendVel", prev_writes, 1, prev_rej);
}

void Arm::sendVelAll(const std::array<VelParam, NUM_JOINTS>& cmds) {
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendVelAll: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendVelAll: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(cmds)) {
        throw std::invalid_argument("Arm::sendVelAll: invalid Vel commands");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        openarm_->get_arm().vel_control_one(resolveDeviceIndex(i), cmds[i]);
    }
    verifySendSucceeded("Arm::sendVelAll", prev_writes, NUM_JOINTS, prev_rej);
}

openarm::can::socket::OpenArm& Arm::rawOpenArm() {
    if (!openarm_) {
        throw std::logic_error("Arm::rawOpenArm: arm is not initialized");
    }
    return *openarm_;
}

} // namespace damiao
