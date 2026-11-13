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

bool validateMitLimits(const JointConfig& jc, const MITParam& cmd) {
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

void validateJointConfig(const JointConfig& j,
                         const std::string& label,
                         std::set<uint32_t>& send_ids,
                         std::set<uint32_t>& recv_ids) {
    if (j.send_can_id == 0 || j.send_can_id > 0x7FF) {
        throw std::invalid_argument("ArmConfig: invalid send_can_id at " + label);
    }
    if (j.recv_can_id == 0 || j.recv_can_id > 0x7FF) {
        throw std::invalid_argument("ArmConfig: invalid recv_can_id at " + label);
    }
    if (!send_ids.insert(j.send_can_id).second) {
        throw std::invalid_argument("ArmConfig: duplicate send_can_id at " + label);
    }
    if (!recv_ids.insert(j.recv_can_id).second) {
        throw std::invalid_argument("ArmConfig: duplicate recv_can_id at " + label);
    }
    if (!(std::isfinite(j.pos_min) && std::isfinite(j.pos_max) && j.pos_max >= j.pos_min &&
          std::isfinite(j.vel_max) && j.vel_max > 0.0 &&
          std::isfinite(j.tau_max) && j.tau_max > 0.0 &&
          std::isfinite(j.kp_max) && j.kp_max > 0.0 &&
          std::isfinite(j.kd_max) && j.kd_max > 0.0)) {
        throw std::invalid_argument("ArmConfig: invalid limits at " + label);
    }
}

struct StateCallbackGuard {
    openarm::can::socket::OpenArm& arm;
    openarm::canbus::CANSocket& sock;
    bool enable_fd;
    ~StateCallbackGuard() {
        try {
            int count = 0;
            while (count++ < 64 && sock.is_data_available(0)) {
                if (enable_fd) {
                    canfd_frame f{};
                    if (!sock.read_canfd_frame(f)) break;
                    if (f.can_id & CAN_ERR_FLAG) sock.get_bus_status().record_error_frame(f.can_id, f.data, f.len);
                } else {
                    can_frame f{};
                    if (!sock.read_can_frame(f)) break;
                    if (f.can_id & CAN_ERR_FLAG) sock.get_bus_status().record_error_frame(f.can_id, f.data, f.can_dlc);
                }
            }
            arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        } catch (...) {
        }
    }
};

void boundedDrainSocket(openarm::canbus::CANSocket& sock, bool enable_fd, int max_frames = 64) {
    int count = 0;
    while (count++ < max_frames && sock.is_data_available(0)) {
        if (enable_fd) {
            canfd_frame f{};
            if (!sock.read_canfd_frame(f)) break;
            if (f.can_id & CAN_ERR_FLAG) {
                sock.get_bus_status().record_error_frame(f.can_id, f.data, f.len);
            }
        } else {
            can_frame f{};
            if (!sock.read_can_frame(f)) break;
            if (f.can_id & CAN_ERR_FLAG) {
                sock.get_bus_status().record_error_frame(f.can_id, f.data, f.can_dlc);
            }
        }
    }
}

std::optional<uint32_t> transactTimeoutRegister(openarm::canbus::CANSocket& sock,
                                                bool enable_fd,
                                                uint32_t send_can_id,
                                                uint32_t recv_can_id,
                                                uint8_t cmd,
                                                uint32_t write_val,
                                                int timeout_ms) {
    constexpr uint8_t kTimeoutRid = static_cast<uint8_t>(openarm::damiao_motor::RID::TIMEOUT);
    bool sent = false;
    if (enable_fd) {
        canfd_frame f{};
        f.can_id = 0x7FF;
        f.len = 8;
        f.flags = CANFD_BRS;
        f.data[0] = static_cast<uint8_t>(send_can_id & 0xFF);
        f.data[1] = static_cast<uint8_t>((send_can_id >> 8) & 0xFF);
        f.data[2] = cmd;
        f.data[3] = kTimeoutRid;
        f.data[4] = static_cast<uint8_t>(write_val & 0xFF);
        f.data[5] = static_cast<uint8_t>((write_val >> 8) & 0xFF);
        f.data[6] = static_cast<uint8_t>((write_val >> 16) & 0xFF);
        f.data[7] = static_cast<uint8_t>((write_val >> 24) & 0xFF);
        sent = sock.write_canfd_frame(f);
    } else {
        can_frame f{};
        f.can_id = 0x7FF;
        f.can_dlc = 8;
        f.data[0] = static_cast<uint8_t>(send_can_id & 0xFF);
        f.data[1] = static_cast<uint8_t>((send_can_id >> 8) & 0xFF);
        f.data[2] = cmd;
        f.data[3] = kTimeoutRid;
        f.data[4] = static_cast<uint8_t>(write_val & 0xFF);
        f.data[5] = static_cast<uint8_t>((write_val >> 8) & 0xFF);
        f.data[6] = static_cast<uint8_t>((write_val >> 16) & 0xFF);
        f.data[7] = static_cast<uint8_t>((write_val >> 24) & 0xFF);
        sent = sock.write_can_frame(f);
    }
    if (!sent) {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        const auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining_us <= 0 || !sock.is_data_available(static_cast<int>(remaining_us))) {
            return std::nullopt;
        }
        canid_t rx_id = 0;
        uint8_t rx_len = 0;
        std::array<uint8_t, 8> rx_data{};
        if (enable_fd) {
            canfd_frame rf{};
            if (!sock.read_canfd_frame(rf)) continue;
            if (rf.can_id & CAN_ERR_FLAG) {
                sock.get_bus_status().record_error_frame(rf.can_id, rf.data, rf.len);
                continue;
            }
            rx_id = rf.can_id & CAN_EFF_MASK;
            rx_len = rf.len;
            if (rx_len >= 8) std::memcpy(rx_data.data(), rf.data, 8);
        } else {
            can_frame rf{};
            if (!sock.read_can_frame(rf)) continue;
            if (rf.can_id & CAN_ERR_FLAG) {
                sock.get_bus_status().record_error_frame(rf.can_id, rf.data, rf.can_dlc);
                continue;
            }
            rx_id = rf.can_id & CAN_EFF_MASK;
            rx_len = rf.can_dlc;
            if (rx_len >= 8) std::memcpy(rx_data.data(), rf.data, 8);
        }
        if (rx_len < 8) continue;
        const uint16_t from_id = static_cast<uint16_t>(rx_data[0]) |
                                 (static_cast<uint16_t>(rx_data[1]) << 8);
        if (rx_id == recv_can_id && from_id == send_can_id &&
            rx_data[2] == cmd && rx_data[3] == kTimeoutRid) {
            return static_cast<uint32_t>(rx_data[4]) |
                   (static_cast<uint32_t>(rx_data[5]) << 8) |
                   (static_cast<uint32_t>(rx_data[6]) << 16) |
                   (static_cast<uint32_t>(rx_data[7]) << 24);
        }
    }
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
    cfg.command_timeout_ms = 0;
    cfg.motor_timeout_reg = 0;
    cfg.recv_timeout_us = 1000;
    cfg.has_gripper = false;

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
    enum class Section { TOP, JOINTS, GRIPPER };
    Section section = Section::TOP;
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
            section = Section::JOINTS;
            continue;
        }
        if (t == "gripper:") {
            section = Section::GRIPPER;
            cfg.has_gripper = true;
            continue;
        }

        if (section == Section::JOINTS && t.rfind("- ", 0) == 0) {
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

        if (section == Section::TOP) {
            if (key == "can_interface")           cfg.can_interface = val;
            else if (key == "enable_fd")          cfg.enable_fd = (val == "true" || val == "1");
            else if (key == "state_timeout_ms")   cfg.state_timeout_ms = static_cast<uint32_t>(std::stoul(val));
            else if (key == "command_timeout_ms") cfg.command_timeout_ms = static_cast<uint32_t>(std::stoul(val));
            else if (key == "motor_timeout_reg")  cfg.motor_timeout_reg = static_cast<uint32_t>(std::stoul(val, nullptr, 0));
            else if (key == "recv_timeout_us")    cfg.recv_timeout_us = std::stoi(val);
            else if (key == "has_gripper")        cfg.has_gripper = (val == "true" || val == "1");
        } else if (section == Section::JOINTS && current_joint >= 0 && current_joint < static_cast<int>(NUM_JOINTS)) {
            applyJointField(cfg.joints[static_cast<std::size_t>(current_joint)], key, val);
        } else if (section == Section::GRIPPER) {
            if (key == "enabled") {
                cfg.has_gripper = (val == "true" || val == "1");
            } else {
                applyJointField(cfg.gripper, key, val);
            }
        }
    }

    if (current_joint >= 0 && current_joint + 1 != static_cast<int>(NUM_JOINTS)) {
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
        validateJointConfig(joints[i], "joint " + std::to_string(i), send_ids, recv_ids);
    }
    if (has_gripper) {
        validateJointConfig(gripper, "gripper", send_ids, recv_ids);
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
    state_.has_gripper = config_.has_gripper;
    if (config_.has_gripper) {
        state_.gripper.send_can_id = config_.gripper.send_can_id;
        state_.gripper.recv_can_id = config_.gripper.recv_can_id;
    }
}

Arm::~Arm() noexcept {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        stopping_ = true;
        monitor_cv_.notify_all();
    }
    if (monitor_.joinable()) monitor_.join();
    if (openarm_ && status_ == ArmStatus::ARMED) {
        try {
            disable();
        } catch (...) {
        }
    }
}

void Arm::init() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // 禁止危险重初始化：ARMED 和 FAULT 状态不能直接重新 init()
    if (status_ == ArmStatus::ARMED || status_ == ArmStatus::FAULT) {
        throw std::logic_error("Arm::init: forbidden while arm status is " +
                               std::string(toString(status_)) +
                               " (must be DISCONNECTED or DISARMED)");
    }

    config_.validate();

    const std::size_t motor_count = config_.activeMotorCount();
    std::vector<MotorType> types(motor_count);
    std::vector<uint32_t> send_ids(motor_count);
    std::vector<uint32_t> recv_ids(motor_count);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        types[i] = config_.joints[i].motor_type;
        send_ids[i] = config_.joints[i].send_can_id;
        recv_ids[i] = config_.joints[i].recv_can_id;
    }
    if (config_.has_gripper) {
        types[NUM_JOINTS] = config_.gripper.motor_type;
        send_ids[NUM_JOINTS] = config_.gripper.send_can_id;
        recv_ids[NUM_JOINTS] = config_.gripper.recv_can_id;
    }

    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(config_.can_interface, config_.enable_fd);

    // 初始化时不传 control_modes，避免在只读初始化阶段向电机写寄存器 (0x55)
    openarm_->init_arm_motors(types, send_ids, recv_ids);
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

    // 建立关节索引 -> OpenArm 内部设备索引的映射（OpenArm 内部 map 按 recv_can_id 排序）
    auto motors = openarm_->get_arm().get_motors();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        for (std::size_t dev_i = 0; dev_i < motors.size(); ++dev_i) {
            if (motors[dev_i].get_recv_can_id() == config_.joints[i].recv_can_id) {
                joint_to_dev_idx_[i] = static_cast<int>(dev_i);
                break;
            }
        }
    }
    gripper_dev_idx_ = -1;
    if (config_.has_gripper) {
        for (std::size_t dev_i = 0; dev_i < motors.size(); ++dev_i) {
            if (motors[dev_i].get_recv_can_id() == config_.gripper.recv_can_id) {
                gripper_dev_idx_ = static_cast<int>(dev_i);
                break;
            }
        }
    }

    last_command_time_ = std::chrono::steady_clock::time_point{};
    status_ = ArmStatus::DISARMED;
    state_.status = status_;
    state_.has_gripper = config_.has_gripper;
    state_.fault_reason.clear();
    syncStateFromOpenArm();
    if (!monitor_.joinable()) {
        stopping_ = false;
        monitor_ = std::thread(&Arm::monitorLoop, this);
    }
}

void Arm::monitorLoop() {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    while (!stopping_) {
        std::chrono::milliseconds period;
        if (status_ == ArmStatus::ARMED && config_.command_timeout_ms > 0) {
            period = std::chrono::milliseconds(
                std::max(1u, std::min(50u, config_.command_timeout_ms / 3)));
        } else {
            period = std::chrono::milliseconds(
                std::max(1u, std::min(100u, config_.state_timeout_ms / 3)));
        }

        if (monitor_cv_.wait_for(lock, period, [this] { return stopping_; })) {
            break;
        }
        if (status_ != ArmStatus::ARMED) {
            continue;
        }

        try {
            if (config_.command_timeout_ms > 0) {
                if (!checkHealth()) {
                    continue;
                }
                const auto now = std::chrono::steady_clock::now();
                if (last_command_time_ != std::chrono::steady_clock::time_point{} &&
                    (now - last_command_time_) > std::chrono::milliseconds(config_.command_timeout_ms)) {
                    enterFault("Command watchdog timeout (" +
                               std::to_string(config_.command_timeout_ms) +
                               " ms without control command)");
                }
            } else {
                if (checkHealth()) {
                    refresh(0);
                }
            }
        } catch (const std::exception& e) {
            enterFault(e.what());
        } catch (...) {
            enterFault("State monitor failed");
        }
    }
}

int Arm::resolveDeviceIndex(std::size_t joint_idx) const {
    if (joint_idx >= NUM_JOINTS) {
        throw std::out_of_range("Arm: joint_idx out of range [0, 5]");
    }
    return joint_to_dev_idx_[joint_idx];
}

bool Arm::pollUntilAllRespond(const std::array<uint64_t, NUM_JOINTS>& baseline_responses,
                              uint64_t gripper_baseline,
                              int total_timeout_ms,
                              bool check_enabled,
                              bool expected_enabled) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(total_timeout_ms);
    int slice_us = std::max(500, std::min(config_.recv_timeout_us, 2000));

    auto motorConfirmed = [&](int dev_idx, uint64_t baseline) {
        const auto& link = openarm_->get_arm().get_link_stats(dev_idx);
        const auto motor = openarm_->get_arm().get_motor(dev_idx);
        if (link.responses <= baseline || motor.has_error()) {
            return false;
        }
        if (check_enabled && motor.is_enabled() != expected_enabled) {
            return false;
        }
        return true;
    };

    while (true) {
        openarm_->recv_all(slice_us);

        bool all_replied = true;
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            if (!motorConfirmed(joint_to_dev_idx_[i], baseline_responses[i])) {
                all_replied = false;
                break;
            }
        }
        if (all_replied && config_.has_gripper) {
            if (gripper_dev_idx_ < 0 || !motorConfirmed(gripper_dev_idx_, gripper_baseline)) {
                all_replied = false;
            }
        }
        if (all_replied) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
    }
}

void Arm::syncStateFromOpenArm() {
    if (!openarm_) {
        state_.status = ArmStatus::DISCONNECTED;
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(config_.state_timeout_ms);
    bool all_online = true;
    bool any_fault = false;

    auto syncOne = [&](int dev_idx, const JointConfig& jc, JointState& js) {
        const auto motor = openarm_->get_arm().get_motor(dev_idx);
        const auto& link = openarm_->get_arm().get_link_stats(dev_idx);

        js.send_can_id = jc.send_can_id;
        js.recv_can_id = jc.recv_can_id;

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
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        syncOne(joint_to_dev_idx_[i], config_.joints[i], state_.joints[i]);
    }
    state_.has_gripper = config_.has_gripper;
    if (config_.has_gripper && gripper_dev_idx_ >= 0) {
        syncOne(gripper_dev_idx_, config_.gripper, state_.gripper);
    }

    state_.bus_healthy = openarm_->is_bus_healthy();
    state_.all_online = all_online;
    state_.any_fault = any_fault;
    state_.status = status_;
    state_.timestamp = now;
}

void Arm::enterFault(const std::string& reason) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        return false;
    }

    // 先非阻塞排空底层已到达的 CAN 反馈帧，避免因上层未及时调用 recv() 造成假性超时
    openarm_->recv_all(0);
    syncStateFromOpenArm();

    if (status_ == ArmStatus::FAULT) {
        return false;
    }

    if (!state_.bus_healthy) {
        enterFault("CAN bus unhealthy");
        return false;
    }

    auto checkJointHealth = [&](const JointConfig& jc, const JointState& js) -> bool {
        if (js.has_fault) {
            enterFault("Motor hardware fault on " + jc.name +
                       " (code=" + std::to_string(js.error_code) + ")");
            return false;
        }
        if (status_ == ArmStatus::ARMED) {
            if (!js.online) {
                enterFault("Motor offline on " + jc.name);
                return false;
            }
            if (!js.enabled) {
                enterFault("Motor unexpectedly disabled on " + jc.name);
                return false;
            }
            if (js.stale) {
                enterFault("Communication timeout on " + jc.name);
                return false;
            }
        }
        return true;
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!checkJointHealth(config_.joints[i], state_.joints[i])) {
            return false;
        }
    }
    if (config_.has_gripper) {
        if (!checkJointHealth(config_.gripper, state_.gripper)) {
            return false;
        }
    }

    return true;
}

void Arm::enable() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::enable: arm is DISCONNECTED, call init() first");
    }
    if (status_ == ArmStatus::FAULT) {
        throw std::logic_error("Arm::enable: arm is in FAULT (" + state_.fault_reason + "), call clearFault() first");
    }

    // 1. 使能前检查：全部配置电机必须在线、有新鲜反馈且无故障
    syncStateFromOpenArm();
    if (!state_.bus_healthy) {
        enterFault("Arm::enable: CAN bus unhealthy before enable");
        throw std::runtime_error(state_.fault_reason);
    }

    auto checkPreEnable = [&](const JointConfig& jc, const JointState& js) {
        if (js.has_fault) {
            enterFault("Arm::enable: motor fault on " + jc.name +
                       " (code=" + std::to_string(js.error_code) + ")");
            throw std::runtime_error(state_.fault_reason);
        }
        if (!js.online || js.stale) {
            throw std::runtime_error("Arm::enable: joint " + jc.name +
                                     " is offline or feedback is stale (refresh state before enable)");
        }
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        checkPreEnable(config_.joints[i], state_.joints[i]);
    }
    if (config_.has_gripper) {
        checkPreEnable(config_.gripper, state_.gripper);
    }

    // 若配置了非 MIT 模式，在 PARAM 回调模式下同步对应关节的控制模式并排空寄存器应答帧
    bool any_non_mit = (config_.has_gripper && config_.gripper.control_mode != ControlMode::MIT);
    for (std::size_t i = 0; i < NUM_JOINTS && !any_non_mit; ++i) {
        if (config_.joints[i].control_mode != ControlMode::MIT) {
            any_non_mit = true;
        }
    }
    if (any_non_mit) {
        StateCallbackGuard guard{*openarm_, openarm_->get_master_can_device_collection().get_can_socket(), config_.enable_fd};
        openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
        auto syncModeOne = [&](int dev_idx, const JointConfig& jc) {
            if (jc.control_mode != ControlMode::MIT) {
                uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
                openarm_->get_arm().set_control_mode_one(dev_idx, jc.control_mode);
                if (openarm_->get_bus_status().writes_ok != prev_writes + 1) {
                    enterFault("Arm::enable: failed to write control mode on " + jc.name);
                    throw std::runtime_error(state_.fault_reason);
                }
            }
        };
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            syncModeOne(resolveDeviceIndex(i), config_.joints[i]);
        }
        if (config_.has_gripper) {
            syncModeOne(gripper_dev_idx_, config_.gripper);
        }
        openarm_->recv_all(config_.recv_timeout_us);
    }
    if (any_non_mit) {
        syncStateFromOpenArm();
    }

    // 发送使能指令并检查全部帧成功写入 SocketCAN
    const std::size_t motor_count = config_.activeMotorCount();
    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint64_t, NUM_JOINTS> prev_responses{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        prev_responses[i] = openarm_->get_arm().get_link_stats(joint_to_dev_idx_[i]).responses;
    }
    uint64_t prev_gripper_responses =
        config_.has_gripper ? openarm_->get_arm().get_link_stats(gripper_dev_idx_).responses : 0;

    openarm_->enable_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + motor_count) {
        enterFault("Arm::enable: CAN write failed during enable_all");
        throw std::runtime_error(state_.fault_reason);
    }

    // 轮询等待全部电机返回使能后的状态帧（兼容 USB-CAN 桥接下串行回传延迟）
    pollUntilAllRespond(prev_responses, prev_gripper_responses, 50, true, true);
    syncStateFromOpenArm();

    auto checkConfirmedEnabled = [&](const JointConfig& jc, const JointState& js, uint64_t baseline) {
        if (js.has_fault) {
            enterFault("Arm::enable: motor fault reported during enable on " + jc.name);
            throw std::runtime_error(state_.fault_reason);
        }
        if (js.responses <= baseline || js.stale || !js.enabled) {
            enterFault("Arm::enable: joint " + jc.name + " did not confirm enabled state");
            throw std::runtime_error(state_.fault_reason);
        }
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        checkConfirmedEnabled(config_.joints[i], state_.joints[i], prev_responses[i]);
    }
    if (config_.has_gripper) {
        checkConfirmedEnabled(config_.gripper, state_.gripper, prev_gripper_responses);
    }

    last_command_time_ = std::chrono::steady_clock::now();
    status_ = ArmStatus::ARMED;
    state_.status = ArmStatus::ARMED;
    monitor_cv_.notify_all();
}

void Arm::disable() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        return;
    }
    // 先排空滞留帧再采集失能确认基线
    openarm_->recv_all(0);
    std::array<uint64_t, NUM_JOINTS> prev_responses{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        prev_responses[i] = openarm_->get_arm().get_link_stats(joint_to_dev_idx_[i]).responses;
    }
    uint64_t prev_gripper_responses =
        config_.has_gripper ? openarm_->get_arm().get_link_stats(gripper_dev_idx_).responses : 0;

    const auto writes = openarm_->get_bus_status().writes_ok;
    openarm_->disable_all();
    const bool written = openarm_->get_bus_status().writes_ok == writes + config_.activeMotorCount();
    const bool replied = pollUntilAllRespond(prev_responses, prev_gripper_responses, 50, true, false);
    syncStateFromOpenArm();

    bool confirmed = written && replied && state_.bus_healthy;
    for (const auto& joint : state_.joints) {
        confirmed = confirmed && joint.online && !joint.stale && !joint.enabled && !joint.has_fault;
    }
    if (config_.has_gripper) {
        const auto& g = state_.gripper;
        confirmed = confirmed && g.online && !g.stale && !g.enabled && !g.has_fault;
    }
    if (!confirmed) {
        enterFault("Arm::disable: failed to confirm fresh disabled feedback from all joints");
        throw std::runtime_error(state_.fault_reason);
    }
    if (status_ != ArmStatus::FAULT) {
        status_ = ArmStatus::DISARMED;
    }
    state_.status = status_;
}

void Arm::clearFault() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::clearFault: arm is DISCONNECTED");
    }

    // 故障恢复：清除总线错误计数并发送失能指令刷新全部电机状态，
    // 若故障条件（总线异常、电机离线/反馈过期、电机硬件故障或仍处于使能）未消除，禁止退出 FAULT
    openarm_->clear_bus_status();
    std::array<uint64_t, NUM_JOINTS> prev_responses{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        prev_responses[i] = openarm_->get_arm().get_link_stats(joint_to_dev_idx_[i]).responses;
    }
    uint64_t prev_gripper_responses =
        config_.has_gripper ? openarm_->get_arm().get_link_stats(gripper_dev_idx_).responses : 0;

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    openarm_->disable_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + config_.activeMotorCount()) {
        enterFault("Arm::clearFault: CAN write failed while disabling motors");
        throw std::runtime_error(state_.fault_reason);
    }

    pollUntilAllRespond(prev_responses, prev_gripper_responses, 50, true, false);
    syncStateFromOpenArm();

    if (!state_.bus_healthy) {
        enterFault("Arm::clearFault: CAN bus remains unhealthy");
        throw std::runtime_error(state_.fault_reason);
    }

    auto verifyCleared = [&](const JointConfig& jc, const JointState& js, uint64_t baseline) {
        if (js.has_fault) {
            enterFault("Arm::clearFault: fault still active on " + jc.name +
                       " (code=" + std::to_string(js.error_code) + ")");
            throw std::runtime_error(state_.fault_reason);
        }
        if (js.responses <= baseline || !js.online || js.stale) {
            enterFault("Arm::clearFault: joint " + jc.name + " is offline or feedback is stale");
            throw std::runtime_error(state_.fault_reason);
        }
        if (js.enabled) {
            enterFault("Arm::clearFault: joint " + jc.name + " failed to confirm disabled state");
            throw std::runtime_error(state_.fault_reason);
        }
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        verifyCleared(config_.joints[i], state_.joints[i], prev_responses[i]);
    }
    if (config_.has_gripper) {
        verifyCleared(config_.gripper, state_.gripper, prev_gripper_responses);
    }

    status_ = ArmStatus::DISARMED;
    state_.status = ArmStatus::DISARMED;
    state_.fault_reason.clear();
}

ArmState Arm::refresh(int timeout_us) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::refresh: arm is DISCONNECTED");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    openarm_->refresh_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + config_.activeMotorCount()) {
        enterFault("Arm::refresh: CAN write failed during refresh_all");
        throw std::runtime_error(state_.fault_reason);
    }
    recv(timeout_us);
    return state_;
}

bool Arm::refreshUntilAllFresh(int timeout_ms) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::refreshUntilAllFresh: arm is DISCONNECTED");
    }

    openarm_->recv_all(0);
    std::array<uint64_t, NUM_JOINTS> prev_responses{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        prev_responses[i] = openarm_->get_arm().get_link_stats(joint_to_dev_idx_[i]).responses;
    }
    uint64_t prev_gripper_responses =
        config_.has_gripper ? openarm_->get_arm().get_link_stats(gripper_dev_idx_).responses : 0;

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    openarm_->refresh_all();
    if (openarm_->get_bus_status().writes_ok != prev_writes + config_.activeMotorCount()) {
        enterFault("Arm::refreshUntilAllFresh: CAN write failed during refresh_all");
        throw std::runtime_error(state_.fault_reason);
    }

    const bool replied = pollUntilAllRespond(prev_responses, prev_gripper_responses, timeout_ms, false, false);
    checkHealth();
    return replied && state_.bus_healthy && state_.all_online && !state_.any_fault;
}

void Arm::recv(int timeout_us) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ == ArmStatus::DISCONNECTED) {
        throw std::logic_error("Arm::recv: arm is DISCONNECTED");
    }
    int wait_us = (timeout_us >= 0) ? timeout_us : config_.recv_timeout_us;
    openarm_->recv_all(wait_us);
    checkHealth();
}

std::vector<std::optional<uint32_t>> Arm::queryMotorTimeouts(int timeout_ms) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ != ArmStatus::DISARMED) {
        throw std::logic_error("Arm::queryMotorTimeouts: arm must be DISARMED");
    }

    auto& can_sock = openarm_->get_master_can_device_collection().get_can_socket();
    StateCallbackGuard guard{*openarm_, can_sock, config_.enable_fd};
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
    boundedDrainSocket(can_sock, config_.enable_fd);

    const int per_motor_ms = std::max(15, timeout_ms);
    std::vector<std::optional<uint32_t>> results(config_.activeMotorCount(), std::nullopt);
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        results[i] = transactTimeoutRegister(
            can_sock, config_.enable_fd,
            config_.joints[i].send_can_id, config_.joints[i].recv_can_id,
            0x33, 0, per_motor_ms);
    }
    if (config_.has_gripper) {
        results[NUM_JOINTS] = transactTimeoutRegister(
            can_sock, config_.enable_fd,
            config_.gripper.send_can_id, config_.gripper.recv_can_id,
            0x33, 0, per_motor_ms);
    }
    return results;
}

bool Arm::configureMotorTimeout(uint32_t raw_timeout_reg, int timeout_ms) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!openarm_ || status_ != ArmStatus::DISARMED) {
        throw std::logic_error("Arm::configureMotorTimeout: arm must be DISARMED");
    }

    auto& can_sock = openarm_->get_master_can_device_collection().get_can_socket();
    StateCallbackGuard guard{*openarm_, can_sock, config_.enable_fd};
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
    boundedDrainSocket(can_sock, config_.enable_fd);

    const int per_motor_ms = std::max(15, timeout_ms);
    auto writeAndVerifyOne = [&](const JointConfig& jc) -> bool {
        auto ack = transactTimeoutRegister(
            can_sock, config_.enable_fd, jc.send_can_id, jc.recv_can_id,
            0x55, raw_timeout_reg, per_motor_ms);
        if (!ack.has_value()) {
            return false;
        }
        auto readback = transactTimeoutRegister(
            can_sock, config_.enable_fd, jc.send_can_id, jc.recv_can_id,
            0x33, 0, per_motor_ms);
        return readback.has_value() && *readback == raw_timeout_reg;
    };

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!writeAndVerifyOne(config_.joints[i])) {
            return false;
        }
    }
    if (config_.has_gripper && !writeAndVerifyOne(config_.gripper)) {
        return false;
    }
    return true;
}

// ============================================================================
// 指令校验 validateCommand
// ============================================================================

bool Arm::validateCommand(std::size_t joint_idx, const MITParam& cmd) const {
    if (joint_idx >= NUM_JOINTS) return false;
    return validateMitLimits(config_.joints[joint_idx], cmd);
}

bool Arm::validateCommand(const std::array<MITParam, NUM_JOINTS>& cmds) const {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!validateCommand(i, cmds[i])) return false;
    }
    return true;
}

bool Arm::validateGripperCommand(const MITParam& cmd) const {
    if (!config_.has_gripper) return false;
    return validateMitLimits(config_.gripper, cmd);
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
                              const std::array<uint32_t, NUM_JOINTS>& prev_rejected,
                              uint32_t prev_gripper_rejected) {
    // 检查是否有指令被 OpenArm 内部因模式不匹配而拒绝
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& link = openarm_->get_arm().get_link_stats(joint_to_dev_idx_[i]);
        state_.joints[i].commands_sent = link.commands_sent;
        state_.joints[i].rejected_commands = link.rejected_commands;
        if (link.rejected_commands > prev_rejected[i]) {
            enterFault(std::string(api_name) + ": command rejected by device on " + config_.joints[i].name);
            syncStateFromOpenArm();
            throw std::runtime_error(state_.fault_reason);
        }
    }
    if (config_.has_gripper && gripper_dev_idx_ >= 0) {
        const auto& link = openarm_->get_arm().get_link_stats(gripper_dev_idx_);
        state_.gripper.commands_sent = link.commands_sent;
        state_.gripper.rejected_commands = link.rejected_commands;
        if (link.rejected_commands > prev_gripper_rejected) {
            enterFault(std::string(api_name) + ": command rejected by device on " + config_.gripper.name);
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

    // 成功写入控制指令后喂给命令看门狗
    last_command_time_ = std::chrono::steady_clock::now();

    // 发送后非阻塞排空已到达的反馈帧，避免高频循环中接收队列积压
    openarm_->recv_all(0);
    syncStateFromOpenArm();
}

void Arm::sendMIT(std::size_t joint_idx, const MITParam& cmd) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendMIT", prev_writes, 1, prev_rej, state_.gripper.rejected_commands);
}

void Arm::sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendMITAll", prev_writes, NUM_JOINTS, prev_rej, state_.gripper.rejected_commands);
}

void Arm::sendMITAll(const std::array<MITParam, NUM_JOINTS>& cmds, const MITParam& gripper_cmd) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!config_.has_gripper || gripper_dev_idx_ < 0) {
        throw std::logic_error("Arm::sendMITAll(arm, gripper): gripper is not configured");
    }
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendMITAll: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendMITAll: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateCommand(cmds) || !validateGripperCommand(gripper_cmd)) {
        throw std::invalid_argument("Arm::sendMITAll: invalid arm or gripper MIT command");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;
    uint32_t prev_gripper_rej = state_.gripper.rejected_commands;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        openarm_->get_arm().mit_control_one(resolveDeviceIndex(i), cmds[i]);
    }
    openarm_->get_arm().mit_control_one(gripper_dev_idx_, gripper_cmd);
    verifySendSucceeded("Arm::sendMITAll", prev_writes, NUM_JOINTS + 1, prev_rej, prev_gripper_rej);
}

void Arm::sendGripperMIT(const MITParam& cmd) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!config_.has_gripper || gripper_dev_idx_ < 0) {
        throw std::logic_error("Arm::sendGripperMIT: gripper is not configured");
    }
    if (status_ != ArmStatus::ARMED) {
        throw std::logic_error("Arm::sendGripperMIT: rejected, arm status is " + std::string(toString(status_)));
    }
    if (!checkHealth()) {
        throw std::runtime_error("Arm::sendGripperMIT: health check failed (" + state_.fault_reason + ")");
    }
    if (!validateGripperCommand(cmd)) {
        throw std::invalid_argument("Arm::sendGripperMIT: invalid gripper MIT command");
    }

    uint64_t prev_writes = openarm_->get_bus_status().writes_ok;
    std::array<uint32_t, NUM_JOINTS> prev_rej{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) prev_rej[i] = state_.joints[i].rejected_commands;
    uint32_t prev_gripper_rej = state_.gripper.rejected_commands;

    openarm_->get_arm().mit_control_one(gripper_dev_idx_, cmd);
    verifySendSucceeded("Arm::sendGripperMIT", prev_writes, 1, prev_rej, prev_gripper_rej);
}

void Arm::sendPosVel(std::size_t joint_idx, const PosVelParam& cmd) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendPosVel", prev_writes, 1, prev_rej, state_.gripper.rejected_commands);
}

void Arm::sendPosVelAll(const std::array<PosVelParam, NUM_JOINTS>& cmds) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendPosVelAll", prev_writes, NUM_JOINTS, prev_rej, state_.gripper.rejected_commands);
}

void Arm::sendVel(std::size_t joint_idx, const VelParam& cmd) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendVel", prev_writes, 1, prev_rej, state_.gripper.rejected_commands);
}

void Arm::sendVelAll(const std::array<VelParam, NUM_JOINTS>& cmds) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    verifySendSucceeded("Arm::sendVelAll", prev_writes, NUM_JOINTS, prev_rej, state_.gripper.rejected_commands);
}

openarm::can::socket::OpenArm& Arm::rawOpenArm() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (status_ == ArmStatus::ARMED) {
        throw std::logic_error("Arm::rawOpenArm: disable arm before direct diagnostic access");
    }
    if (!openarm_) {
        throw std::logic_error("Arm::rawOpenArm: arm is not initialized");
    }
    return *openarm_;
}

} // namespace damiao
