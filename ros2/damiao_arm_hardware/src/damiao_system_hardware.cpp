#include "damiao_arm_hardware/damiao_system_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace damiao_arm_hardware {

namespace {

const rclcpp::Logger& logger() {
    static const rclcpp::Logger kLogger = rclcpp::get_logger("DamiaoSystemHardware");
    return kLogger;
}

bool parseBool(const std::string& raw) {
    std::string lower = raw;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower == "true" || lower == "1" || lower == "yes";
}

std::string getParamOr(const std::unordered_map<std::string, std::string>& params,
                       const std::string& key,
                       const std::string& fallback) {
    auto it = params.find(key);
    return (it != params.end()) ? it->second : fallback;
}

damiao::MotorType parseMotorType(const std::string& s) {
    if (s == "DM3507")     return damiao::MotorType::DM3507;
    if (s == "DM4310" || s == "DM_J4310_2EC" || s == "J4310") return damiao::MotorType::DM4310;
    if (s == "DM4310_48V") return damiao::MotorType::DM4310_48V;
    if (s == "DM4340" || s == "DM_J4340_2EC" || s == "J4340") return damiao::MotorType::DM4340;
    if (s == "DM4340_48V") return damiao::MotorType::DM4340_48V;
    if (s == "DM6006")     return damiao::MotorType::DM6006;
    if (s == "DM8006")     return damiao::MotorType::DM8006;
    if (s == "DM8009" || s == "DM_J8009" || s == "J8009")     return damiao::MotorType::DM8009;
    if (s == "DM10010L")   return damiao::MotorType::DM10010L;
    if (s == "DM10010")    return damiao::MotorType::DM10010;
    if (s == "DMH3510")    return damiao::MotorType::DMH3510;
    if (s == "DMH6215")    return damiao::MotorType::DMH6215;
    if (s == "DMG6220")    return damiao::MotorType::DMG6220;
    throw std::invalid_argument("Unknown motor_type: '" + s + "'");
}

} // namespace

DamiaoSystemHardware::~DamiaoSystemHardware() {
    teardownHardware();
}

void DamiaoSystemHardware::teardownHardware() noexcept {
    if (arm_) {
        try {
            if (arm_->status() == damiao::ArmStatus::ARMED) {
                arm_->disable();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger(), "Exception during teardown disable: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(logger(), "Unknown exception during teardown disable");
        }
        arm_.reset();
    }
    if (bridge_) {
        try {
            bridge_->stop();
        } catch (...) {
        }
        bridge_.reset();
    }
}

bool DamiaoSystemHardware::parseHardwareParameters(const hardware_interface::HardwareInfo& info) {
    const auto& hp = info.hardware_parameters;

    const std::string yaml_path = getParamOr(hp, "arm_config_yaml", "");
    if (!yaml_path.empty()) {
        arm_config_ = damiao::ArmConfig::fromYaml(yaml_path);
    } else {
        arm_config_ = damiao::ArmConfig::defaultJ4310SixAxis(getParamOr(hp, "can_interface", "vcan0"));
    }

    if (hp.count("can_interface")) {
        arm_config_.can_interface = hp.at("can_interface");
    }
    if (hp.count("enable_fd")) {
        arm_config_.enable_fd = parseBool(hp.at("enable_fd"));
    }
    if (hp.count("state_timeout_ms")) {
        arm_config_.state_timeout_ms = static_cast<uint32_t>(std::stoul(hp.at("state_timeout_ms")));
    }
    // In ros2_control, default command_timeout_ms to 100 ms unless explicitly overridden
    if (hp.count("command_timeout_ms")) {
        arm_config_.command_timeout_ms = static_cast<uint32_t>(std::stoul(hp.at("command_timeout_ms")));
    } else if (yaml_path.empty()) {
        arm_config_.command_timeout_ms = 100;
    }
    if (hp.count("motor_timeout_reg")) {
        arm_config_.motor_timeout_reg = static_cast<uint32_t>(std::stoul(hp.at("motor_timeout_reg"), nullptr, 0));
    }
    if (hp.count("recv_timeout_us")) {
        arm_config_.recv_timeout_us = std::stoi(hp.at("recv_timeout_us"));
    }
    if (hp.count("has_gripper")) {
        arm_config_.has_gripper = parseBool(hp.at("has_gripper"));
    } else if (info.joints.size() == damiao::NUM_JOINTS + 1) {
        arm_config_.has_gripper = true;
    }

    const std::size_t expected_joints = arm_config_.activeMotorCount();
    if (info.joints.size() != expected_joints) {
        RCLCPP_ERROR(logger(), "Expected %zu URDF joints (has_gripper=%s), got %zu",
                     expected_joints, arm_config_.has_gripper ? "true" : "false", info.joints.size());
        return false;
    }

    use_usb_bridge_ = parseBool(getParamOr(hp, "use_usb_bridge", "false"));
    bridge_config_.socketcan_if = arm_config_.can_interface;
    bridge_config_.serial_port = getParamOr(hp, "serial_port", "/dev/ttyACM0");
    bridge_config_.baudrate = std::stoi(getParamOr(hp, "baudrate", "921600"));
    bridge_config_.auto_reconnect = parseBool(getParamOr(hp, "bridge_auto_reconnect", "true"));
    bridge_config_.read_only = false;

    const double global_kp = std::stod(getParamOr(hp, "default_kp", "25.0"));
    const double global_kd = std::stod(getParamOr(hp, "default_kd", "1.0"));
    const double gripper_kp = std::stod(getParamOr(hp, "gripper_kp", "8.0"));
    const double gripper_kd = std::stod(getParamOr(hp, "gripper_kd", "0.2"));

    joint_names_.clear();
    joint_index_by_name_.clear();
    default_kp_.assign(expected_joints, global_kp);
    default_kd_.assign(expected_joints, global_kd);

    for (std::size_t i = 0; i < expected_joints; ++i) {
        const auto& jinfo = info.joints[i];
        joint_names_.push_back(jinfo.name);
        joint_index_by_name_[jinfo.name] = i;

        damiao::JointConfig& jc = (i < damiao::NUM_JOINTS) ? arm_config_.joints[i] : arm_config_.gripper;
        jc.name = jinfo.name;
        jc.control_mode = damiao::ControlMode::MIT; // ros2_control assembly uses MIT mode

        const auto& jp = jinfo.parameters;
        if (jp.count("motor_type"))  jc.motor_type = parseMotorType(jp.at("motor_type"));
        if (jp.count("send_can_id")) jc.send_can_id = static_cast<uint32_t>(std::stoul(jp.at("send_can_id"), nullptr, 0));
        if (jp.count("recv_can_id")) jc.recv_can_id = static_cast<uint32_t>(std::stoul(jp.at("recv_can_id"), nullptr, 0));
        if (jp.count("pos_min"))     jc.pos_min = std::stod(jp.at("pos_min"));
        if (jp.count("pos_max"))     jc.pos_max = std::stod(jp.at("pos_max"));
        if (jp.count("vel_max"))     jc.vel_max = std::stod(jp.at("vel_max"));
        if (jp.count("tau_max"))     jc.tau_max = std::stod(jp.at("tau_max"));
        if (jp.count("kp_max"))      jc.kp_max = std::stod(jp.at("kp_max"));
        if (jp.count("kd_max"))      jc.kd_max = std::stod(jp.at("kd_max"));

        if (i >= damiao::NUM_JOINTS) {
            default_kp_[i] = gripper_kp;
            default_kd_[i] = gripper_kd;
        }
        if (jp.count("kp")) default_kp_[i] = std::stod(jp.at("kp"));
        if (jp.count("kd")) default_kd_[i] = std::stod(jp.at("kd"));

        if (default_kp_[i] < 0.0 || default_kp_[i] > jc.kp_max ||
            default_kd_[i] < 0.0 || default_kd_[i] > jc.kd_max) {
            RCLCPP_ERROR(logger(), "Default kp/kd out of limits on joint '%s'", jinfo.name.c_str());
            return false;
        }
    }

    arm_config_.validate();
    return true;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    try {
        if (!parseHardwareParameters(info_)) {
            return CallbackReturn::ERROR;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "on_init parameter error: %s", e.what());
        return CallbackReturn::ERROR;
    }

    const std::size_t n = joint_names_.size();
    pos_states_.assign(n, 0.0);
    vel_states_.assign(n, 0.0);
    tau_states_.assign(n, 0.0);

    pos_commands_.assign(n, 0.0);
    vel_commands_.assign(n, 0.0);
    tau_commands_.assign(n, 0.0);
    kp_commands_ = default_kp_;
    kd_commands_ = default_kd_;

    pos_latched_.assign(n, 0.0);
    claimed_mask_.assign(n, CLAIM_NONE);

    RCLCPP_INFO(logger(),
                "Initialized DamiaoSystemHardware: if=%s, joints=%zu, cmd_watchdog=%u ms, motor_timeout_reg=%u, usb_bridge=%s",
                arm_config_.can_interface.c_str(), n, arm_config_.command_timeout_ms,
                arm_config_.motor_timeout_reg, use_usb_bridge_ ? "true" : "false");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    try {
        teardownHardware();

        if (use_usb_bridge_) {
            bridge_ = std::make_unique<damiao::SocketCanUsbBridge>(bridge_config_);
            bridge_->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        arm_ = std::make_unique<damiao::Arm>(arm_config_);
        arm_->init();

        if (arm_config_.motor_timeout_reg > 0) {
            if (!arm_->configureMotorTimeout(arm_config_.motor_timeout_reg, 100)) {
                RCLCPP_ERROR(logger(),
                             "on_configure: failed to write/verify motor RID::TIMEOUT=%u across all motors",
                             arm_config_.motor_timeout_reg);
                teardownHardware();
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(logger(), "Verified motor RID::TIMEOUT=%u on all %zu motors",
                        arm_config_.motor_timeout_reg, joint_names_.size());
        } else {
            const auto timeouts = arm_->queryMotorTimeouts(80);
            for (std::size_t i = 0; i < timeouts.size(); ++i) {
                if (!timeouts[i].has_value() || *timeouts[i] == 0) {
                    RCLCPP_WARN(logger(),
                                "Joint '%s' motor RID::TIMEOUT is %s (hardware CAN timeout protection disabled; relying on host watchdog %u ms)",
                                joint_names_[i].c_str(),
                                timeouts[i].has_value() ? "0" : "unreadable",
                                arm_config_.command_timeout_ms);
                }
            }
        }

        // Refresh all state frames AFTER parameter transactions so initial buffers are clean
        if (!arm_->refreshUntilAllFresh(100)) {
            RCLCPP_ERROR(logger(), "on_configure: failed to receive fresh feedback from all %zu motors",
                         joint_names_.size());
            teardownHardware();
            return CallbackReturn::ERROR;
        }

        copyArmStateToBuffers(arm_->getState());
        latchAllCommandsToCurrentState();
        last_disarmed_refresh_ = std::chrono::steady_clock::now();
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "on_configure exception: %s", e.what());
        teardownHardware();
        return CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    teardownHardware();
    std::fill(claimed_mask_.begin(), claimed_mask_.end(), CLAIM_NONE);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!arm_) {
        RCLCPP_ERROR(logger(), "on_activate: arm is not configured");
        return CallbackReturn::ERROR;
    }

    try {
        if (arm_->status() == damiao::ArmStatus::FAULT) {
            RCLCPP_ERROR(logger(), "on_activate: refusing activation while Arm is in FAULT (%s)",
                         arm_->getState().fault_reason.c_str());
            return CallbackReturn::ERROR;
        }

        if (!arm_->refreshUntilAllFresh(100)) {
            RCLCPP_ERROR(logger(), "on_activate: motors not fresh/online before enable");
            return CallbackReturn::ERROR;
        }

        const auto pre_state = arm_->getState();
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            const auto& js = (i < damiao::NUM_JOINTS) ? pre_state.joints[i] : pre_state.gripper;
            if (!js.online || js.stale || js.has_fault || !std::isfinite(js.position)) {
                RCLCPP_ERROR(logger(), "on_activate: invalid pre-enable state on '%s'",
                             joint_names_[i].c_str());
                return CallbackReturn::ERROR;
            }
        }

        copyArmStateToBuffers(pre_state);
        latchAllCommandsToCurrentState();

        arm_->enable();

        copyArmStateToBuffers(arm_->getState());
        latchAllCommandsToCurrentState();

        RCLCPP_INFO(logger(), "DamiaoSystemHardware activated (all %zu motors enabled, holding current position)",
                    joint_names_.size());
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "on_activate exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!arm_) {
        return CallbackReturn::SUCCESS;
    }
    try {
        arm_->disable();
        copyArmStateToBuffers(arm_->getState());
        RCLCPP_INFO(logger(), "DamiaoSystemHardware deactivated (all motors confirmed disabled)");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "on_deactivate exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    teardownHardware();
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    teardownHardware();
    std::fill(claimed_mask_.begin(), claimed_mask_.end(), CLAIM_NONE);
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DamiaoSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(joint_names_.size() * 3);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DamiaoSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(joint_names_.size() * 5);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
        command_interfaces.emplace_back(
            joint_names_[i], "kp", &kp_commands_[i]);
        command_interfaces.emplace_back(
            joint_names_[i], "kd", &kd_commands_[i]);
    }
    return command_interfaces;
}

bool DamiaoSystemHardware::isSupportedClaimMask(uint8_t mask) noexcept {
    if (mask == CLAIM_NONE) return true;
    // Full MIT (requires at least kp or kd along with pos/vel/eff) or explicit gain claims
    if ((mask & CLAIM_KP) || (mask & CLAIM_KD)) {
        // Require at least one motion/effort channel when commanding kp/kd
        return (mask & (CLAIM_POS | CLAIM_VEL | CLAIM_EFF)) != 0;
    }
    // Position (with optional vel/eff feedforward), Velocity, or Effort
    if (mask & CLAIM_POS) return true;
    if (mask == CLAIM_VEL) return true;
    if (mask == CLAIM_EFF) return true;
    return false;
}

bool DamiaoSystemHardware::computeCandidateClaims(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces,
    std::vector<uint8_t>& candidate_claims) const {
    candidate_claims = claimed_mask_;

    auto parseInterfaceBit = [&](const std::string& full_name,
                                 std::size_t& out_idx,
                                 uint8_t& out_bit) -> bool {
        const auto slash = full_name.find('/');
        if (slash == std::string::npos) return false;
        const std::string jname = full_name.substr(0, slash);
        const std::string ifname = full_name.substr(slash + 1);

        auto it = joint_index_by_name_.find(jname);
        if (it == joint_index_by_name_.end()) {
            return false; // unrelated resource
        }
        out_idx = it->second;

        if (ifname == hardware_interface::HW_IF_POSITION)      out_bit = CLAIM_POS;
        else if (ifname == hardware_interface::HW_IF_VELOCITY) out_bit = CLAIM_VEL;
        else if (ifname == hardware_interface::HW_IF_EFFORT)   out_bit = CLAIM_EFF;
        else if (ifname == "kp")                               out_bit = CLAIM_KP;
        else if (ifname == "kd")                               out_bit = CLAIM_KD;
        else return false;

        return true;
    };

    for (const auto& iface : stop_interfaces) {
        std::size_t idx = 0;
        uint8_t bit = CLAIM_NONE;
        if (parseInterfaceBit(iface, idx, bit)) {
            candidate_claims[idx] = static_cast<uint8_t>(candidate_claims[idx] & ~bit);
        }
    }
    for (const auto& iface : start_interfaces) {
        std::size_t idx = 0;
        uint8_t bit = CLAIM_NONE;
        if (parseInterfaceBit(iface, idx, bit)) {
            candidate_claims[idx] = static_cast<uint8_t>(candidate_claims[idx] | bit);
        }
    }

    for (std::size_t i = 0; i < candidate_claims.size(); ++i) {
        if (!isSupportedClaimMask(candidate_claims[i])) {
            RCLCPP_ERROR(logger(), "Unsupported command interface combination (mask=0x%02X) on joint '%s'",
                         candidate_claims[i], joint_names_[i].c_str());
            return false;
        }
    }
    return true;
}

hardware_interface::return_type DamiaoSystemHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
    std::vector<uint8_t> candidate_claims;
    if (!computeCandidateClaims(start_interfaces, stop_interfaces, candidate_claims)) {
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DamiaoSystemHardware::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
    std::vector<uint8_t> candidate_claims;
    if (!computeCandidateClaims(start_interfaces, stop_interfaces, candidate_claims)) {
        return hardware_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        if (candidate_claims[i] != claimed_mask_[i]) {
            const damiao::JointConfig& jc =
                (i < damiao::NUM_JOINTS) ? arm_config_.joints[i] : arm_config_.gripper;
            double safe_pos = std::isfinite(pos_states_[i]) ? pos_states_[i] : 0.0;
            safe_pos = std::clamp(safe_pos, jc.pos_min, jc.pos_max);

            pos_latched_[i] = safe_pos;
            pos_commands_[i] = safe_pos;
            vel_commands_[i] = 0.0;
            tau_commands_[i] = 0.0;
            kp_commands_[i] = default_kp_[i];
            kd_commands_[i] = default_kd_[i];
        }
    }

    claimed_mask_ = std::move(candidate_claims);
    return hardware_interface::return_type::OK;
}

void DamiaoSystemHardware::copyArmStateToBuffers(const damiao::ArmState& state) {
    for (std::size_t i = 0; i < damiao::NUM_JOINTS && i < joint_names_.size(); ++i) {
        pos_states_[i] = state.joints[i].position;
        vel_states_[i] = state.joints[i].velocity;
        tau_states_[i] = state.joints[i].torque;
    }
    if (arm_config_.has_gripper && joint_names_.size() > damiao::NUM_JOINTS) {
        pos_states_[damiao::NUM_JOINTS] = state.gripper.position;
        vel_states_[damiao::NUM_JOINTS] = state.gripper.velocity;
        tau_states_[damiao::NUM_JOINTS] = state.gripper.torque;
    }
}

void DamiaoSystemHardware::latchAllCommandsToCurrentState() {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        const damiao::JointConfig& jc =
            (i < damiao::NUM_JOINTS) ? arm_config_.joints[i] : arm_config_.gripper;
        double safe_pos = std::isfinite(pos_states_[i]) ? pos_states_[i] : 0.0;
        safe_pos = std::clamp(safe_pos, jc.pos_min, jc.pos_max);
        pos_latched_[i] = safe_pos;
        pos_commands_[i] = safe_pos;
        vel_commands_[i] = 0.0;
        tau_commands_[i] = 0.0;
        kp_commands_[i] = default_kp_[i];
        kd_commands_[i] = default_kd_[i];
    }
}

damiao::MITParam DamiaoSystemHardware::buildMitCommand(std::size_t idx, const damiao::JointConfig& jc) {
    static rclcpp::Clock warn_clock(RCL_STEADY_TIME);
    auto clampCmd = [&](const char* field, double raw, double lo, double hi, double fallback) -> double {
        if (!std::isfinite(raw)) {
            RCLCPP_WARN_THROTTLE(logger(), warn_clock, 1000,
                                 "Non-finite %s command on '%s', using fallback %.4f",
                                 field, joint_names_[idx].c_str(), fallback);
            return std::clamp(fallback, lo, hi);
        }
        if (raw < lo || raw > hi) {
            double clamped = std::clamp(raw, lo, hi);
            RCLCPP_WARN_THROTTLE(logger(), warn_clock, 1000,
                                 "Command %s=%.4f out of bounds [%.4f, %.4f] on '%s', clamping to %.4f",
                                 field, raw, lo, hi, joint_names_[idx].c_str(), clamped);
            return clamped;
        }
        return raw;
    };

    double safe_pos = std::isfinite(pos_states_[idx]) ? pos_states_[idx] : pos_latched_[idx];
    safe_pos = std::clamp(safe_pos, jc.pos_min, jc.pos_max);

    const uint8_t mask = claimed_mask_[idx];
    // Only track measured position when actively controlled in a non-position mode (VEL/EFF);
    // in HOLD (CLAIM_NONE), keep pos_latched_ frozen so proportional stiffness holds against gravity!
    if (mask != CLAIM_NONE && (mask & CLAIM_POS) == 0) {
        pos_latched_[idx] = safe_pos;
    }

    damiao::MITParam cmd{};
    if (mask == CLAIM_NONE) {
        cmd.kp = default_kp_[idx];
        cmd.kd = default_kd_[idx];
        cmd.q = std::clamp(pos_latched_[idx], jc.pos_min, jc.pos_max);
        cmd.dq = 0.0;
        cmd.tau = 0.0;
        return cmd;
    }

    if ((mask & CLAIM_KP) || (mask & CLAIM_KD)) {
        cmd.q = (mask & CLAIM_POS) ? clampCmd("position", pos_commands_[idx], jc.pos_min, jc.pos_max, safe_pos) : safe_pos;
        cmd.dq = (mask & CLAIM_VEL) ? clampCmd("velocity", vel_commands_[idx], -jc.vel_max, jc.vel_max, 0.0) : 0.0;
        cmd.tau = (mask & CLAIM_EFF) ? clampCmd("effort", tau_commands_[idx], -jc.tau_max, jc.tau_max, 0.0) : 0.0;
        cmd.kp = (mask & CLAIM_KP) ? clampCmd("kp", kp_commands_[idx], 0.0, jc.kp_max, default_kp_[idx]) : default_kp_[idx];
        cmd.kd = (mask & CLAIM_KD) ? clampCmd("kd", kd_commands_[idx], 0.0, jc.kd_max, default_kd_[idx]) : default_kd_[idx];
        return cmd;
    }

    if (mask & CLAIM_POS) {
        cmd.q = clampCmd("position", pos_commands_[idx], jc.pos_min, jc.pos_max, pos_latched_[idx]);
        cmd.dq = (mask & CLAIM_VEL) ? clampCmd("velocity", vel_commands_[idx], -jc.vel_max, jc.vel_max, 0.0) : 0.0;
        cmd.tau = (mask & CLAIM_EFF) ? clampCmd("effort", tau_commands_[idx], -jc.tau_max, jc.tau_max, 0.0) : 0.0;
        cmd.kp = default_kp_[idx];
        cmd.kd = default_kd_[idx];
        return cmd;
    }

    if (mask == CLAIM_VEL) {
        cmd.kp = 0.0;
        cmd.kd = default_kd_[idx];
        cmd.q = safe_pos;
        cmd.dq = clampCmd("velocity", vel_commands_[idx], -jc.vel_max, jc.vel_max, 0.0);
        cmd.tau = 0.0;
        return cmd;
    }

    // CLAIM_EFF
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = safe_pos;
    cmd.dq = 0.0;
    cmd.tau = clampCmd("effort", tau_commands_[idx], -jc.tau_max, jc.tau_max, 0.0);
    return cmd;
}

hardware_interface::return_type DamiaoSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (!arm_) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        if (bridge_ && !bridge_->isRunning()) {
            RCLCPP_ERROR(logger(), "USB-CAN bridge stopped unexpectedly");
            arm_->enterFault("USB-CAN bridge stopped");
            return hardware_interface::return_type::ERROR;
        }

        const auto st = arm_->status();
        if (st == damiao::ArmStatus::ARMED) {
            if (!arm_->checkHealth()) {
                const auto arm_state = arm_->getState();
                copyArmStateToBuffers(arm_state);
                RCLCPP_ERROR(logger(), "Health check failed: %s", arm_state.fault_reason.c_str());
                return hardware_interface::return_type::ERROR;
            }
        } else if (st == damiao::ArmStatus::DISARMED) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_disarmed_refresh_ >= std::chrono::milliseconds(50)) {
                arm_->refresh(0);
                last_disarmed_refresh_ = now;
            } else {
                arm_->recv(0);
            }
        } else if (st == damiao::ArmStatus::FAULT) {
            const auto arm_state = arm_->getState();
            copyArmStateToBuffers(arm_state);
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(logger(), steady_clock, 1000,
                                  "Arm in FAULT: %s", arm_state.fault_reason.c_str());
            return hardware_interface::return_type::ERROR;
        } else {
            return hardware_interface::return_type::ERROR;
        }

        const auto arm_state = arm_->getState();
        copyArmStateToBuffers(arm_state);
        if (arm_state.status == damiao::ArmStatus::FAULT) {
            RCLCPP_ERROR(logger(), "Arm entered FAULT: %s", arm_state.fault_reason.c_str());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "read() exception: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

hardware_interface::return_type DamiaoSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (!arm_) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        if (arm_->status() != damiao::ArmStatus::ARMED) {
            return hardware_interface::return_type::ERROR;
        }

        std::array<damiao::MITParam, damiao::NUM_JOINTS> arm_cmds{};
        for (std::size_t i = 0; i < damiao::NUM_JOINTS; ++i) {
            arm_cmds[i] = buildMitCommand(i, arm_config_.joints[i]);
        }

        if (arm_config_.has_gripper) {
            const damiao::MITParam gripper_cmd = buildMitCommand(damiao::NUM_JOINTS, arm_config_.gripper);
            arm_->sendMITAll(arm_cmds, gripper_cmd);
        } else {
            arm_->sendMITAll(arm_cmds);
        }
        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "write() exception: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

} // namespace damiao_arm_hardware

PLUGINLIB_EXPORT_CLASS(
    damiao_arm_hardware::DamiaoSystemHardware,
    hardware_interface::SystemInterface)
