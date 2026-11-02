#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace damiao {

enum class MotorModel {
    DM_J4310_2EC, // 24V J4310 (default: P_MAX=12.5, V_MAX=30.0, T_MAX=10.0)
    DM_J4340_2EC, // 24V J4340 (P_MAX=12.5, V_MAX=10.0, T_MAX=28.0)
    DM_J8009,     // P_MAX=12.5, V_MAX=45.0, T_MAX=54.0
    CUSTOM
};

enum class ControlMode : uint32_t {
    MIT     = 0x000, // CAN ID = can_id
    POS_VEL = 0x100, // CAN ID = 0x100 + can_id
    VEL     = 0x200  // CAN ID = 0x200 + can_id
};

// Damiao 4-bit status/error code in feedback byte[0] high nibble
enum class MotorError : uint8_t {
    DISABLED            = 0x0, // 失能
    ENABLED             = 0x1, // 使能正常
    OVERVOLTAGE         = 0x8, // 超压
    UNDERVOLTAGE        = 0x9, // 欠压
    OVERCURRENT         = 0xA, // 过流
    MOS_OVERTEMP        = 0xB, // MOS 过温
    COIL_OVERTEMP       = 0xC, // 电机线圈过温
    COMM_LOST           = 0xD, // 通讯丢失
    OVERLOAD            = 0xE, // 过载
    UNKNOWN             = 0xF
};

inline const char* toString(MotorError err) noexcept {
    switch (err) {
        case MotorError::DISABLED:      return "DISABLED (0x0)";
        case MotorError::ENABLED:       return "ENABLED (0x1)";
        case MotorError::OVERVOLTAGE:   return "OVERVOLTAGE (0x8)";
        case MotorError::UNDERVOLTAGE:  return "UNDERVOLTAGE (0x9)";
        case MotorError::OVERCURRENT:   return "OVERCURRENT (0xA)";
        case MotorError::MOS_OVERTEMP:  return "MOS_OVERTEMP (0xB)";
        case MotorError::COIL_OVERTEMP: return "COIL_OVERTEMP (0xC)";
        case MotorError::COMM_LOST:     return "COMM_LOST (0xD)";
        case MotorError::OVERLOAD:      return "OVERLOAD (0xE)";
        default:                        return "UNKNOWN";
    }
}

inline MotorError parseMotorError(uint8_t raw_nibble) noexcept {
    switch (raw_nibble & 0x0F) {
        case 0x0: return MotorError::DISABLED;
        case 0x1: return MotorError::ENABLED;
        case 0x8: return MotorError::OVERVOLTAGE;
        case 0x9: return MotorError::UNDERVOLTAGE;
        case 0xA: return MotorError::OVERCURRENT;
        case 0xB: return MotorError::MOS_OVERTEMP;
        case 0xC: return MotorError::COIL_OVERTEMP;
        case 0xD: return MotorError::COMM_LOST;
        case 0xE: return MotorError::OVERLOAD;
        default:  return MotorError::UNKNOWN;
    }
}

// ProtocolLimits: Full-scale mapping range configured in motor registers (PMAX, VMAX, TMAX).
// NOTE: This is the protocol quantization mapping span (used for float <-> uint16/uint12 conversion
// in MIT mode and feedback decoding), NOT the physical joint mechanical travel limit!
struct ProtocolLimits {
    float p_max{12.5f};   // rad   (protocol mapping span [-p_max, +p_max])
    float v_max{30.0f};   // rad/s (J4310 24V default)
    float t_max{10.0f};   // N*m   (J4310 24V default)
    float kp_min{0.0f};
    float kp_max{500.0f};
    float kd_min{0.0f};
    float kd_max{5.0f};

    static constexpr ProtocolLimits forModel(MotorModel model) noexcept {
        switch (model) {
            case MotorModel::DM_J4310_2EC:
                return ProtocolLimits{12.5f, 30.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f};
            case MotorModel::DM_J4340_2EC:
                return ProtocolLimits{12.5f, 10.0f, 28.0f, 0.0f, 500.0f, 0.0f, 5.0f};
            case MotorModel::DM_J8009:
                return ProtocolLimits{12.5f, 45.0f, 54.0f, 0.0f, 500.0f, 0.0f, 5.0f};
            case MotorModel::CUSTOM:
            default:
                return ProtocolLimits{12.5f, 30.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f};
        }
    }

    bool isValid() const noexcept {
        return std::isfinite(p_max) && p_max > 0.0f &&
               std::isfinite(v_max) && v_max > 0.0f &&
               std::isfinite(t_max) && t_max > 0.0f &&
               std::isfinite(kp_min) && std::isfinite(kp_max) && kp_max > kp_min &&
               std::isfinite(kd_min) && std::isfinite(kd_max) && kd_max > kd_min;
    }
};

// CommandLimits: Optional user/mechanical safety limits (distinct from ProtocolLimits).
// Physical joint position limits (e.g., [-3.14, 3.14] rad for a joint or [0, 0.8] rad for a gripper)
// or POS_VEL multi-turn limits can be enforced independently of the protocol quantization span.
struct CommandLimits {
    float pos_min{-12.5f}; // rad
    float pos_max{12.5f};  // rad
    float vel_max{30.0f};  // rad/s
    float tau_max{10.0f};  // N*m

    static constexpr CommandLimits fromProtocol(const ProtocolLimits& p) noexcept {
        return CommandLimits{-p.p_max, p.p_max, p.v_max, p.t_max};
    }

    bool isValid() const noexcept {
        return std::isfinite(pos_min) && std::isfinite(pos_max) && pos_max >= pos_min &&
               std::isfinite(vel_max) && vel_max > 0.0f &&
               std::isfinite(tau_max) && tau_max > 0.0f;
    }
};

struct MotorConfig {
    MotorModel model{MotorModel::DM_J4310_2EC};
    uint16_t can_id{0x01};      // Slave CAN ID (1 ~ 0xFF)
    uint16_t master_id{0x11};   // Master / feedback CAN ID (0x00 ~ 0x7FF, recommend can_id + 0x10)
    ControlMode mode{ControlMode::MIT};
    ProtocolLimits limits{ProtocolLimits::forModel(MotorModel::DM_J4310_2EC)};
    CommandLimits cmd_limits{CommandLimits::fromProtocol(ProtocolLimits::forModel(MotorModel::DM_J4310_2EC))};
    uint32_t state_timeout_ms{500}; // Feedback staleness threshold in ms
    bool strict_range_check{true};  // P1: Default true (throw std::out_of_range on invalid command range)
    bool special_cmd_use_mode_offset{true}; // P0: true = mode_offset + can_id; false = raw can_id (new firmware)

    static MotorConfig J4310(uint16_t can_id,
                             uint16_t master_id,
                             ControlMode mode = ControlMode::MIT) {
        MotorConfig cfg;
        cfg.model = MotorModel::DM_J4310_2EC;
        cfg.can_id = can_id;
        cfg.master_id = master_id;
        cfg.mode = mode;
        cfg.limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);
        cfg.cmd_limits = CommandLimits::fromProtocol(cfg.limits);
        cfg.strict_range_check = true;
        return cfg;
    }

    void validate() const {
        if (can_id == 0 || can_id > 0x0FF) {
            throw std::invalid_argument("MotorConfig: can_id must be in range [0x01, 0xFF]");
        }
        if (master_id > 0x7FF) {
            throw std::invalid_argument("MotorConfig: master_id must be a valid 11-bit CAN ID (<= 0x7FF)");
        }
        const uint32_t tx_id = static_cast<uint32_t>(mode) + can_id;
        if (tx_id > 0x7FF) {
            throw std::invalid_argument("MotorConfig: computed transmit CAN ID exceeds 11-bit standard CAN limit");
        }
        if (!limits.isValid()) {
            throw std::invalid_argument("MotorConfig: invalid protocol limits");
        }
        if (!cmd_limits.isValid()) {
            throw std::invalid_argument("MotorConfig: invalid command limits");
        }
    }
};

struct MotorState {
    uint16_t slave_id{0};           // Full slave ID (resolved from config if can_id > 0x0F, or 8-bit byte[0] if master_id != 0)
    uint8_t raw_status{0};          // 4-bit status from feedback byte[0] high nibble
    MotorError error{MotorError::DISABLED};
    float position{0.0f};           // rad
    float velocity{0.0f};           // rad/s
    float torque{0.0f};             // N*m
    uint8_t temp_mos{0};            // degC
    uint8_t temp_rotor{0};          // degC
    std::chrono::steady_clock::time_point timestamp{};
    uint64_t rx_count{0};
    bool valid{false};

    bool hasFault() const noexcept {
        return raw_status >= 0x8;
    }

    bool isEnabled() const noexcept {
        return error == MotorError::ENABLED;
    }

    bool isStale(std::chrono::milliseconds max_age,
                 std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) const noexcept {
        if (!valid) {
            return true;
        }
        return (now - timestamp) > max_age;
    }
};

} // namespace damiao
