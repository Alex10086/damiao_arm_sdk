#include "damiao_arm_sdk/motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace damiao {

namespace protocol {

namespace {

void validateFinite(float val, const char* name) {
    if (!std::isfinite(val)) {
        throw std::invalid_argument(std::string("Damiao protocol: parameter '") + name + "' must be finite (not NaN/Inf)");
    }
}

float checkOrClamp(float val, float min_val, float max_val, bool strict, const char* name) {
    validateFinite(val, name);
    if (val < min_val || val > max_val) {
        if (strict) {
            throw std::out_of_range(std::string("Damiao protocol: parameter '") + name + "' out of range");
        }
        return std::clamp(val, min_val, max_val);
    }
    return val;
}

CanFrame makeSpecialCommandFrame(uint32_t tx_can_id, uint8_t cmd_byte) {
    if (tx_can_id == 0 || tx_can_id > 0x7FF) {
        throw std::invalid_argument("Damiao protocol: tx_can_id must be in [0x001, 0x7FF]");
    }
    CanFrame frame;
    frame.can_id = tx_can_id;
    frame.dlc = 8;
    frame.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte};
    return frame;
}

} // namespace

uint16_t floatToUint(float x, float x_min, float x_max, int bits) {
    if (bits <= 0 || bits > 16) {
        throw std::invalid_argument("floatToUint: bits must be in [1, 16]");
    }
    if (!(x_max > x_min)) {
        throw std::invalid_argument("floatToUint: x_max must be greater than x_min");
    }
    float clamped = std::clamp(x, x_min, x_max);
    float span = x_max - x_min;
    float offset = clamped - x_min;
    uint32_t max_int = (1u << bits) - 1u;
    float scaled = (offset * static_cast<float>(max_int)) / span;
    return static_cast<uint16_t>(scaled + 0.5f);
}

float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits) {
    if (bits <= 0 || bits > 16) {
        throw std::invalid_argument("uintToFloat: bits must be in [1, 16]");
    }
    if (!(x_max > x_min)) {
        throw std::invalid_argument("uintToFloat: x_max must be greater than x_min");
    }
    uint32_t max_int = (1u << bits) - 1u;
    uint32_t clamped_int = std::min<uint32_t>(x_int, max_int);
    float span = x_max - x_min;
    return static_cast<float>(clamped_int) * span / static_cast<float>(max_int) + x_min;
}

CanFrame encodeEnableFrame(uint32_t tx_can_id) {
    return makeSpecialCommandFrame(tx_can_id, 0xFC);
}

CanFrame encodeDisableFrame(uint32_t tx_can_id) {
    return makeSpecialCommandFrame(tx_can_id, 0xFD);
}

CanFrame encodeSaveZeroFrame(uint32_t tx_can_id) {
    return makeSpecialCommandFrame(tx_can_id, 0xFE);
}

CanFrame encodeClearErrorFrame(uint32_t tx_can_id) {
    return makeSpecialCommandFrame(tx_can_id, 0xFB);
}

CanFrame encodeMITFrame(uint16_t can_id,
                        float pos,
                        float vel,
                        float kp,
                        float kd,
                        float tau,
                        const ProtocolLimits& limits,
                        bool strict_range_check) {
    if (can_id == 0 || can_id > 0x7FF) {
        throw std::invalid_argument("encodeMITFrame: invalid can_id");
    }
    if (!limits.isValid()) {
        throw std::invalid_argument("encodeMITFrame: invalid ProtocolLimits");
    }

    // MIT mode maps pos/vel/tau into fixed-point integers using ProtocolLimits [p_max, v_max, t_max]
    float p = checkOrClamp(pos, -limits.p_max, limits.p_max, strict_range_check, "pos");
    float v = checkOrClamp(vel, -limits.v_max, limits.v_max, strict_range_check, "vel");
    float kp_c = checkOrClamp(kp, limits.kp_min, limits.kp_max, strict_range_check, "kp");
    float kd_c = checkOrClamp(kd, limits.kd_min, limits.kd_max, strict_range_check, "kd");
    float t = checkOrClamp(tau, -limits.t_max, limits.t_max, strict_range_check, "tau");

    uint16_t p_int  = floatToUint(p, -limits.p_max, limits.p_max, 16);
    uint16_t v_int  = floatToUint(v, -limits.v_max, limits.v_max, 12);
    uint16_t kp_int = floatToUint(kp_c, limits.kp_min, limits.kp_max, 12);
    uint16_t kd_int = floatToUint(kd_c, limits.kd_min, limits.kd_max, 12);
    uint16_t t_int  = floatToUint(t, -limits.t_max, limits.t_max, 12);

    CanFrame frame;
    frame.can_id = static_cast<uint32_t>(ControlMode::MIT) + can_id;
    frame.dlc = 8;
    frame.data[0] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(p_int & 0xFF);
    frame.data[2] = static_cast<uint8_t>((v_int >> 4) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    frame.data[4] = static_cast<uint8_t>(kp_int & 0xFF);
    frame.data[5] = static_cast<uint8_t>((kd_int >> 4) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    frame.data[7] = static_cast<uint8_t>(t_int & 0xFF);
    return frame;
}

CanFrame encodePosVelFrame(uint16_t can_id,
                           float pos,
                           float vel,
                           const ProtocolLimits& limits,
                           bool strict_range_check) {
    if (can_id == 0 || (static_cast<uint32_t>(ControlMode::POS_VEL) + can_id) > 0x7FF) {
        throw std::invalid_argument("encodePosVelFrame: invalid can_id");
    }
    if (!limits.isValid()) {
        throw std::invalid_argument("encodePosVelFrame: invalid ProtocolLimits");
    }

    // POS_VEL transmits raw IEEE-754 float32 (not quantized by limits.p_max),
    // but validates finite values and optional range check against limits if requested.
    validateFinite(pos, "pos");
    float p = strict_range_check ? checkOrClamp(pos, -limits.p_max, limits.p_max, true, "pos") : pos;
    float v = checkOrClamp(vel, -limits.v_max, limits.v_max, strict_range_check, "vel");

    CanFrame frame;
    frame.can_id = static_cast<uint32_t>(ControlMode::POS_VEL) + can_id;
    frame.dlc = 8;
    std::memcpy(&frame.data[0], &p, sizeof(float));
    std::memcpy(&frame.data[4], &v, sizeof(float));
    return frame;
}

CanFrame encodeVelFrame(uint16_t can_id,
                        float vel,
                        const ProtocolLimits& limits,
                        bool strict_range_check) {
    if (can_id == 0 || (static_cast<uint32_t>(ControlMode::VEL) + can_id) > 0x7FF) {
        throw std::invalid_argument("encodeVelFrame: invalid can_id");
    }
    if (!limits.isValid()) {
        throw std::invalid_argument("encodeVelFrame: invalid ProtocolLimits");
    }

    float v = checkOrClamp(vel, -limits.v_max, limits.v_max, strict_range_check, "vel");

    CanFrame frame;
    frame.can_id = static_cast<uint32_t>(ControlMode::VEL) + can_id;
    frame.dlc = 4;
    frame.data.fill(0);
    std::memcpy(&frame.data[0], &v, sizeof(float));
    return frame;
}

MotorState decodeFeedbackFrame(const CanFrame& frame,
                               const ProtocolLimits& limits,
                               uint64_t prev_rx_count,
                               std::chrono::steady_clock::time_point timestamp) {
    if (frame.is_error || frame.is_rtr || frame.is_extended) {
        throw std::invalid_argument("decodeFeedbackFrame: invalid CAN frame flags (error/RTR/extended)");
    }
    if (frame.dlc != 8) {
        throw std::invalid_argument("decodeFeedbackFrame: Damiao feedback frame must have DLC == 8");
    }
    if (!limits.isValid()) {
        throw std::invalid_argument("decodeFeedbackFrame: invalid ProtocolLimits");
    }

    uint16_t p_int = (static_cast<uint16_t>(frame.data[1]) << 8) |
                     static_cast<uint16_t>(frame.data[2]);
    uint16_t v_int = (static_cast<uint16_t>(frame.data[3]) << 4) |
                     (static_cast<uint16_t>(frame.data[4]) >> 4);
    uint16_t t_int = (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) |
                     static_cast<uint16_t>(frame.data[5]);

    MotorState state;
    state.slave_id = frame.data[0] & 0x0F;
    state.raw_status = (frame.data[0] >> 4) & 0x0F;
    state.error = parseMotorError(state.raw_status);
    state.position = uintToFloat(p_int, -limits.p_max, limits.p_max, 16);
    state.velocity = uintToFloat(v_int, -limits.v_max, limits.v_max, 12);
    state.torque = uintToFloat(t_int, -limits.t_max, limits.t_max, 12);
    state.temp_mos = frame.data[6];
    state.temp_rotor = frame.data[7];
    state.timestamp = timestamp;
    state.rx_count = prev_rx_count + 1;
    state.valid = true;
    return state;
}

} // namespace protocol

MotorDriver::MotorDriver(std::shared_ptr<ICanTransport> transport, const MotorConfig& config)
    : transport_(std::move(transport)), config_(config) {
    if (!transport_) {
        throw std::invalid_argument("MotorDriver: transport pointer cannot be null");
    }
    config_.validate();
}

uint32_t MotorDriver::activeTxCanId() const noexcept {
    return static_cast<uint32_t>(config_.mode) + config_.can_id;
}

uint32_t MotorDriver::specialCommandCanId() const noexcept {
    return config_.special_cmd_use_mode_offset ? activeTxCanId() : static_cast<uint32_t>(config_.can_id);
}

void MotorDriver::enable() {
    transport_->send(protocol::encodeEnableFrame(specialCommandCanId()));
}

void MotorDriver::disable() {
    transport_->send(protocol::encodeDisableFrame(specialCommandCanId()));
}

void MotorDriver::saveZeroPosition() {
    transport_->send(protocol::encodeSaveZeroFrame(specialCommandCanId()));
}

void MotorDriver::clearError() {
    transport_->send(protocol::encodeClearErrorFrame(specialCommandCanId()));
}

void MotorDriver::setMIT(float pos, float vel, float kp, float kd, float tau) {
    if (config_.mode != ControlMode::MIT) {
        throw std::logic_error("MotorDriver::setMIT: control mode mismatch (configured mode is not MIT)");
    }
    if (!std::isfinite(pos) || !std::isfinite(vel) || !std::isfinite(tau)) {
        throw std::invalid_argument("MotorDriver::setMIT: non-finite command argument");
    }
    if (pos < config_.cmd_limits.pos_min || pos > config_.cmd_limits.pos_max ||
        std::fabs(vel) > config_.cmd_limits.vel_max ||
        std::fabs(tau) > config_.cmd_limits.tau_max) {
        if (config_.strict_range_check) {
            throw std::out_of_range("MotorDriver::setMIT: command exceeds configured CommandLimits");
        }
        pos = std::clamp(pos, config_.cmd_limits.pos_min, config_.cmd_limits.pos_max);
        vel = std::clamp(vel, -config_.cmd_limits.vel_max, config_.cmd_limits.vel_max);
        tau = std::clamp(tau, -config_.cmd_limits.tau_max, config_.cmd_limits.tau_max);
    }

    CanFrame frame = protocol::encodeMITFrame(
        config_.can_id, pos, vel, kp, kd, tau, config_.limits, config_.strict_range_check);
    transport_->send(frame);
}

void MotorDriver::setPosVel(float pos, float vel) {
    if (config_.mode != ControlMode::POS_VEL) {
        throw std::logic_error("MotorDriver::setPosVel: control mode mismatch (configured mode is not POS_VEL)");
    }
    if (!std::isfinite(pos) || !std::isfinite(vel)) {
        throw std::invalid_argument("MotorDriver::setPosVel: non-finite command argument");
    }
    if (pos < config_.cmd_limits.pos_min || pos > config_.cmd_limits.pos_max ||
        std::fabs(vel) > config_.cmd_limits.vel_max) {
        if (config_.strict_range_check) {
            throw std::out_of_range("MotorDriver::setPosVel: command exceeds configured CommandLimits");
        }
        pos = std::clamp(pos, config_.cmd_limits.pos_min, config_.cmd_limits.pos_max);
        vel = std::clamp(vel, -config_.cmd_limits.vel_max, config_.cmd_limits.vel_max);
    }

    // Note: pass strict_range_check=false to protocol::encodePosVelFrame if cmd_limits already validated
    // multi-turn pos beyond protocol limits.p_max, while still enforcing finite check.
    CanFrame frame = protocol::encodePosVelFrame(
        config_.can_id, pos, vel, config_.limits, false);
    transport_->send(frame);
}

void MotorDriver::setVelocity(float vel) {
    if (config_.mode != ControlMode::VEL) {
        throw std::logic_error("MotorDriver::setVelocity: control mode mismatch (configured mode is not VEL)");
    }
    if (!std::isfinite(vel)) {
        throw std::invalid_argument("MotorDriver::setVelocity: non-finite command argument");
    }
    if (std::fabs(vel) > config_.cmd_limits.vel_max) {
        if (config_.strict_range_check) {
            throw std::out_of_range("MotorDriver::setVelocity: command exceeds configured CommandLimits");
        }
        vel = std::clamp(vel, -config_.cmd_limits.vel_max, config_.cmd_limits.vel_max);
    }

    CanFrame frame = protocol::encodeVelFrame(
        config_.can_id, vel, config_.limits, false);
    transport_->send(frame);
}

bool MotorDriver::processFeedbackFrame(const CanFrame& frame) {
    if (frame.can_id != config_.master_id) {
        return false;
    }
    if (frame.dlc != 8 || frame.is_error || frame.is_rtr || frame.is_extended) {
        throw CanException("MotorDriver::processFeedbackFrame: malformed feedback frame on master_id");
    }

    // P1 Fix: byte[0] low nibble only holds 4 bits (0..15).
    // - When master_id != 0 (recommended unique master_id per motor, e.g. 0x11 for 0x01, 0x21 for 0x11),
    //   frame.can_id == config_.master_id uniquely identifies the motor, and low nibble matches (can_id & 0x0F).
    // - If master_id == 0 (shared master ID), only can_id <= 0x0F can be disambiguated via the 4-bit nibble.
    uint8_t frame_nibble = frame.data[0] & 0x0F;
    if (config_.master_id == 0) {
        if (config_.can_id > 0x0F || frame_nibble != config_.can_id) {
            return false;
        }
    } else if (config_.can_id <= 0x0F) {
        if (frame_nibble != config_.can_id) {
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = protocol::decodeFeedbackFrame(frame, config_.limits, state_.rx_count);
    state_.slave_id = config_.can_id;
    return true;
}

bool MotorDriver::pollFeedback(int timeout_ms) {
    CanFrame frame;
    if (!transport_->receive(frame, timeout_ms)) {
        return false;
    }
    return processFeedbackFrame(frame);
}

MotorState MotorDriver::getState(bool check_staleness) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (check_staleness) {
        if (!state_.valid) {
            throw CanException("MotorDriver::getState: no valid feedback received yet");
        }
        if (state_.isStale(std::chrono::milliseconds(config_.state_timeout_ms))) {
            throw CanException("MotorDriver::getState: motor feedback is stale (timeout exceeded)");
        }
    }
    return state_;
}

} // namespace damiao
