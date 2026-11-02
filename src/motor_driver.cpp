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
    if (tx_can_id > 0x7FF) {
        throw std::invalid_argument("Damiao protocol: tx_can_id exceeds 11-bit standard CAN ID");
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
    return static_cast<uint16_t>( scaled + 0.5f );
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

    float p = checkOrClamp(pos, -limits.p_max, limits.p_max, strict_range_check, "pos");
    float v = checkOrClamp(vel, -limits.v_max, limits.v_max, strict_range_check, "vel");

    CanFrame frame;
    frame.can_id = static_cast<uint32_t>(ControlMode::POS_VEL) + can_id;
    frame.dlc = 8;
    // IEEE-754 little-endian float packing (Bytes 0..3: pos, Bytes 4..7: vel)
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

void MotorDriver::enable() {
    transport_->send(protocol::encodeEnableFrame(activeTxCanId()));
}

void MotorDriver::disable() {
    transport_->send(protocol::encodeDisableFrame(activeTxCanId()));
}

void MotorDriver::saveZeroPosition() {
    transport_->send(protocol::encodeSaveZeroFrame(activeTxCanId()));
}

void MotorDriver::clearError() {
    transport_->send(protocol::encodeClearErrorFrame(activeTxCanId()));
}

void MotorDriver::setMIT(float pos, float vel, float kp, float kd, float tau) {
    CanFrame frame = protocol::encodeMITFrame(
        config_.can_id, pos, vel, kp, kd, tau, config_.limits, config_.strict_range_check);
    transport_->send(frame);
}

void MotorDriver::setPosVel(float pos, float vel) {
    CanFrame frame = protocol::encodePosVelFrame(
        config_.can_id, pos, vel, config_.limits, config_.strict_range_check);
    transport_->send(frame);
}

void MotorDriver::setVelocity(float vel) {
    CanFrame frame = protocol::encodeVelFrame(
        config_.can_id, vel, config_.limits, config_.strict_range_check);
    transport_->send(frame);
}

bool MotorDriver::processFeedbackFrame(const CanFrame& frame) {
    // Match either by master_id or, when master_id == 0 (shared master ID), by low nibble of byte[0]
    if (frame.can_id != config_.master_id) {
        return false;
    }
    if (frame.dlc != 8 || frame.is_error || frame.is_rtr || frame.is_extended) {
        throw CanException("MotorDriver::processFeedbackFrame: malformed feedback frame on master_id");
    }
    uint8_t frame_slave_id = frame.data[0] & 0x0F;
    if (frame_slave_id != (config_.can_id & 0x0F)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = protocol::decodeFeedbackFrame(frame, config_.limits, state_.rx_count);
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
