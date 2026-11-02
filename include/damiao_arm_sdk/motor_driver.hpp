#pragma once

#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <memory>
#include <mutex>

namespace damiao {

// Pure Damiao protocol codec utilities (stateless, directly unit-testable)
namespace protocol {

uint16_t floatToUint(float x, float x_min, float x_max, int bits);
float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits);

CanFrame encodeEnableFrame(uint32_t tx_can_id);
CanFrame encodeDisableFrame(uint32_t tx_can_id);
CanFrame encodeSaveZeroFrame(uint32_t tx_can_id);
CanFrame encodeClearErrorFrame(uint32_t tx_can_id);

CanFrame encodeMITFrame(uint16_t can_id,
                        float pos,
                        float vel,
                        float kp,
                        float kd,
                        float tau,
                        const ProtocolLimits& limits,
                        bool strict_range_check = false);

CanFrame encodePosVelFrame(uint16_t can_id,
                           float pos,
                           float vel,
                           const ProtocolLimits& limits,
                           bool strict_range_check = false);

CanFrame encodeVelFrame(uint16_t can_id,
                        float vel,
                        const ProtocolLimits& limits,
                        bool strict_range_check = false);

// Decodes an 8-byte Damiao motor feedback frame into state.
// Throws std::invalid_argument if frame DLC != 8 or frame is error/RTR.
MotorState decodeFeedbackFrame(const CanFrame& frame,
                               const ProtocolLimits& limits,
                               uint64_t prev_rx_count = 0,
                               std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now());

} // namespace protocol

class MotorDriver {
public:
    MotorDriver(std::shared_ptr<ICanTransport> transport, const MotorConfig& config);

    // State commands
    void enable();
    void disable();
    void saveZeroPosition();
    void clearError();

    // Control mode commands
    void setMIT(float pos, float vel, float kp, float kd, float tau);
    void setPosVel(float pos, float vel);
    void setVelocity(float vel);

    // Process an incoming CAN frame if it matches this motor's master_id / slave_id.
    // Returns true if the frame belonged to this motor and updated state.
    bool processFeedbackFrame(const CanFrame& frame);

    // Poll CAN transport once and update state if a matching feedback frame arrives.
    bool pollFeedback(int timeout_ms = 5);

    // Thread-safe state access
    MotorState getState(bool check_staleness = false) const;

    const MotorConfig& config() const noexcept { return config_; }
    uint32_t activeTxCanId() const noexcept;

private:
    std::shared_ptr<ICanTransport> transport_;
    MotorConfig config_;
    mutable std::mutex state_mutex_;
    MotorState state_{};
};

} // namespace damiao
