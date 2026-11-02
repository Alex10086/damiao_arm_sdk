#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/motor_driver.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

using namespace damiao;

namespace {

int g_passed = 0;
int g_failed = 0;

#define EXPECT_TRUE(cond, msg)                                                        \
    do {                                                                              \
        if (cond) {                                                                   \
            ++g_passed;                                                               \
        } else {                                                                      \
            ++g_failed;                                                               \
            std::cerr << "[FAIL] " << __FUNCTION__ << ":" << __LINE__ << " - " << msg \
                      << std::endl;                                                   \
        }                                                                             \
    } while (0)

#define EXPECT_NEAR(a, b, tol, msg) \
    EXPECT_TRUE(std::fabs((a) - (b)) <= (tol), msg)

#define EXPECT_THROW(stmt, exc_type, msg)                                             \
    do {                                                                              \
        bool caught = false;                                                          \
        try {                                                                         \
            stmt;                                                                     \
        } catch (const exc_type&) {                                                   \
            caught = true;                                                            \
        } catch (...) {                                                               \
        }                                                                             \
        EXPECT_TRUE(caught, msg);                                                     \
    } while (0)

// Helper to build a raw Damiao feedback frame as a motor would send it
CanFrame buildMockFeedbackFrame(uint16_t master_id,
                                uint8_t slave_id,
                                MotorError status,
                                float pos,
                                float vel,
                                float tau,
                                uint8_t t_mos,
                                uint8_t t_rotor,
                                const ProtocolLimits& limits) {
    CanFrame frame;
    frame.can_id = master_id;
    frame.dlc = 8;

    uint16_t p_int = protocol::floatToUint(pos, -limits.p_max, limits.p_max, 16);
    uint16_t v_int = protocol::floatToUint(vel, -limits.v_max, limits.v_max, 12);
    uint16_t t_int = protocol::floatToUint(tau, -limits.t_max, limits.t_max, 12);

    frame.data[0] = static_cast<uint8_t>(((static_cast<uint8_t>(status) & 0x0F) << 4) | (slave_id & 0x0F));
    frame.data[1] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(p_int & 0xFF);
    frame.data[3] = static_cast<uint8_t>((v_int >> 4) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    frame.data[5] = static_cast<uint8_t>(t_int & 0xFF);
    frame.data[6] = t_mos;
    frame.data[7] = t_rotor;
    return frame;
}

void testProtocolCodec() {
    std::cout << "[TEST] Protocol Codec (J4310 24V MIT / POS_VEL / VEL / State Cmds)..." << std::endl;
    const auto limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);

    // 1. Enable / Disable / SaveZero / ClearError frames
    auto en_frame = protocol::encodeEnableFrame(0x01);
    EXPECT_TRUE(en_frame.can_id == 0x01 && en_frame.dlc == 8, "Enable CAN ID & DLC");
    EXPECT_TRUE(en_frame.data[0] == 0xFF && en_frame.data[7] == 0xFC, "Enable payload 0xFC");

    auto dis_frame = protocol::encodeDisableFrame(0x01);
    EXPECT_TRUE(dis_frame.data[7] == 0xFD, "Disable payload 0xFD");

    auto zero_frame = protocol::encodeSaveZeroFrame(0x01);
    EXPECT_TRUE(zero_frame.data[7] == 0xFE, "SaveZero payload 0xFE");

    auto clr_frame = protocol::encodeClearErrorFrame(0x01);
    EXPECT_TRUE(clr_frame.data[7] == 0xFB, "ClearError payload 0xFB");

    // 2. MIT encoding & bit-unpacking verification
    float target_pos = 1.57f;
    float target_vel = -5.2f;
    float target_kp  = 45.0f;
    float target_kd  = 1.25f;
    float target_tau = 2.4f;

    auto mit_frame = protocol::encodeMITFrame(0x01, target_pos, target_vel, target_kp, target_kd, target_tau, limits);
    EXPECT_TRUE(mit_frame.can_id == 0x01 && mit_frame.dlc == 8, "MIT CAN ID == 0x01, DLC == 8");

    uint16_t p_int  = (static_cast<uint16_t>(mit_frame.data[0]) << 8) | mit_frame.data[1];
    uint16_t v_int  = (static_cast<uint16_t>(mit_frame.data[2]) << 4) | (mit_frame.data[3] >> 4);
    uint16_t kp_int = (static_cast<uint16_t>(mit_frame.data[3] & 0x0F) << 8) | mit_frame.data[4];
    uint16_t kd_int = (static_cast<uint16_t>(mit_frame.data[5]) << 4) | (mit_frame.data[6] >> 4);
    uint16_t t_int  = (static_cast<uint16_t>(mit_frame.data[6] & 0x0F) << 8) | mit_frame.data[7];

    EXPECT_NEAR(protocol::uintToFloat(p_int, -limits.p_max, limits.p_max, 16), target_pos, 1e-3f, "MIT pos roundtrip");
    EXPECT_NEAR(protocol::uintToFloat(v_int, -limits.v_max, limits.v_max, 12), target_vel, 2e-2f, "MIT vel roundtrip");
    EXPECT_NEAR(protocol::uintToFloat(kp_int, limits.kp_min, limits.kp_max, 12), target_kp, 0.2f, "MIT kp roundtrip");
    EXPECT_NEAR(protocol::uintToFloat(kd_int, limits.kd_min, limits.kd_max, 12), target_kd, 2e-3f, "MIT kd roundtrip");
    EXPECT_NEAR(protocol::uintToFloat(t_int, -limits.t_max, limits.t_max, 12), target_tau, 1e-2f, "MIT tau roundtrip");

    // 3. POS_VEL encoding verification (CAN ID = 0x100 + id, 2 IEEE-754 floats)
    auto pv_frame = protocol::encodePosVelFrame(0x02, -3.14159f, 8.5f, limits);
    EXPECT_TRUE(pv_frame.can_id == 0x102 && pv_frame.dlc == 8, "POS_VEL CAN ID == 0x102, DLC == 8");
    float decoded_pos = 0.0f, decoded_vel = 0.0f;
    std::memcpy(&decoded_pos, &pv_frame.data[0], sizeof(float));
    std::memcpy(&decoded_vel, &pv_frame.data[4], sizeof(float));
    EXPECT_NEAR(decoded_pos, -3.14159f, 1e-6f, "POS_VEL pos float32 exact");
    EXPECT_NEAR(decoded_vel, 8.5f, 1e-6f, "POS_VEL vel float32 exact");

    // 4. VEL encoding verification (CAN ID = 0x200 + id, 1 IEEE-754 float)
    auto vel_frame = protocol::encodeVelFrame(0x03, -12.75f, limits);
    EXPECT_TRUE(vel_frame.can_id == 0x203 && vel_frame.dlc == 4, "VEL CAN ID == 0x203, DLC == 4");
    float decoded_vonly = 0.0f;
    std::memcpy(&decoded_vonly, &vel_frame.data[0], sizeof(float));
    EXPECT_NEAR(decoded_vonly, -12.75f, 1e-6f, "VEL float32 exact");
}

void testFeedbackParsing() {
    std::cout << "[TEST] Feedback Parsing (Position / Velocity / Torque / Fault Codes / Temps)..." << std::endl;
    const auto limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);

    auto fb = buildMockFeedbackFrame(0x11, 0x01, MotorError::ENABLED, -2.5f, 6.0f, -1.8f, 34, 41, limits);
    MotorState state = protocol::decodeFeedbackFrame(fb, limits, 0);

    EXPECT_TRUE(state.valid, "Feedback state marked valid");
    EXPECT_TRUE(state.slave_id == 0x01, "Feedback slave_id == 1");
    EXPECT_TRUE(state.error == MotorError::ENABLED && state.isEnabled() && !state.hasFault(), "Motor ENABLED status");
    EXPECT_NEAR(state.position, -2.5f, 1e-3f, "Feedback position accuracy");
    EXPECT_NEAR(state.velocity, 6.0f, 2e-2f, "Feedback velocity accuracy");
    EXPECT_NEAR(state.torque, -1.8f, 1e-2f, "Feedback torque accuracy");
    EXPECT_TRUE(state.temp_mos == 34 && state.temp_rotor == 41, "Feedback temperatures");

    // Fault code parsing (e.g., 0xC = Coil Overtemperature)
    auto fault_fb = buildMockFeedbackFrame(0x11, 0x01, MotorError::COIL_OVERTEMP, 0.0f, 0.0f, 0.0f, 45, 102, limits);
    MotorState fault_state = protocol::decodeFeedbackFrame(fault_fb, limits, state.rx_count);
    EXPECT_TRUE(fault_state.hasFault(), "Fault state detected");
    EXPECT_TRUE(fault_state.error == MotorError::COIL_OVERTEMP, "Fault code COIL_OVERTEMP (0xC)");
    EXPECT_TRUE(fault_state.rx_count == 2, "RX counter incremented");
}

void testExceptionsAndEdgeCases() {
    std::cout << "[TEST] Exception Handling (Invalid Params / Comm Failures / Invalid & Stale Feedback)..." << std::endl;
    const auto limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);

    // 1. Invalid parameters (NaN / Inf / out-of-range in strict mode / bad MotorConfig)
    EXPECT_THROW(protocol::encodeMITFrame(0x01, std::nanf(""), 0.0f, 1.0f, 0.1f, 0.0f, limits),
                 std::invalid_argument, "NaN position rejected");
    EXPECT_THROW(protocol::encodeMITFrame(0x01, 20.0f, 0.0f, 1.0f, 0.1f, 0.0f, limits, true),
                 std::out_of_range, "Out-of-range pos rejected in strict mode");
    EXPECT_THROW(protocol::encodePosVelFrame(0x01, 0.0f, 50.0f, limits, true),
                 std::out_of_range, "Out-of-range vel rejected in strict mode");

    MotorConfig bad_cfg = MotorConfig::J4310(0x00, 0x11);
    EXPECT_THROW(bad_cfg.validate(), std::invalid_argument, "can_id == 0 rejected");

    // 2. Communication failures (unopened socket, non-existent interface)
    SocketCanTransport unopened_can;
    CanFrame dummy;
    EXPECT_THROW(unopened_can.send(dummy), CanException, "Send on closed transport throws CanException");
    EXPECT_THROW(unopened_can.receive(dummy, 5), CanException, "Receive on closed transport throws CanException");
    EXPECT_THROW(unopened_can.open("nonexistent_can99"), CanException, "Open invalid interface throws CanException");

    // 3. Invalid feedback frame (wrong DLC, RTR, error flag)
    CanFrame short_frame;
    short_frame.can_id = 0x11;
    short_frame.dlc = 5;
    EXPECT_THROW(protocol::decodeFeedbackFrame(short_frame, limits),
                 std::invalid_argument, "DLC != 8 feedback rejected");

    CanFrame err_frame;
    err_frame.can_id = 0x11;
    err_frame.dlc = 8;
    err_frame.is_error = true;
    EXPECT_THROW(protocol::decodeFeedbackFrame(err_frame, limits),
                 std::invalid_argument, "Error flag frame rejected");

    // 4. Stale / missing feedback detection in MotorDriver
    auto transport = std::make_shared<SocketCanTransport>();
    MotorConfig cfg = MotorConfig::J4310(0x01, 0x11);
    cfg.state_timeout_ms = 30;
    MotorDriver driver(transport, cfg);

    // Before any feedback arrives, check_staleness=true must throw
    EXPECT_THROW(driver.getState(true), CanException, "getState(true) throws when no feedback received yet");

    auto fb = buildMockFeedbackFrame(0x11, 0x01, MotorError::ENABLED, 1.0f, 0.0f, 0.0f, 28, 30, cfg.limits);
    EXPECT_TRUE(driver.processFeedbackFrame(fb), "Valid feedback accepted");
    MotorState fresh = driver.getState(true);
    EXPECT_TRUE(fresh.valid && !fresh.isStale(std::chrono::milliseconds(30)), "Feedback is fresh immediately");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_THROW(driver.getState(true), CanException, "getState(true) throws after feedback expires");
}

void testVcan0Communication() {
    std::cout << "[TEST] SocketCAN vcan0 Communication & End-to-End MotorDriver Loop..." << std::endl;

    auto tx_transport = std::make_shared<SocketCanTransport>("vcan0", true);
    auto rx_transport = std::make_shared<SocketCanTransport>("vcan0", true);

    EXPECT_TRUE(tx_transport->isOpen() && rx_transport->isOpen(), "Both vcan0 endpoints opened");

    MotorConfig cfg = MotorConfig::J4310(0x01, 0x11, ControlMode::MIT);
    MotorDriver driver(tx_transport, cfg);

    // 1. Driver sends Enable -> RX endpoint receives on vcan0
    driver.enable();
    CanFrame wire_frame;
    bool got_en = rx_transport->receive(wire_frame, 100);
    EXPECT_TRUE(got_en && wire_frame.can_id == 0x01 && wire_frame.data[7] == 0xFC,
                "vcan0 received Enable frame from MotorDriver");

    // 2. Driver sends MIT command -> RX endpoint receives on vcan0
    driver.setMIT(0.8f, 1.5f, 25.0f, 0.8f, 0.5f);
    bool got_mit = rx_transport->receive(wire_frame, 100);
    EXPECT_TRUE(got_mit && wire_frame.can_id == 0x01 && wire_frame.dlc == 8,
                "vcan0 received MIT frame from MotorDriver");

    // 3. Simulated motor on rx_transport sends feedback -> MotorDriver polls and updates state
    CanFrame mock_fb = buildMockFeedbackFrame(
        0x11, 0x01, MotorError::ENABLED, 0.79f, 1.48f, 0.49f, 31, 33, cfg.limits);
    rx_transport->send(mock_fb);

    bool updated = driver.pollFeedback(100);
    EXPECT_TRUE(updated, "MotorDriver::pollFeedback received simulated motor response over vcan0");

    MotorState st = driver.getState(true);
    EXPECT_TRUE(st.isEnabled(), "Motor state is ENABLED after vcan0 feedback");
    EXPECT_NEAR(st.position, 0.79f, 1e-3f, "vcan0 feedback position matches");
    EXPECT_NEAR(st.velocity, 1.48f, 2e-2f, "vcan0 feedback velocity matches");
    EXPECT_NEAR(st.torque, 0.49f, 1e-2f, "vcan0 feedback torque matches");
    EXPECT_TRUE(st.temp_mos == 31 && st.temp_rotor == 33, "vcan0 feedback temperatures match");
}

} // namespace

int main() {
    testProtocolCodec();
    testFeedbackParsing();
    testExceptionsAndEdgeCases();
    testVcan0Communication();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
