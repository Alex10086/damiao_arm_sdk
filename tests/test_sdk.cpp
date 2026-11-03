#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/damiao_usb_transport.hpp"
#include "damiao_arm_sdk/motor_driver.hpp"
#include "damiao_arm_sdk/socketcan_bridge.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <pty.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

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

CanFrame buildMockFeedbackFrame(uint16_t master_id,
                                uint8_t slave_id_nibble,
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

    frame.data[0] = static_cast<uint8_t>(((static_cast<uint8_t>(status) & 0x0F) << 4) | (slave_id_nibble & 0x0F));
    frame.data[1] = static_cast<uint16_t>(p_int >> 8) & 0xFF;
    frame.data[2] = static_cast<uint16_t>(p_int) & 0xFF;
    frame.data[3] = static_cast<uint16_t>(v_int >> 4) & 0xFF;
    frame.data[4] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    frame.data[5] = static_cast<uint8_t>(t_int & 0xFF);
    frame.data[6] = t_mos;
    frame.data[7] = t_rotor;
    return frame;
}

std::vector<uint8_t> buildUsbRxPacket(uint8_t cmd,
                                      uint32_t can_id,
                                      uint8_t dlc,
                                      bool is_ext,
                                      bool is_rtr,
                                      const std::array<uint8_t, 8>& data) {
    std::vector<uint8_t> pkt(16, 0);
    pkt[0] = 0xAA;
    pkt[1] = cmd;
    pkt[2] = (dlc & 0x3F) | (is_ext ? 0x40 : 0x00) | (is_rtr ? 0x80 : 0x00);
    std::memcpy(&pkt[3], &can_id, 4);
    std::memcpy(&pkt[7], data.data(), 8);
    pkt[15] = 0x55;
    return pkt;
}

void testProtocolCodec() {
    std::cout << "[TEST] Protocol Codec (J4310 24V MIT / POS_VEL / VEL / State Cmds)..." << std::endl;
    const auto limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);

    auto en_frame = protocol::encodeEnableFrame(0x01);
    EXPECT_TRUE(en_frame.can_id == 0x01 && en_frame.dlc == 8, "Enable CAN ID & DLC");
    EXPECT_TRUE(en_frame.data[0] == 0xFF && en_frame.data[7] == 0xFC, "Enable payload 0xFC");

    auto dis_frame = protocol::encodeDisableFrame(0x01);
    EXPECT_TRUE(dis_frame.data[7] == 0xFD, "Disable payload 0xFD");

    auto zero_frame = protocol::encodeSaveZeroFrame(0x01);
    EXPECT_TRUE(zero_frame.data[7] == 0xFE, "SaveZero payload 0xFE");

    auto clr_frame = protocol::encodeClearErrorFrame(0x01);
    EXPECT_TRUE(clr_frame.data[7] == 0xFB, "ClearError payload 0xFB");

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

    auto pv_frame = protocol::encodePosVelFrame(0x02, -3.14159f, 8.5f, limits);
    EXPECT_TRUE(pv_frame.can_id == 0x102 && pv_frame.dlc == 8, "POS_VEL CAN ID == 0x102, DLC == 8");
    float decoded_pos = 0.0f, decoded_vel = 0.0f;
    std::memcpy(&decoded_pos, &pv_frame.data[0], sizeof(float));
    std::memcpy(&decoded_vel, &pv_frame.data[4], sizeof(float));
    EXPECT_NEAR(decoded_pos, -3.14159f, 1e-6f, "POS_VEL pos float32 exact");
    EXPECT_NEAR(decoded_vel, 8.5f, 1e-6f, "POS_VEL vel float32 exact");

    auto vel_frame = protocol::encodeVelFrame(0x03, -12.75f, limits);
    EXPECT_TRUE(vel_frame.can_id == 0x203 && vel_frame.dlc == 4, "VEL CAN ID == 0x203, DLC == 4");
    float decoded_vonly = 0.0f;
    std::memcpy(&decoded_vonly, &vel_frame.data[0], sizeof(float));
    EXPECT_NEAR(decoded_vonly, -12.75f, 1e-6f, "VEL float32 exact");
}

void testFeedbackParsingAndHighCanId() {
    std::cout << "[TEST] Feedback Parsing & High CAN ID (> 15) Matching..." << std::endl;
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

    auto fault_fb = buildMockFeedbackFrame(0x11, 0x01, MotorError::COIL_OVERTEMP, 0.0f, 0.0f, 0.0f, 45, 102, limits);
    MotorState fault_state = protocol::decodeFeedbackFrame(fault_fb, limits, state.rx_count);
    EXPECT_TRUE(fault_state.hasFault(), "Fault state detected");
    EXPECT_TRUE(fault_state.error == MotorError::COIL_OVERTEMP, "Fault code COIL_OVERTEMP (0xC)");
    EXPECT_TRUE(fault_state.rx_count == 2, "RX counter incremented");

    auto transport = std::make_shared<SocketCanTransport>();
    MotorConfig cfg_high = MotorConfig::J4310(0x15, 0x25, ControlMode::MIT);
    MotorDriver driver_high(transport, cfg_high);

    CanFrame high_id_fb = buildMockFeedbackFrame(0x25, 0x00, MotorError::ENABLED, 1.25f, 0.0f, 0.0f, 28, 29, limits);
    EXPECT_TRUE(driver_high.processFeedbackFrame(high_id_fb), "Feedback matched via unique master_id when can_id > 15");
    EXPECT_TRUE(driver_high.getState().slave_id == 0x15, "Resolved full slave_id (0x15) stored in MotorState");
}

void testP0AndP1Fixes() {
    std::cout << "[TEST] P0 & P1 Fixes (Mode Check / Special Cmd ID / Default Strict Check / CommandLimits)..." << std::endl;
    auto tx_transport = std::make_shared<SocketCanTransport>("vcan0", true);
    auto rx_transport = std::make_shared<SocketCanTransport>("vcan0", true);

    MotorConfig mit_cfg = MotorConfig::J4310(0x01, 0x11, ControlMode::MIT);
    MotorDriver mit_driver(tx_transport, mit_cfg);
    EXPECT_THROW(mit_driver.setPosVel(1.0f, 1.0f), std::logic_error, "setPosVel throws when configured in MIT mode");
    EXPECT_THROW(mit_driver.setVelocity(1.0f), std::logic_error, "setVelocity throws when configured in MIT mode");

    MotorConfig pv_cfg = MotorConfig::J4310(0x02, 0x12, ControlMode::POS_VEL);
    MotorDriver pv_driver(tx_transport, pv_cfg);
    EXPECT_THROW(pv_driver.setMIT(0.0f, 0.0f, 1.0f, 0.1f, 0.0f), std::logic_error, "setMIT throws when configured in POS_VEL mode");

    EXPECT_TRUE(pv_driver.specialCommandCanId() == 0x102, "Default specialCommandCanId includes mode offset (0x102)");
    pv_driver.enable();
    CanFrame wire_frame;
    EXPECT_TRUE(rx_transport->receive(wire_frame, 50) && wire_frame.can_id == 0x102, "POS_VEL enable uses 0x102 by default");

    MotorConfig pv_raw_id_cfg = pv_cfg;
    pv_raw_id_cfg.special_cmd_use_mode_offset = false;
    MotorDriver pv_raw_driver(tx_transport, pv_raw_id_cfg);
    EXPECT_TRUE(pv_raw_driver.specialCommandCanId() == 0x02, "special_cmd_use_mode_offset=false uses raw can_id (0x02)");
    pv_raw_driver.enable();
    EXPECT_TRUE(rx_transport->receive(wire_frame, 50) && wire_frame.can_id == 0x02, "POS_VEL enable uses raw 0x02 when configured");

    EXPECT_TRUE(mit_cfg.strict_range_check == true, "MotorConfig::strict_range_check defaults to true");
    EXPECT_THROW(mit_driver.setMIT(20.0f, 0.0f, 1.0f, 0.1f, 0.0f), std::out_of_range, "Default strict_range_check throws on pos > 12.5");

    MotorConfig joint_cfg = MotorConfig::J4310(0x01, 0x11, ControlMode::MIT);
    joint_cfg.cmd_limits.pos_min = -1.57f;
    joint_cfg.cmd_limits.pos_max = 1.57f;
    MotorDriver joint_driver(tx_transport, joint_cfg);
    EXPECT_THROW(joint_driver.setMIT(2.0f, 0.0f, 10.0f, 0.5f, 0.0f), std::out_of_range,
                 "Physical CommandLimits [-1.57, 1.57] enforced even when inside ProtocolLimits [-12.5, 12.5]");
}

void testExceptionsAndEdgeCases() {
    std::cout << "[TEST] Exception Handling (Invalid Params / Comm Failures / Invalid & Stale Feedback)..." << std::endl;
    const auto limits = ProtocolLimits::forModel(MotorModel::DM_J4310_2EC);

    EXPECT_THROW(protocol::encodeMITFrame(0x01, std::nanf(""), 0.0f, 1.0f, 0.1f, 0.0f, limits),
                 std::invalid_argument, "NaN position rejected");
    EXPECT_THROW(protocol::encodeMITFrame(0x01, 20.0f, 0.0f, 1.0f, 0.1f, 0.0f, limits),
                 std::out_of_range, "Out-of-range pos rejected by default in encodeMITFrame");
    EXPECT_THROW(protocol::encodePosVelFrame(0x01, 0.0f, 50.0f, limits),
                 std::out_of_range, "Out-of-range vel rejected by default in encodePosVelFrame");

    MotorConfig bad_cfg = MotorConfig::J4310(0x00, 0x11);
    EXPECT_THROW(bad_cfg.validate(), std::invalid_argument, "can_id == 0 rejected");

    SocketCanTransport unopened_can;
    CanFrame dummy;
    EXPECT_THROW(unopened_can.send(dummy), CanException, "Send on closed transport throws CanException");
    EXPECT_THROW(unopened_can.receive(dummy, 5), CanException, "Receive on closed transport throws CanException");
    EXPECT_THROW(unopened_can.open("nonexistent_can99"), CanException, "Open invalid interface throws CanException");

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

    auto transport = std::make_shared<SocketCanTransport>();
    MotorConfig cfg = MotorConfig::J4310(0x01, 0x11);
    cfg.state_timeout_ms = 30;
    MotorDriver driver(transport, cfg);

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

    driver.enable();
    CanFrame wire_frame;
    bool got_en = rx_transport->receive(wire_frame, 100);
    EXPECT_TRUE(got_en && wire_frame.can_id == 0x01 && wire_frame.data[7] == 0xFC,
                "vcan0 received Enable frame from MotorDriver");

    driver.setMIT(0.8f, 1.5f, 25.0f, 0.8f, 0.5f);
    bool got_mit = rx_transport->receive(wire_frame, 100);
    EXPECT_TRUE(got_mit && wire_frame.can_id == 0x01 && wire_frame.dlc == 8,
                "vcan0 received MIT frame from MotorDriver");

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

void testUsbTransportAndBridgeEdgeCases() {
    std::cout << "[TEST] USB Transport & Bridge (Malformed Frames / Partial Reads / Disconnect / No Loopback)..." << std::endl;

    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    int rc = ::openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr);
    EXPECT_TRUE(rc == 0 && master_fd >= 0 && slave_fd >= 0, "openpty created virtual serial pair");
    ::close(slave_fd); // DamiaoUsbCanTransport will open slave_name by path

    DamiaoUsbCanTransport usb(slave_name, 921600);
    EXPECT_TRUE(usb.isOpen(), "DamiaoUsbCanTransport opened pty slave");

    // 1. Test garbage bytes + non-0x11 packet + malformed DLC (>8) + malformed ID (>0x7FF) + valid 0x11 packet
    std::vector<uint8_t> stream = {0x00, 0xFF, 0xAA, 0x99, 0x12}; // 5 garbage bytes
    auto ack_pkt = buildUsbRxPacket(0x03, 0x01, 8, false, false, {1, 2, 3, 4, 5, 6, 7, 8}); // non-0x11
    auto bad_dlc_pkt = buildUsbRxPacket(0x11, 0x11, 15, false, false, {1, 2, 3, 4, 5, 6, 7, 8}); // DLC=15 > 8
    auto bad_id_pkt = buildUsbRxPacket(0x11, 0x999, 8, false, false, {1, 2, 3, 4, 5, 6, 7, 8}); // std ID 0x999 > 0x7FF
    auto valid_pkt = buildUsbRxPacket(0x11, 0x11, 8, false, false, {0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 27, 25});

    stream.insert(stream.end(), ack_pkt.begin(), ack_pkt.end());
    stream.insert(stream.end(), bad_dlc_pkt.begin(), bad_dlc_pkt.end());
    stream.insert(stream.end(), bad_id_pkt.begin(), bad_id_pkt.end());
    stream.insert(stream.end(), valid_pkt.begin(), valid_pkt.end());

    // Write stream in two partial chunks to verify stream reassembly
    ssize_t w1 = ::write(master_fd, stream.data(), 19);
    EXPECT_TRUE(w1 == 19, "Wrote first partial chunk to pty master");
    CanFrame rx_frame;
    EXPECT_TRUE(!usb.receive(rx_frame, 10), "Partial stream does not emit premature frame");

    ssize_t w2 = ::write(master_fd, stream.data() + 19, stream.size() - 19);
    EXPECT_TRUE(w2 == static_cast<ssize_t>(stream.size() - 19), "Wrote remaining chunk to pty master");

    EXPECT_TRUE(usb.receive(rx_frame, 50), "Recovered valid frame after garbage/ACK/malformed packets");
    EXPECT_TRUE(rx_frame.can_id == 0x11 && rx_frame.dlc == 8 && rx_frame.data[6] == 27 && rx_frame.data[7] == 25,
                "Decoded valid frame ID, DLC, and payload match");

    auto st = usb.getStats();
    EXPECT_TRUE(st.rx_frames == 1, "Stats counted 1 valid frame");
    EXPECT_TRUE(st.rx_non_data_packets == 1, "Stats counted 1 non-0x11 packet");
    EXPECT_TRUE(st.rx_malformed_frames == 2, "Stats counted 2 malformed frames (bad DLC and bad ID)");
    EXPECT_TRUE(st.rx_discarded_bytes >= 5, "Stats counted discarded garbage bytes");

    // 2. Test Bridge forwarding & anti-loopback using pty + vcan0
    usb.close();

    BridgeConfig bcfg;
    bcfg.socketcan_if = "vcan0";
    bcfg.serial_port = slave_name;
    bcfg.auto_reconnect = false;
    SocketCanUsbBridge bridge(bcfg);
    bridge.start();
    EXPECT_TRUE(bridge.isRunning(), "SocketCanUsbBridge started");

    SocketCanTransport app_can("vcan0", true);
    // Drain any existing frames
    CanFrame drain;
    while (app_can.receive(drain, 5)) {}

    // Inject a frame from USB side -> vcan0, verify app_can receives it, and verify bridge does NOT echo it back to USB!
    int flags = ::fcntl(master_fd, F_GETFL, 0);
    ::fcntl(master_fd, F_SETFL, flags | O_NONBLOCK);
    uint8_t flush_buf[256];
    while (::read(master_fd, flush_buf, sizeof(flush_buf)) > 0) {}

    auto usb_in_pkt = buildUsbRxPacket(0x11, 0x11, 8, false, false, {0x01, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 26, 24});
    EXPECT_TRUE(::write(master_fd, usb_in_pkt.data(), usb_in_pkt.size()) == 16, "Injected USB->vcan0 frame");

    CanFrame vcan_rx;
    EXPECT_TRUE(app_can.receive(vcan_rx, 100), "Application on vcan0 received frame forwarded from USB");
    EXPECT_TRUE(vcan_rx.can_id == 0x11 && vcan_rx.dlc == 8 && vcan_rx.data[6] == 26, "Forwarded frame content intact");

    // Wait 30ms and verify bridge did NOT loop that frame back to master_fd!
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ssize_t echo_bytes = ::read(master_fd, flush_buf, sizeof(flush_buf));
    EXPECT_TRUE(echo_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
                "Zero loopback: USB->vcan0 frame was NOT echoed back to USB");

    // Now send a command from app_can on vcan0 -> Bridge -> USB master_fd
    CanFrame cmd_frame;
    cmd_frame.can_id = 0x7FF;
    cmd_frame.dlc = 8;
    cmd_frame.data = {0x01, 0x00, 0xCC, 0, 0, 0, 0, 0};
    app_can.send(cmd_frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    uint8_t tx_pkt_buf[64]{};
    ssize_t tx_bytes = ::read(master_fd, tx_pkt_buf, sizeof(tx_pkt_buf));
    EXPECT_TRUE(tx_bytes == 30 && tx_pkt_buf[0] == 0x55 && tx_pkt_buf[1] == 0xAA,
                "vcan0->USB command forwarded as exact 30-byte Damiao USB packet");
    uint32_t forwarded_id = 0;
    std::memcpy(&forwarded_id, &tx_pkt_buf[13], 4);
    EXPECT_TRUE(forwarded_id == 0x7FF && tx_pkt_buf[18] == 8 && tx_pkt_buf[21] == 0x01 && tx_pkt_buf[23] == 0xCC,
                "vcan0->USB packet CAN ID (0x7FF), DLC (8), and payload [01 00 CC ...] verified");

    // 3. Simulate USB disconnect by closing master_fd and verifying bridge detects it and exits cleanly
    ::close(master_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(!bridge.isRunning(), "Bridge with auto_reconnect=false stopped cleanly on USB disconnect (POLLHUP/EIO)");
    bridge.stop();
}

} // namespace

int main() {
    testProtocolCodec();
    testFeedbackParsingAndHighCanId();
    testP0AndP1Fixes();
    testExceptionsAndEdgeCases();
    testVcan0Communication();
    testUsbTransportAndBridgeEdgeCases();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
