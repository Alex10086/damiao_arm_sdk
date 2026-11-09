#include "damiao_arm_sdk/damiao_usb_transport.hpp"
#include "damiao_arm_sdk/socketcan_bridge.hpp"

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pty.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
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

void testUsbTransportValidationAndReassembly() {
    std::cout << "[TEST] DamiaoUsbCanTransport (Validation / Partial Stream Reassembly / Malformed Filtering)..." << std::endl;

    DamiaoUsbCanTransport unopened;
    CanFrame dummy;
    EXPECT_THROW(unopened.send(dummy), CanException, "Send on unopened USB transport throws CanException");
    EXPECT_THROW(unopened.receive(dummy, 5), CanException, "Receive on unopened USB transport throws CanException");

    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    int rc = ::openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr);
    EXPECT_TRUE(rc == 0 && master_fd >= 0 && slave_fd >= 0, "openpty created virtual serial pair");
    ::close(slave_fd);

    DamiaoUsbCanTransport usb(slave_name, 921600);
    EXPECT_TRUE(usb.isOpen(), "DamiaoUsbCanTransport opened pty slave");

    CanFrame bad_dlc;
    bad_dlc.can_id = 0x01;
    bad_dlc.dlc = 9;
    EXPECT_THROW(usb.send(bad_dlc), std::invalid_argument, "DLC > 8 rejected on send");

    CanFrame bad_id;
    bad_id.can_id = 0x800;
    bad_id.dlc = 8;
    EXPECT_THROW(usb.send(bad_id), std::invalid_argument, "Standard CAN ID > 0x7FF rejected on send");

    CanFrame err_frame;
    err_frame.can_id = 0x01;
    err_frame.dlc = 8;
    err_frame.is_error = true;
    EXPECT_THROW(usb.send(err_frame), std::invalid_argument, "Error frame rejected on send");

    std::vector<uint8_t> stream = {0x00, 0xFF, 0xAA, 0x99, 0x12};
    auto ack_pkt = buildUsbRxPacket(0x03, 0x01, 8, false, false, {1, 2, 3, 4, 5, 6, 7, 8});
    auto bad_dlc_pkt = buildUsbRxPacket(0x11, 0x11, 15, false, false, {1, 2, 3, 4, 5, 6, 7, 8});
    auto bad_id_pkt = buildUsbRxPacket(0x11, 0x999, 8, false, false, {1, 2, 3, 4, 5, 6, 7, 8});
    auto valid_pkt = buildUsbRxPacket(0x11, 0x11, 8, false, false, {0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 27, 25});

    stream.insert(stream.end(), ack_pkt.begin(), ack_pkt.end());
    stream.insert(stream.end(), bad_dlc_pkt.begin(), bad_dlc_pkt.end());
    stream.insert(stream.end(), bad_id_pkt.begin(), bad_id_pkt.end());
    stream.insert(stream.end(), valid_pkt.begin(), valid_pkt.end());

    EXPECT_TRUE(::write(master_fd, stream.data(), 19) == 19, "Wrote partial first chunk");
    CanFrame rx_frame;
    EXPECT_TRUE(!usb.receive(rx_frame, 10), "Partial stream waits for remaining bytes");

    EXPECT_TRUE(::write(master_fd, stream.data() + 19, stream.size() - 19) == static_cast<ssize_t>(stream.size() - 19),
                "Wrote remaining chunk");
    EXPECT_TRUE(usb.receive(rx_frame, 50), "Decoded valid frame after garbage & malformed packets");
    EXPECT_TRUE(rx_frame.can_id == 0x11 && rx_frame.dlc == 8 && rx_frame.data[6] == 27 && rx_frame.data[7] == 25,
                "Decoded CAN ID, DLC, and payload match");

    auto st = usb.getStats();
    EXPECT_TRUE(st.rx_frames == 1, "Stats counted 1 valid frame");
    EXPECT_TRUE(st.rx_non_data_packets == 1, "Stats counted 1 non-0x11 packet");
    EXPECT_TRUE(st.rx_malformed_frames == 2, "Stats counted 2 malformed frames");
    EXPECT_TRUE(st.rx_discarded_bytes >= 5, "Stats counted discarded garbage bytes");

    usb.close();
    ::close(master_fd);
}

void testReadOnlyCommandFilterRules() {
    std::cout << "[TEST] Bridge Default Transparent Mode & Optional Read-Only Command Filter Rules..." << std::endl;

    BridgeConfig default_cfg;
    EXPECT_TRUE(default_cfg.read_only == false, "BridgeConfig::read_only defaults to false (transparent)");

    CanFrame refresh_frame;
    refresh_frame.can_id = 0x7FF;
    refresh_frame.dlc = 8;
    refresh_frame.data = {0x01, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_TRUE(SocketCanUsbBridge::isReadOnlyAllowedTxFrame(refresh_frame),
                "Status refresh (0x7FF, D[2]=0xCC) allowed in read-only mode");

    CanFrame query_param_frame;
    query_param_frame.can_id = 0x7FF;
    query_param_frame.dlc = 8;
    query_param_frame.data = {0x01, 0x00, 0x33, 0x15, 0x00, 0x00, 0x00, 0x00};
    EXPECT_TRUE(SocketCanUsbBridge::isReadOnlyAllowedTxFrame(query_param_frame),
                "Parameter query (0x7FF, D[2]=0x33) allowed in read-only mode");

    CanFrame write_param_frame;
    write_param_frame.can_id = 0x7FF;
    write_param_frame.dlc = 8;
    write_param_frame.data = {0x01, 0x00, 0x55, 0x0A, 0x01, 0x00, 0x00, 0x00};
    EXPECT_TRUE(!SocketCanUsbBridge::isReadOnlyAllowedTxFrame(write_param_frame),
                "Register write (0x7FF, D[2]=0x55) BLOCKED in read-only mode");

    CanFrame enable_frame;
    enable_frame.can_id = 0x01;
    enable_frame.dlc = 8;
    enable_frame.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    EXPECT_TRUE(!SocketCanUsbBridge::isReadOnlyAllowedTxFrame(enable_frame),
                "Motor enable (0x01, D[7]=0xFC) BLOCKED in read-only mode");

    CanFrame mit_frame;
    mit_frame.can_id = 0x01;
    mit_frame.dlc = 8;
    mit_frame.data = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    EXPECT_TRUE(!SocketCanUsbBridge::isReadOnlyAllowedTxFrame(mit_frame),
                "MIT control frame (0x01) BLOCKED in read-only mode");

    CanFrame posvel_frame;
    posvel_frame.can_id = 0x101;
    posvel_frame.dlc = 8;
    EXPECT_TRUE(!SocketCanUsbBridge::isReadOnlyAllowedTxFrame(posvel_frame),
                "POS_VEL control frame (0x101) BLOCKED in read-only mode");
}

void testBridgeEndToEndWithOpenArmAndReadOnlyProtection() {
    std::cout << "[TEST] End-to-End Bridge + Transparent Forwarding + Opt-in Read-Only Filter..." << std::endl;

    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    int rc = ::openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr);
    EXPECT_TRUE(rc == 0 && master_fd >= 0 && slave_fd >= 0, "openpty created virtual serial pair");
    ::close(slave_fd);

    int flags = ::fcntl(master_fd, F_GETFL, 0);
    ::fcntl(master_fd, F_SETFL, flags | O_NONBLOCK);

    BridgeConfig bcfg;
    bcfg.socketcan_if = "vcan0";
    bcfg.serial_port = slave_name;
    bcfg.auto_reconnect = false;
    SocketCanUsbBridge bridge(bcfg);
    EXPECT_TRUE(!bridge.isReadOnly(), "SocketCanUsbBridge defaults to transparent writable mode");
    bridge.start();
    EXPECT_TRUE(bridge.isRunning(), "SocketCanUsbBridge started");

    // 1. Initialize OpenArm("vcan0", false) WITHOUT control_modes and verify ZERO bytes sent to USB!
    openarm::can::socket::OpenArm openarm("vcan0", false);
    openarm.init_arm_motors({openarm::damiao_motor::MotorType::DM4310}, {0x01}, {0x11});
    openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    openarm.recv_all(1000);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    uint8_t tx_buf[128]{};
    ssize_t init_bytes = ::read(master_fd, tx_buf, sizeof(tx_buf));
    EXPECT_TRUE(init_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
                "OpenArm init_arm_motors without control_modes emits ZERO bus writes");

    // 2. Default transparent mode forwards enable_all() directly without hidden blocking
    openarm.enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ssize_t enable_bytes = ::read(master_fd, tx_buf, sizeof(tx_buf));
    EXPECT_TRUE(enable_bytes == 30 && tx_buf[28] == 0xFC,
                "Default Bridge forwarded enable frame (0xFC)");

    // 3. Opt-in to read-only mode (setReadOnly(true)) and verify motion/enable commands are blocked
    bridge.setReadOnly(true);
    EXPECT_TRUE(bridge.isReadOnly(), "Bridge read-only filter enabled");
    openarm.enable_all();
    openarm.get_arm().mit_control_one(0, {0.5, 0.0, 10.0, 0.5, 0.0});
    openarm.get_arm().set_control_mode_all(openarm::damiao_motor::ControlMode::MIT);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ssize_t blocked_bytes = ::read(master_fd, tx_buf, sizeof(tx_buf));
    EXPECT_TRUE(blocked_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
                "Read-Only Bridge blocked enable_all, mit_control_one, and set_control_mode_all");
    auto st_after_block = bridge.getStats();
    EXPECT_TRUE(st_after_block.sock_readonly_blocked == 3 && st_after_block.sock_to_usb_forwarded == 1,
                "BridgeStats recorded 3 blocked unsafe commands and 1 forwarded");

    // 4. Send a safe passive status refresh from OpenArm while in read-only mode and verify it IS forwarded
    openarm.refresh_one(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ssize_t refresh_bytes = ::read(master_fd, tx_buf, sizeof(tx_buf));
    EXPECT_TRUE(refresh_bytes == 30 && tx_buf[0] == 0x55 && tx_buf[1] == 0xAA,
                "Read-Only Bridge allowed passive refresh_one(0) (30-byte USB packet)");

    // 5. Inject simulated J4310 feedback from USB -> vcan0 -> OpenArm, and verify no loopback echo
    auto usb_fb = buildUsbRxPacket(0x11, 0x11, 8, false, false, {0x01, 0x80, 0x00, 0x80, 0x08, 0x00, 26, 25});
    EXPECT_TRUE(::write(master_fd, usb_fb.data(), usb_fb.size()) == 16, "Injected simulated J4310 feedback on USB");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    openarm.recv_all(2000);

    auto m0 = openarm.get_arm().get_motor(0);
    EXPECT_TRUE(openarm.get_arm().get_link_stats(0).responses == 1, "OpenArm received simulated feedback via Bridge");
    EXPECT_TRUE(m0.get_state_tmos() == 26 && m0.get_state_trotor() == 25, "OpenArm decoded temperatures 26C / 25C");

    ssize_t echo_check = ::read(master_fd, tx_buf, sizeof(tx_buf));
    EXPECT_TRUE(echo_check < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
                "Zero loopback: USB->vcan0 frame was NOT echoed back to USB");

    // 6. Simulate USB disconnect and verify clean exit
    ::close(master_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(!bridge.isRunning(), "Bridge with auto_reconnect=false stopped cleanly on USB disconnect");
    bridge.stop();
}

} // namespace

int main() {
    testUsbTransportValidationAndReassembly();
    testReadOnlyCommandFilterRules();
    testBridgeEndToEndWithOpenArmAndReadOnlyProtection();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
