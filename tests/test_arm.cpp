#include "damiao_arm_sdk/arm.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

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

int openRawVcanSocket(const std::string& ifname = "vcan0") {
    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return -1;

    int loopback = 1;
    ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    int recv_own = 0;
    ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd);
        return -1;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

void drainSocket(int fd) {
    struct pollfd pfd { fd, POLLIN, 0 };
    struct can_frame frame {};
    while (::poll(&pfd, 1, 2) > 0) {
        if (::read(fd, &frame, sizeof(frame)) <= 0) break;
    }
}

uint16_t doubleToUint(double x, double x_min, double x_max, int bits) {
    double span = x_max - x_min;
    double norm = (x - x_min) / span;
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;
    return static_cast<uint16_t>(norm * ((1 << bits) - 1));
}

void sendSimulatedFeedback(int fd, uint32_t recv_can_id, uint8_t slave_id, uint8_t err_code,
                           double q, double dq, double tau, uint8_t t_mos, uint8_t t_rotor) {
    struct can_frame frame {};
    frame.can_id = recv_can_id;
    frame.can_dlc = 8;

    uint16_t q_u = doubleToUint(q, -12.5, 12.5, 16);
    uint16_t dq_u = doubleToUint(dq, -30.0, 30.0, 12);
    uint16_t tau_u = doubleToUint(tau, -10.0, 10.0, 12);

    frame.data[0] = static_cast<uint8_t>(((err_code & 0x0F) << 4) | (slave_id & 0x0F));
    frame.data[1] = static_cast<uint8_t>((q_u >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(q_u & 0xFF);
    frame.data[3] = static_cast<uint8_t>((dq_u >> 4) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(((dq_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F));
    frame.data[5] = static_cast<uint8_t>(tau_u & 0xFF);
    frame.data[6] = t_mos;
    frame.data[7] = t_rotor;

    ssize_t written = ::write(fd, &frame, sizeof(frame));
    EXPECT_TRUE(written == sizeof(frame), "Simulated feedback frame written to vcan0");
}

void testArmConfigAndYaml() {
    std::cout << "[TEST] ArmConfig Validation & YAML Loading (config/arm.yaml)..." << std::endl;

    ArmConfig def_cfg = ArmConfig::defaultJ4310SixAxis("vcan0");
    EXPECT_TRUE(def_cfg.joints.size() == 6, "Default config has 6 joints");
    EXPECT_TRUE(def_cfg.joints[0].send_can_id == 0x01 && def_cfg.joints[5].send_can_id == 0x06,
                "Default send CAN IDs 0x01..0x06");
    EXPECT_TRUE(def_cfg.joints[0].recv_can_id == 0x11 && def_cfg.joints[5].recv_can_id == 0x16,
                "Default recv CAN IDs 0x11..0x16");

    ArmConfig dup_cfg = def_cfg;
    dup_cfg.joints[3].send_can_id = 0x01;
    EXPECT_THROW(dup_cfg.validate(), std::invalid_argument, "Duplicate send_can_id rejected");

    ArmConfig yaml_cfg = ArmConfig::fromYaml(std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml");
    EXPECT_TRUE(yaml_cfg.can_interface == "vcan0" && !yaml_cfg.enable_fd, "YAML loaded can_interface and enable_fd");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_TRUE(yaml_cfg.joints[i].send_can_id == i + 1, "YAML joint send_can_id matches");
        EXPECT_TRUE(yaml_cfg.joints[i].recv_can_id == i + 1 + 0x10, "YAML joint recv_can_id matches");
        EXPECT_NEAR(yaml_cfg.joints[i].pos_max, 3.1416, 1e-4, "YAML joint pos_max == 3.1416");
    }
}

void testStrictEnableAndReinitSafety() {
    std::cout << "[TEST] Strict Enable Preconditions, Staggered Enable Confirmation & Re-init Blocking..." << std::endl;

    int sim_fd = openRawVcanSocket("vcan0");
    EXPECT_TRUE(sim_fd >= 0, "Opened raw vcan0 simulator socket");
    drainSocket(sim_fd);

    ArmConfig cfg = ArmConfig::fromYaml(std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml");
    cfg.state_timeout_ms = 60;
    Arm arm(cfg);

    arm.init();
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Status is DISARMED after init()");

    // 1. Calling enable() when motors have never responded (offline) MUST fail
    EXPECT_THROW(arm.enable(), std::runtime_error, "enable() rejected when motors are offline");
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Status stays DISARMED when pre-enable offline check fails");

    // 2. Only 5 of 6 motors respond -> enable() MUST still fail
    for (std::size_t i = 0; i < NUM_JOINTS - 1; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.refresh(5000);
    drainSocket(sim_fd);
    EXPECT_THROW(arm.enable(), std::runtime_error, "enable() rejected when joint6 is offline");

    // 3. All 6 motors respond, but feedback expires (becomes stale) before enable() -> MUST fail
    sendSimulatedFeedback(sim_fd, 0x16, 6, 0x0, 0.0, 0.0, 0.0, 25, 25);
    arm.recv(5000);
    drainSocket(sim_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(80)); // exceed 60ms timeout
    EXPECT_THROW(arm.enable(), std::runtime_error, "enable() rejected when feedback is stale");

    // 4. All 6 motors have fresh feedback, but during enable() joint 5 reports still DISABLED (0x0) -> enters FAULT
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.refresh(5000);
    drainSocket(sim_fd);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        uint8_t status_code = (i == 4) ? 0x0 : 0x1; // joint5 fails to enable
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, status_code, 0.0, 0.0, 0.0, 25, 25);
    }
    EXPECT_THROW(arm.enable(), std::runtime_error, "enable() throws when a motor fails to confirm enabled state (0x1)");
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Unconfirmed enable transitions Arm to FAULT");

    // 5. Re-initialization while in FAULT is forbidden
    EXPECT_THROW(arm.init(), std::logic_error, "init() forbidden when Arm is in FAULT");

    // 6. Recover from FAULT cleanly, then enable all 6 motors with staggered serial-like replies (1.5ms apart)
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.clearFault();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Recovered to DISARMED");

    std::thread delayed_responder([sim_fd]() {
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.1 * (i + 1), 0.0, 0.0, 26, 27);
        }
    });
    arm.enable();
    delayed_responder.join();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Status is ARMED even when 6 motor replies arrive staggered over 12ms");

    // 7. Re-initialization while in ARMED is forbidden
    EXPECT_THROW(arm.init(), std::logic_error, "init() forbidden when Arm is ARMED");

    ::close(sim_fd);
}

void testStrictFaultRecoveryAndBatchCommandFailures() {
    std::cout << "[TEST] Strict Fault Recovery, Idle Hold Tolerance & Batch Command Failure Handling..." << std::endl;

    int sim_fd = openRawVcanSocket("vcan0");
    EXPECT_TRUE(sim_fd >= 0, "Opened raw vcan0 simulator socket");
    drainSocket(sim_fd);

    ArmConfig cfg = ArmConfig::fromYaml(std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml");
    cfg.state_timeout_ms = 60;
    Arm arm(cfg);
    arm.init();

    // Bring all 6 motors online and ARMED
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.refresh(5000);
    drainSocket(sim_fd);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.enable();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Arm is ARMED");

    // 1. Trigger hardware fault on joint 2 (0x9 UNDERVOLTAGE)
    sendSimulatedFeedback(sim_fd, 0x12, 2, 0x9, 0.0, 0.0, 0.0, 25, 25);
    arm.recv(5000);
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Joint 2 undervoltage triggers FAULT");

    // 2. Attempt clearFault() while joint 2 STILL reports fault 0x9 -> MUST fail and stay in FAULT
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        uint8_t code = (i == 1) ? 0x9 : 0x0;
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, code, 0.0, 0.0, 0.0, 25, 25);
    }
    EXPECT_THROW(arm.clearFault(), std::runtime_error, "clearFault() rejected while joint 2 still has fault 0x9");
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Status remains FAULT when fault condition is unresolved");

    // 3. Attempt clearFault() when feedback is missing -> MUST fail and stay in FAULT
    EXPECT_THROW(arm.clearFault(), std::runtime_error, "clearFault() rejected when motor does not reply to disable");
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Status remains FAULT when motor does not reply");

    // 4. Attempt clearFault() when a motor refuses to disable (still reports 0x1 ENABLED) -> MUST fail
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        uint8_t code = (i == 0) ? 0x1 : 0x0;
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, code, 0.0, 0.0, 0.0, 25, 25);
    }
    EXPECT_THROW(arm.clearFault(), std::runtime_error, "clearFault() rejected when joint 1 remains enabled");
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Status remains FAULT when motor is still enabled");

    // 5. All 6 motors report healthy disabled state (0x0) -> clearFault() succeeds -> DISARMED
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.clearFault();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "clearFault() succeeds once all 6 motors are healthy & disabled");

    // 6. Re-enable to ARMED and verify idle hold > state_timeout_ms does NOT falsely trip FAULT
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.enable();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Arm re-enabled to ARMED");

    // Wait 80ms (> 60ms state_timeout_ms) with no commands sent (idle hold); next command should still succeed!
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    std::array<MITParam, NUM_JOINTS> batch_cmds;
    batch_cmds.fill(MITParam{10.0, 0.5, 0.2, 0.0, 0.0});
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.2, 0.0, 0.0, 25, 25);
    }
    arm.sendMITAll(batch_cmds); // auto-drains the feedback via recv_all(0)
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "sendMITAll after idle pause succeeds and auto-drains replies");

    // 7. Verify true communication timeout: sendMITAll with NO motor reply for > 60ms triggers FAULT on next check/send
    arm.sendMITAll(batch_cmds);
    drainSocket(sim_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_THROW(arm.sendMITAll(batch_cmds), std::runtime_error,
                 "sendMITAll throws when previous command went unanswered beyond state_timeout_ms");
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Unanswered command timeout transitions Arm to FAULT");

    // Recover and test underlying CAN socket write failure -> FAULT
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.clearFault();
    drainSocket(sim_fd);
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.enable();
    drainSocket(sim_fd);

    int raw_fd = arm.rawOpenArm().get_master_can_device_collection().get_can_socket().get_socket_fd();
    ::close(raw_fd);

    EXPECT_THROW(arm.sendMITAll(batch_cmds), std::runtime_error,
                 "sendMITAll throws runtime_error when underlying CAN write fails");
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT,
                "CAN write failure during sendMITAll immediately transitions Arm to FAULT");

    ::close(sim_fd);
}

} // namespace

int main() {
    testArmConfigAndYaml();
    testStrictEnableAndReinitSafety();
    testStrictFaultRecoveryAndBatchCommandFailures();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Arm Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
