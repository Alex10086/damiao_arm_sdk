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

    // Duplicate ID check
    ArmConfig dup_cfg = def_cfg;
    dup_cfg.joints[3].send_can_id = 0x01;
    EXPECT_THROW(dup_cfg.validate(), std::invalid_argument, "Duplicate send_can_id rejected");

    // Load from config/arm.yaml
    ArmConfig yaml_cfg = ArmConfig::fromYaml( std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml" );
    EXPECT_TRUE(yaml_cfg.can_interface == "vcan0" && !yaml_cfg.enable_fd, "YAML loaded can_interface and enable_fd");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_TRUE(yaml_cfg.joints[i].send_can_id == i + 1, "YAML joint send_can_id matches");
        EXPECT_TRUE(yaml_cfg.joints[i].recv_can_id == i + 1 + 0x10, "YAML joint recv_can_id matches");
        EXPECT_NEAR(yaml_cfg.joints[i].pos_max, 3.1416, 1e-4, "YAML joint pos_max == 3.1416");
    }
}

void testSafetyStateMachineAndCommandValidation() {
    std::cout << "[TEST] Arm Safety State Machine (DISCONNECTED -> DISARMED -> ARMED -> FAULT) & validateCommand..." << std::endl;

    int sim_fd = openRawVcanSocket("vcan0");
    EXPECT_TRUE(sim_fd >= 0, "Opened raw vcan0 simulator socket");
    drainSocket(sim_fd);

    ArmConfig cfg = ArmConfig::fromYaml(std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml");
    cfg.state_timeout_ms = 60;
    Arm arm(cfg);

    // 1. Initial state is DISCONNECTED
    EXPECT_TRUE(arm.status() == ArmStatus::DISCONNECTED, "Initial status is DISCONNECTED");
    EXPECT_TRUE(!arm.checkHealth(), "checkHealth() is false when DISCONNECTED");
    MITParam zero_mit{10.0, 0.5, 0.5, 0.0, 0.0};
    EXPECT_THROW(arm.sendMIT(0, zero_mit), std::logic_error, "sendMIT rejected when DISCONNECTED");
    EXPECT_THROW(arm.enable(), std::logic_error, "enable() rejected when DISCONNECTED");

    // 2. init() transitions to DISARMED with zero bus writes
    arm.init();
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Status is DISARMED after init()");
    EXPECT_TRUE(arm.checkHealth(), "checkHealth() is true when DISARMED and no faults");

    struct pollfd pfd { sim_fd, POLLIN, 0 };
    EXPECT_TRUE(::poll(&pfd, 1, 10) == 0, "init() emitted zero CAN frames on vcan0");

    // 3. In DISARMED state, motion commands are still rejected
    EXPECT_THROW(arm.sendMIT(0, zero_mit), std::logic_error, "sendMIT rejected when DISARMED");

    // 4. Test validateCommand() boundaries
    EXPECT_TRUE(arm.validateCommand(0, zero_mit), "Valid MIT command passes validateCommand");
    EXPECT_TRUE(!arm.validateCommand(6, zero_mit), "Out-of-bounds joint index 6 rejected");
    EXPECT_TRUE(!arm.validateCommand(0, MITParam{10.0, 0.5, 5.0, 0.0, 0.0}),
                "Position 5.0 rad > pos_max (3.1416 rad) rejected");
    EXPECT_TRUE(!arm.validateCommand(0, MITParam{10.0, 0.5, 0.0, 50.0, 0.0}),
                "Velocity 50.0 rad/s > vel_max (30.0 rad/s) rejected");
    EXPECT_TRUE(!arm.validateCommand(0, MITParam{10.0, 0.5, 0.0, 0.0, 15.0}),
                "Torque 15.0 Nm > tau_max (10.0 Nm) rejected");
    EXPECT_TRUE(!arm.validateCommand(0, MITParam{-1.0, 0.5, 0.0, 0.0, 0.0}),
                "Negative Kp rejected");
    EXPECT_TRUE(!arm.validateCommand(0, MITParam{10.0, 0.5, std::nan(""), 0.0, 0.0}),
                "NaN position rejected");
    EXPECT_TRUE(!arm.validateCommand(0, PosVelParam{0.0, 1.0}),
                "PosVel command rejected when joint is configured in MIT mode");

    // 5. Passive refresh() in DISARMED state updates all 6 joint states
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x0, 0.1 * (i + 1), -0.2 * (i + 1), 0.05 * (i + 1), 25 + i, 26 + i);
    }
    ArmState st = arm.refresh(5000);
    drainSocket(sim_fd); // drain the 6 refresh requests sent by arm.refresh()

    EXPECT_TRUE(st.status == ArmStatus::DISARMED && st.all_online && !st.any_fault,
                "All 6 joints online and healthy after refresh()");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(st.joints[i].position, 0.1 * (i + 1), 2e-3, "Joint position feedback matches");
        EXPECT_NEAR(st.joints[i].velocity, -0.2 * (i + 1), 3e-2, "Joint velocity feedback matches");
        EXPECT_TRUE(st.joints[i].t_mos == static_cast<int>(25 + i), "Joint MOS temperature matches");
    }

    // 6. enable() transitions to ARMED and allows valid MIT control
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.1 * (i + 1), 0.0, 0.0, 26, 27);
    }
    arm.enable();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Status is ARMED after enable()");
    EXPECT_TRUE(arm.getState().joints[0].enabled, "Joint 0 reported enabled=true");

    // Valid sendMIT succeeds in ARMED state
    arm.sendMIT(0, zero_mit);
    struct can_frame wire {};
    EXPECT_TRUE(::poll(&pfd, 1, 20) > 0 && ::read(sim_fd, &wire, sizeof(wire)) == sizeof(wire),
                "sendMIT emitted CAN frame on vcan0");
    EXPECT_TRUE(wire.can_id == 0x01 && wire.can_dlc == 8, "MIT frame CAN ID == 0x01, DLC == 8");

    // Invalid command in ARMED state throws std::invalid_argument
    EXPECT_THROW(arm.sendMIT(0, MITParam{10.0, 0.5, 9.9, 0.0, 0.0}), std::invalid_argument,
                 "Out-of-range MIT command throws invalid_argument even when ARMED");

    // 7. Hardware fault reported by motor (e.g. OVERVOLTAGE 0x8 on joint 3) triggers FAULT & auto-disables
    sendSimulatedFeedback(sim_fd, 0x14, 0x04, 0x8, 0.4, 0.0, 0.0, 40, 42);
    arm.recv(5000);
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Motor fault code 0x8 transitions Arm to FAULT");
    EXPECT_TRUE(!arm.getState().fault_reason.empty(), "Fault reason recorded: " + arm.getState().fault_reason);
    EXPECT_THROW(arm.sendMIT(0, zero_mit), std::logic_error, "sendMIT blocked in FAULT state");

    // 8. clearFault() resets back to DISARMED
    sendSimulatedFeedback(sim_fd, 0x14, 0x04, 0x0, 0.4, 0.0, 0.0, 30, 31);
    arm.clearFault();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "clearFault() transitions FAULT -> DISARMED");

    // 9. Communication timeout detection in ARMED state
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        sendSimulatedFeedback(sim_fd, 0x11 + i, i + 1, 0x1, 0.0, 0.0, 0.0, 25, 25);
    }
    arm.enable();
    drainSocket(sim_fd);
    EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Re-enabled to ARMED");

    std::this_thread::sleep_for(std::chrono::milliseconds(80)); // exceed state_timeout_ms (60ms)
    EXPECT_TRUE(!arm.checkHealth() && arm.status() == ArmStatus::FAULT,
                "Stale feedback in ARMED state transitions Arm to FAULT");

    ::close(sim_fd);
}

} // namespace

int main() {
    testArmConfigAndYaml();
    testSafetyStateMachineAndCommandValidation();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Arm Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
