#include "damiao_arm_sdk/arm.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
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
    EXPECT_TRUE(def_cfg.joint_count == NUM_JOINTS && def_cfg.activeMotorCount() == 6,
                "Default active count remains six");
    auto count_cfg = def_cfg;
    count_cfg.joint_count = 0;
    EXPECT_THROW(count_cfg.validate(), std::invalid_argument, "Zero active joints rejected");
    count_cfg.joint_count = NUM_JOINTS + 1;
    EXPECT_THROW(count_cfg.validate(), std::invalid_argument, "More than six active joints rejected");
    count_cfg.joint_count = 1;
    count_cfg.joints[0].recv_can_id = 0;
    count_cfg.joints[1] = count_cfg.joints[0];
    count_cfg.validate();
    EXPECT_TRUE(count_cfg.activeMotorCount() == 1, "Inactive duplicate IDs are ignored, receive ID zero accepted");
    count_cfg.joint_count = 2;
    EXPECT_THROW(count_cfg.validate(), std::invalid_argument, "Duplicate active send IDs rejected");
    count_cfg.joints[1].send_can_id = 2;
    EXPECT_THROW(count_cfg.validate(), std::invalid_argument, "Duplicate active receive ID zero rejected");
    count_cfg.joint_count = 1;
    count_cfg.has_gripper = true;
    EXPECT_TRUE(count_cfg.activeMotorCount() == 2, "Gripper follows configured active prefix");
    count_cfg.gripper.recv_can_id = 0;
    EXPECT_THROW(count_cfg.validate(), std::invalid_argument, "Gripper cannot duplicate active receive ID zero");
    EXPECT_TRUE(def_cfg.joints[0].send_can_id == 0x01 && def_cfg.joints[5].send_can_id == 0x06,
                "Default send CAN IDs 0x01..0x06");
    EXPECT_TRUE(def_cfg.joints[0].recv_can_id == 0x11 && def_cfg.joints[5].recv_can_id == 0x16,
                "Default recv CAN IDs 0x11..0x16");

    ArmConfig dup_cfg = def_cfg;
    dup_cfg.joints[3].send_can_id = 0x01;
    EXPECT_THROW(dup_cfg.validate(), std::invalid_argument, "Duplicate send_can_id rejected");

    ArmConfig grip_dup = def_cfg;
    grip_dup.has_gripper = true;
    grip_dup.gripper.send_can_id = 0x06;
    EXPECT_THROW(grip_dup.validate(), std::invalid_argument, "Duplicate gripper send_can_id rejected");

    ArmConfig yaml_cfg = ArmConfig::fromYaml(std::string(DAMIAO_SOURCE_DIR) + "/config/arm.yaml");
    EXPECT_TRUE(yaml_cfg.can_interface == "vcan0" && !yaml_cfg.enable_fd, "YAML loaded can_interface and enable_fd");
    EXPECT_TRUE(yaml_cfg.joint_count == NUM_JOINTS, "YAML without joint_count retains six active joints");
    auto loadTemporaryYaml = [](const std::string& contents) {
        char path[] = "/tmp/damiao_arm_config_XXXXXX";
        const int fd = ::mkstemp(path);
        if (fd < 0) throw std::runtime_error("Cannot create temporary YAML");
        const auto written = ::write(fd, contents.data(), contents.size());
        ::close(fd);
        if (written != static_cast<ssize_t>(contents.size())) {
            ::unlink(path);
            throw std::runtime_error("Cannot write temporary YAML");
        }
        try {
            auto cfg = ArmConfig::fromYaml(path);
            ::unlink(path);
            return cfg;
        } catch (...) {
            ::unlink(path);
            throw;
        }
    };
    const std::string single_joint_yaml =
        "can_interface: vcan0\njoints:\n"
        "  - name: single\n"
        "    motor_type: DM3507\n"
        "    send_can_id: 1\n"
        "    recv_can_id: 0\n"
        "    control_mode: MIT\n"
        "    pos_min: -12.566\n"
        "    pos_max: 12.566\n"
        "    vel_max: 50\n"
        "    tau_max: 5\n";
    const auto single_yaml = loadTemporaryYaml("joint_count: 1\n" + single_joint_yaml);
    EXPECT_TRUE(single_yaml.joint_count == 1 && single_yaml.activeMotorCount() == 1 &&
                single_yaml.joints[0].recv_can_id == 0 &&
                single_yaml.joints[0].motor_type == MotorType::DM3507,
                "Single DM3507 YAML loads receive ID zero");
    EXPECT_THROW(loadTemporaryYaml("joint_count: 2\n" + single_joint_yaml),
                 std::runtime_error, "YAML joint count must match declared active count");
    EXPECT_THROW(loadTemporaryYaml(single_joint_yaml), std::runtime_error,
                 "Omitting joint_count still requires six YAML entries");
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

    // 6. Recover from FAULT cleanly, then enable all 6 motors with staggered serial-like replies (2ms apart)
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
    std::cout << "[TEST] Confirmed disable and autonomous idle monitoring..." << std::endl;
    const int fd = openRawVcanSocket();
    if (fd < 0) throw std::runtime_error("Cannot open simulator socket");
    std::atomic<bool> running{true};
    std::atomic<int> missing{0}, stuck{0}, fault{0};
    std::atomic<unsigned> queries{0}, motions{0};
    std::thread simulator([&] {
        std::array<bool, NUM_JOINTS> enabled{};
        while (running.load()) {
            pollfd pfd{fd, POLLIN, 0};
            if (::poll(&pfd, 1, 2) <= 0) continue;
            can_frame request{};
            if (::read(fd, &request, sizeof(request)) != sizeof(request)) continue;
            unsigned id = request.can_id;
            if (id == 0x7FF && request.data[2] == 0xCC) {
                id = request.data[0] | (request.data[1] << 8);
                ++queries;
            } else if (id >= 1 && id <= NUM_JOINTS && request.data[7] == 0xFC) {
                enabled[id - 1] = true;
            } else if (id >= 1 && id <= NUM_JOINTS && request.data[7] == 0xFD) {
                if (stuck.load() != static_cast<int>(id)) enabled[id - 1] = false;
            } else {
                ++motions;
                continue;
            }
            if (id < 1 || id > NUM_JOINTS || missing.load() == static_cast<int>(id)) continue;
            can_frame reply{};
            reply.can_id = 0x10 + id;
            reply.can_dlc = 8;
            const unsigned code = fault.load() == static_cast<int>(id) ? 9 : enabled[id - 1];
            reply.data[0] = (code << 4) | id;
            reply.data[1] = 0x80;
            reply.data[3] = 0x80;
            reply.data[4] = 0x08;
            reply.data[6] = reply.data[7] = 25;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (::write(fd, &reply, sizeof(reply)) != sizeof(reply)) running = false;
        }
    });
    try {
        auto cfg = ArmConfig::defaultJ4310SixAxis();
        cfg.state_timeout_ms = 150;
        Arm arm(cfg);
        arm.init();
        auto bringOnline = [&] {
            EXPECT_TRUE(arm.refreshUntilAllFresh(50), "refreshUntilAllFresh brings all 6 joints online");
        };
        auto waitStatus = [&](ArmStatus expected) {
            const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (arm.status() != expected && std::chrono::steady_clock::now() < end)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            return arm.status() == expected;
        };
        bringOnline();
        arm.enable();
        const auto before = arm.getState().joints[5].responses;
        const auto q = queries.load();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Idle arm remains armed with valid query replies");
        EXPECT_TRUE(queries.load() >= q + 12 && motions.load() == 0,
                    "Idle monitor queries repeatedly without sending motion commands");
        EXPECT_TRUE(arm.getState().joints[5].responses > before && !arm.getState().joints[5].stale,
                    "Idle monitor consumes fresh feedback without caller recv");
        arm.disable();
        EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Delayed six-axis disable confirmation succeeds");
        for (const auto& joint : arm.getState().joints)
            EXPECT_TRUE(!joint.enabled && !joint.stale, "Every joint confirmed disabled");

        bringOnline();
        arm.enable();
        stuck = 3;
        EXPECT_THROW(arm.disable(), std::runtime_error, "Still-enabled joint rejects disable");
        EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Unconfirmed disable latches FAULT");
        stuck = 0;
        arm.clearFault();
        bringOnline();
        arm.enable();
        missing = 6;
        EXPECT_THROW(arm.disable(), std::runtime_error, "Missing disable feedback rejects disable");
        EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Missing confirmation cannot report DISARMED");
        missing = 0;
        arm.clearFault();
        bringOnline();
        arm.enable();
        missing = 2;
        EXPECT_TRUE(waitStatus(ArmStatus::FAULT), "Idle disconnect faults autonomously without caller polling");
        EXPECT_THROW(arm.clearFault(), std::runtime_error, "Offline joint prevents fault recovery");
        missing = 0;
        arm.clearFault();
        bringOnline();
        arm.enable();
        fault = 4;
        EXPECT_TRUE(waitStatus(ArmStatus::FAULT), "Idle hardware fault is detected autonomously");
        EXPECT_THROW(arm.clearFault(), std::runtime_error, "Active hardware fault prevents recovery");
        fault = 0;
        arm.clearFault();
    } catch (...) {
        running = false;
        simulator.join();
        ::close(fd);
        throw;
    }
    running = false;
    simulator.join();
    ::close(fd);
}

void testGripperAndCommandWatchdog() {
    std::cout << "[TEST] Optional 7th-axis Gripper + RID::TIMEOUT Config + Command Watchdog..." << std::endl;
    const int fd = openRawVcanSocket();
    if (fd < 0) throw std::runtime_error("Cannot open simulator socket");
    drainSocket(fd);

    std::atomic<bool> running{true};
    std::atomic<bool> spoof_readback_with_0x55{false};
    std::atomic<unsigned> queries{0}, motions{0}, disables{0};
    std::thread simulator([&] {
        std::array<bool, NUM_JOINTS + 1> enabled{};
        std::array<uint32_t, NUM_JOINTS + 1> timeout_reg{};
        while (running.load()) {
            pollfd pfd{fd, POLLIN, 0};
            if (::poll(&pfd, 1, 2) <= 0) continue;
            can_frame request{};
            if (::read(fd, &request, sizeof(request)) != sizeof(request)) continue;
            unsigned id = request.can_id;
            if (id == 0x7FF && (request.data[2] == 0x55 || request.data[2] == 0x33) &&
                request.data[3] == static_cast<uint8_t>(openarm::damiao_motor::RID::TIMEOUT)) {
                unsigned target = request.data[0] | (request.data[1] << 8);
                if (target >= 1 && target <= NUM_JOINTS + 1) {
                    if (request.data[2] == 0x55) {
                        uint32_t val = static_cast<uint32_t>(request.data[4]) |
                                       (static_cast<uint32_t>(request.data[5]) << 8) |
                                       (static_cast<uint32_t>(request.data[6]) << 16) |
                                       (static_cast<uint32_t>(request.data[7]) << 24);
                        timeout_reg[target - 1] = val;
                    }
                    uint32_t cur = timeout_reg[target - 1];
                    can_frame reply{};
                    reply.can_id = 0x10 + target;
                    reply.can_dlc = 8;
                    reply.data[0] = request.data[0];
                    reply.data[1] = request.data[1];
                    reply.data[2] = (spoof_readback_with_0x55.load() && request.data[2] == 0x33) ? 0x55 : request.data[2];
                    reply.data[3] = request.data[3];
                    reply.data[4] = static_cast<uint8_t>(cur & 0xFF);
                    reply.data[5] = static_cast<uint8_t>((cur >> 8) & 0xFF);
                    reply.data[6] = static_cast<uint8_t>((cur >> 16) & 0xFF);
                    reply.data[7] = static_cast<uint8_t>((cur >> 24) & 0xFF);
                    if (::write(fd, &reply, sizeof(reply)) != sizeof(reply)) running = false;
                }
                continue;
            }
            if (id == 0x7FF && request.data[2] == 0xCC) {
                id = request.data[0] | (request.data[1] << 8);
                ++queries;
            } else if (id >= 1 && id <= NUM_JOINTS + 1 && request.data[7] == 0xFC) {
                enabled[id - 1] = true;
            } else if (id >= 1 && id <= NUM_JOINTS + 1 && request.data[7] == 0xFD) {
                enabled[id - 1] = false;
                ++disables;
            } else if (id >= 1 && id <= NUM_JOINTS + 1) {
                ++motions;
            } else {
                continue;
            }
            if (id < 1 || id > NUM_JOINTS + 1) continue;
            can_frame reply{};
            reply.can_id = 0x10 + id;
            reply.can_dlc = 8;
            const unsigned code = enabled[id - 1] ? 1 : 0;
            reply.data[0] = static_cast<uint8_t>((code << 4) | id);
            reply.data[1] = 0x80;
            reply.data[3] = 0x80;
            reply.data[4] = 0x08;
            reply.data[6] = reply.data[7] = 25;
            if (::write(fd, &reply, sizeof(reply)) != sizeof(reply)) running = false;
        }
    });

    try {
        auto cfg = ArmConfig::defaultJ4310SixAxis();
        cfg.has_gripper = true;
        cfg.state_timeout_ms = 200;
        cfg.command_timeout_ms = 90; // active command watchdog enabled (ROS mode)

        Arm arm(cfg);
        arm.init();
        spoof_readback_with_0x55 = true;
        EXPECT_TRUE(!arm.configureMotorTimeout(2000, 30), "configureMotorTimeout rejects 0x55 packet masquerading as 0x33 readback");
        spoof_readback_with_0x55 = false;
        EXPECT_TRUE(arm.configureMotorTimeout(2000, 100), "RID::TIMEOUT (2000) written and verified across all 7 motors");
        auto rb = arm.queryMotorTimeouts(100);
        EXPECT_TRUE(rb.size() == 7 && rb[0].has_value() && *rb[0] == 2000 && rb[6].has_value() && *rb[6] == 2000,
                    "queryMotorTimeouts confirms RID::TIMEOUT == 2000 on joints and gripper");

        EXPECT_TRUE(arm.refreshUntilAllFresh(50), "7-motor (6 joints + gripper) refreshUntilAllFresh succeeds");
        EXPECT_TRUE(arm.getState().has_gripper && arm.getState().gripper.online, "Gripper reported online");

        arm.enable();
        EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "7-motor arm+gripper armed");
        EXPECT_TRUE(arm.getState().gripper.enabled, "Gripper confirmed enabled");

        const unsigned q_before = queries.load();
        std::array<MITParam, NUM_JOINTS> arm_cmds;
        arm_cmds.fill(MITParam{10.0, 0.5, 0.0, 0.0, 0.0});
        MITParam grip_cmd{5.0, 0.1, 0.0, 0.0, 0.0};

        // Feed periodic MIT commands for 200ms (at 50Hz / 20ms period): no redundant 0xCC queries should occur
        for (int k = 0; k < 10; ++k) {
            arm.sendMITAll(arm_cmds, grip_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Periodic 7-axis sendMITAll keeps arm ARMED");
        EXPECT_TRUE(queries.load() == q_before, "No redundant 0xCC queries emitted when command_timeout_ms > 0");
        EXPECT_TRUE(motions.load() >= 70, "All 7 motor MIT frames sent each cycle");

        // Now freeze caller (no commands sent for > command_timeout_ms): watchdog MUST trigger FAULT & disable!
        const unsigned disables_before = disables.load();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (arm.status() != ArmStatus::FAULT && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        EXPECT_TRUE(arm.status() == ArmStatus::FAULT, "Command watchdog autonomously enters FAULT on host stall");
        EXPECT_TRUE(disables.load() >= disables_before + 7, "Watchdog sent disable_all to all 7 motors on stall");
        EXPECT_TRUE(queries.load() == q_before, "Watchdog did NOT feed motor via 0xCC during stall");

        arm.clearFault();
        EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Recovered from command watchdog FAULT cleanly");
    } catch (...) {
        running = false;
        simulator.join();
        ::close(fd);
        throw;
    }

    running = false;
    simulator.join();
    ::close(fd);
}

void testSingleMotorActivePrefix(ControlMode mode) {
    std::cout << "[TEST] Single DM3507, receive ID zero, mode "
              << static_cast<int>(mode) << "..." << std::endl;
    const int fd = openRawVcanSocket();
    if (fd < 0) throw std::runtime_error("Cannot open simulator socket");
    std::atomic<bool> running{true}, inject_fault{false};
    std::atomic<unsigned> disables{0};
    const uint32_t motion_id = mode == ControlMode::MIT ? 1 :
                               mode == ControlMode::POS_VEL ? 0x101 : 0x201;
    std::thread simulator([&] {
        bool enabled = false;
        ControlMode motor_mode = ControlMode::MIT;
        while (running.load()) {
            pollfd pfd{fd, POLLIN, 0};
            if (::poll(&pfd, 1, 2) <= 0) continue;
            can_frame request{};
            if (::read(fd, &request, sizeof(request)) != sizeof(request)) continue;
            if (request.can_id == 0x7FF && request.can_dlc == 8 &&
                request.data[0] == 1 && request.data[1] == 0 &&
                (request.data[2] == 0x55 || request.data[2] == 0x33) && request.data[3] == 10) {
                if (request.data[2] == 0x55) {
                    motor_mode = static_cast<ControlMode>(request.data[4]);
                    // Real USB-CAN ordering: queued state precedes a delayed mode ACK.
                    const can_frame queued{0, 8, 0, 0, 0, {1, 0x80, 0, 0x80, 8, 0, 25, 25}};
                    if (::write(fd, &queued, sizeof(queued)) != sizeof(queued)) running = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                can_frame reply = request;
                reply.can_id = 0;
                const auto value = static_cast<uint32_t>(motor_mode);
                std::memcpy(reply.data + 4, &value, sizeof(value));
                if (::write(fd, &reply, sizeof(reply)) != sizeof(reply)) running = false;
                continue;
            }
            const bool query = request.can_id == 0x7FF && request.data[2] == 0xCC;
            if (query) {
                if ((request.data[0] | (request.data[1] << 8)) != 1) {
                    continue;
                }
            } else if (request.can_id == 1 && request.can_dlc == 8 &&
                       request.data[7] == 0xFC) {
                enabled = true;
            } else if (request.can_id == 1 && request.can_dlc == 8 &&
                       request.data[7] == 0xFD) {
                enabled = false;
                ++disables;
            } else if (request.can_id != motion_id || motor_mode != mode) {
                continue;
            }
            can_frame reply{};
            reply.can_id = 0;
            reply.can_dlc = 8;
            const unsigned code = inject_fault.load() ? 9 : (enabled ? 1 : 0);
            reply.data[0] = static_cast<uint8_t>((code << 4) | 1);
            // Midpoint protocol values represent near-zero DM3507 feedback.
            reply.data[1] = 0x80;
            reply.data[3] = 0x80;
            reply.data[4] = 0x08;
            reply.data[6] = reply.data[7] = 25;
            if (::write(fd, &reply, sizeof(reply)) != sizeof(reply)) running = false;
        }
    });
    try {
        auto cfg = ArmConfig::defaultJ4310SixAxis();
        cfg.joint_count = 1;
        cfg.joints[0].motor_type = MotorType::DM3507;
        cfg.joints[0].recv_can_id = 0;
        cfg.joints[0].control_mode = mode;
        cfg.joints[0].pos_min = -12.566;
        cfg.joints[0].pos_max = 12.566;
        cfg.joints[0].vel_max = 50;
        cfg.joints[0].tau_max = 5;
        cfg.state_timeout_ms = 200;
        // Deliberately unusable inactive entries must never affect the live motor.
        for (std::size_t i = 1; i < NUM_JOINTS; ++i) cfg.joints[i] = cfg.joints[0];
        cfg.validate();
        Arm arm(cfg);
        arm.init();
        EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Single motor initializes disarmed");
        EXPECT_TRUE(arm.refreshUntilAllFresh(100), "Only active motor must respond to refresh");
        const auto state = arm.refresh(5000);
        EXPECT_TRUE(state.all_online && !state.any_fault && state.joints[0].online &&
                    state.joints[0].recv_can_id == 0, "CAN ID zero feedback reaches active joint");
        EXPECT_TRUE(arm.checkHealth(), "Five absent inactive motors do not fail health");
        arm.enable();
        EXPECT_TRUE(arm.status() == ArmStatus::ARMED, "Single configured motor enables");
        EXPECT_TRUE(arm.getState().joints[0].malformed_frames == 0 &&
                    std::abs(arm.getState().joints[0].position) < 0.001,
                    "Queued state and delayed mode ACK do not corrupt feedback during enable");
        const MITParam mit{1.0, 0.1, 0.0, 0.0, 0.0};
        const PosVelParam pos{0.0, 0.1};
        const VelParam vel{0.1};
        EXPECT_TRUE(!arm.validateCommand(1, mit) && !arm.validateCommand(1, pos) &&
                    !arm.validateCommand(1, vel), "All validators reject inactive indices");
        EXPECT_THROW(arm.sendMIT(1, mit), std::invalid_argument, "MIT rejects inactive index");
        EXPECT_THROW(arm.sendPosVel(1, pos), std::invalid_argument, "POS_VEL rejects inactive index");
        EXPECT_THROW(arm.sendVel(1, vel), std::invalid_argument, "VEL rejects inactive index");
        if (mode == ControlMode::MIT) {
            std::array<MITParam, NUM_JOINTS> cmds;
            cmds.fill(MITParam{-1.0, -1.0, 1000.0, 1000.0, 1000.0});
            cmds[0] = mit;
            EXPECT_TRUE(arm.validateCommand(cmds), "MIT array ignores invalid inactive commands");
            arm.sendMIT(0, mit);
            arm.sendMITAll(cmds);
        } else if (mode == ControlMode::POS_VEL) {
            std::array<PosVelParam, NUM_JOINTS> cmds;
            cmds.fill(PosVelParam{1000.0, 1000.0});
            cmds[0] = pos;
            EXPECT_TRUE(arm.validateCommand(cmds), "POS_VEL array ignores inactive commands");
            arm.sendPosVel(0, pos);
            arm.sendPosVelAll(cmds);
        } else {
            std::array<VelParam, NUM_JOINTS> cmds;
            cmds.fill(VelParam{1000.0});
            cmds[0] = vel;
            EXPECT_TRUE(arm.validateCommand(cmds), "VEL array ignores inactive commands");
            arm.sendVel(0, vel);
            arm.sendVelAll(cmds);
        }
        arm.disable();
        EXPECT_TRUE(arm.status() == ArmStatus::DISARMED && !arm.getState().joints[0].enabled,
                    "Single motor disable requires confirmed feedback");
        EXPECT_TRUE(arm.refreshUntilAllFresh(100), "Single motor refreshes after disable");
        arm.enable();
        const auto before = disables.load();
        inject_fault = true;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while ((arm.status() != ArmStatus::FAULT || disables.load() == before) &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        EXPECT_TRUE(arm.status() == ArmStatus::FAULT && disables.load() > before,
                    "Active hardware fault autonomously disables the motor");
        inject_fault = false;
        arm.clearFault();
        EXPECT_TRUE(arm.status() == ArmStatus::DISARMED, "Single motor fault recovery confirmed");
    } catch (...) {
        running = false;
        simulator.join();
        ::close(fd);
        throw;
    }
    running = false;
    simulator.join();
    ::close(fd);
}

} // namespace

int main() {
    testArmConfigAndYaml();
    testStrictEnableAndReinitSafety();
    testStrictFaultRecoveryAndBatchCommandFailures();
    testGripperAndCommandWatchdog();
    testSingleMotorActivePrefix(ControlMode::MIT);
    testSingleMotorActivePrefix(ControlMode::POS_VEL);
    testSingleMotorActivePrefix(ControlMode::VEL);

    std::cout << "\n========================================" << std::endl;
    std::cout << "Arm Tests Passed: " << g_passed << " | Failed: " << g_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_failed == 0) ? 0 : 1;
}
