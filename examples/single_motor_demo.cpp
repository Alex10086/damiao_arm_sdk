#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/damiao_usb_transport.hpp"
#include "damiao_arm_sdk/motor_driver.hpp"
#include "damiao_arm_sdk/types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace damiao;

namespace {
std::atomic<bool> g_running{true};

void sigHandler(int) {
    g_running = false;
}

void printState(const MotorState& st) {
    std::cout << std::fixed << std::setprecision(3)
              << "[Feedback] valid=" << (st.valid ? "yes" : "no")
              << " | id=0x" << std::hex << static_cast<int>(st.slave_id) << std::dec
              << " | status=" << toString(st.error)
              << " | pos=" << st.position << " rad (" << (st.position * 180.0f / 3.14159265f) << " deg)"
              << " | vel=" << st.velocity << " rad/s"
              << " | tau=" << st.torque << " Nm"
              << " | T_mos=" << static_cast<int>(st.temp_mos) << "C"
              << " | T_rotor=" << static_cast<int>(st.temp_rotor) << "C"
              << " | rx=" << st.rx_count
              << std::endl;
}

// Safe passive status refresh frame (CAN ID = 0x7FF, byte[0]=can_id, byte[2]=0xCC)
void requestStatusRefresh(ICanTransport& transport, uint16_t can_id) {
    CanFrame f;
    f.can_id = 0x7FF;
    f.dlc = 8;
    f.data = {static_cast<uint8_t>(can_id & 0xFF),
              static_cast<uint8_t>((can_id >> 8) & 0xFF),
              0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
    transport.send(f);
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    std::string transport_type = "socketcan"; // "socketcan" or "serial"
    std::string device_name = "vcan0";
    uint16_t can_id = 0x01;
    uint16_t master_id = 0x11;
    // P0 Safety Fix: Default is read-only probe (NO motor enable unless --enable or --hold is explicitly passed)
    bool enable_drive = false;
    bool hold_current_pos = false;
    bool continuous_monitor = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            device_name = argv[++i];
            transport_type = "socketcan";
        } else if ((arg == "--serial" || arg == "--usb") && i + 1 < argc) {
            device_name = argv[++i];
            transport_type = "serial";
        } else if (arg == "--can-id" && i + 1 < argc) {
            can_id = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--master-id" && i + 1 < argc) {
            master_id = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--monitor") {
            continuous_monitor = true;
        } else if (arg == "--probe") {
            enable_drive = false;
        } else if (arg == "--enable") {
            enable_drive = true;
        } else if (arg == "--hold") {
            enable_drive = true;
            hold_current_pos = true;
        }
    }

    std::cout << "=== Damiao J4310 (24V) Motor Controller ===" << std::endl;
    std::cout << "Transport: " << transport_type << " (" << device_name << ")"
              << " | CAN ID: 0x" << std::hex << can_id
              << " | Master ID: 0x" << master_id << std::dec
              << " | Mode: " << (enable_drive ? (hold_current_pos ? "ENABLE+HOLD" : "ENABLE+DAMPING") : "READ-ONLY PROBE (Safe Default)")
              << std::endl;

    std::shared_ptr<ICanTransport> transport;
    if (transport_type == "serial") {
        transport = std::make_shared<DamiaoUsbCanTransport>(device_name, 921600);
    } else {
        transport = std::make_shared<SocketCanTransport>(device_name, true);
    }

    MotorConfig config = MotorConfig::J4310(can_id, master_id, ControlMode::MIT);
    MotorDriver motor(transport, config);

    // 1. Send passive status query
    std::cout << "\n[1] Querying motor status (passive 0x7FF refresh frame, NO motion)..." << std::endl;
    requestStatusRefresh(*transport, can_id);
    for (int i = 0; i < 10; ++i) {
        if (motor.pollFeedback(50)) break;
    }

    MotorState initial_state = motor.getState();
    if (!initial_state.valid) {
        std::cerr << "[ERROR] No response received from motor! Please check wiring, power (24V), and CAN IDs." << std::endl;
        return 1;
    }
    printState(initial_state);

    if (!enable_drive) {
        if (continuous_monitor) {
            std::cout << "\n[Monitor Mode] Continuous passive polling (Ctrl+C to quit)..." << std::endl;
            while (g_running) {
                requestStatusRefresh(*transport, can_id);
                if (motor.pollFeedback(50)) {
                    printState(motor.getState());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        } else {
            std::cout << "\n[Safe Default] Read-only probe finished. Pass --enable or --hold to actively enable motor." << std::endl;
        }
        return 0;
    }

    // 2. Explicit active drive test (--enable or --hold required)
    float initial_pos = initial_state.position;
    std::cout << "\n[2] Initial position: " << initial_pos << " rad ("
              << (initial_pos * 180.0f / 3.14159265f) << " deg)" << std::endl;

    std::cout << "[3] Enabling motor..." << std::endl;
    motor.enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motor.pollFeedback(50);
    printState(motor.getState());

    if (hold_current_pos) {
        std::cout << "\n[4] Safe Position Hold: holding current position " << initial_pos
                  << " rad with gentle stiffness (Kp=5.0, Kd=0.5)..." << std::endl;
        for (int i = 0; i < 50 && g_running; ++i) {
            motor.setMIT(initial_pos, 0.0f, 5.0f, 0.5f, 0.0f);
            motor.pollFeedback(20);
            if (i % 10 == 0) printState(motor.getState());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } else {
        std::cout << "\n[4] Safe Damping Mode: zero torque, gentle damping (Kp=0.0, Kd=0.5)..." << std::endl;
        for (int i = 0; i < 50 && g_running; ++i) {
            motor.setMIT(0.0f, 0.0f, 0.0f, 0.5f, 0.0f);
            motor.pollFeedback(20);
            if (i % 10 == 0) printState(motor.getState());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::cout << "\n[5] Disabling motor..." << std::endl;
    motor.disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motor.pollFeedback(50);
    printState(motor.getState());

    std::cout << "\n[Done] Motor safely disabled." << std::endl;
    return 0;
}
