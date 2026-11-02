#include "damiao_arm_sdk/can_transport.hpp"
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
              << " | pos=" << st.position << " rad"
              << " | vel=" << st.velocity << " rad/s"
              << " | tau=" << st.torque << " Nm"
              << " | T_mos=" << static_cast<int>(st.temp_mos) << "C"
              << " | T_rotor=" << static_cast<int>(st.temp_rotor) << "C"
              << " | rx=" << st.rx_count
              << std::endl;
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);

    std::string can_if = "vcan0";
    uint16_t can_id = 0x01;
    uint16_t master_id = 0x11;
    bool simulate_responder = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            can_if = argv[++i];
        } else if (arg == "--can-id" && i + 1 < argc) {
            can_id = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--master-id" && i + 1 < argc) {
            master_id = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--real-hw") {
            simulate_responder = false;
        }
    }

    std::cout << "=== Damiao J4310 (24V) Single Motor Demo ===" << std::endl;
    std::cout << "Interface: " << can_if
              << " | CAN ID: 0x" << std::hex << can_id
              << " | Master ID: 0x" << master_id << std::dec
              << " | Simulated Responder: " << (simulate_responder ? "ON" : "OFF") << std::endl;

    auto transport = std::make_shared<SocketCanTransport>(can_if, true);
    MotorConfig config = MotorConfig::J4310(can_id, master_id, ControlMode::MIT);
    MotorDriver motor(transport, config);

    // Optional virtual motor responder thread when testing on vcan0
    std::thread sim_thread;
    if (simulate_responder) {
        sim_thread = std::thread([can_if, config]() {
            SocketCanTransport sim_can(can_if, true);
            bool enabled = false;
            float sim_pos = 0.0f;
            float sim_vel = 0.0f;
            float sim_tau = 0.0f;

            while (g_running) {
                CanFrame rx;
                if (!sim_can.receive(rx, 20)) {
                    continue;
                }
                if (rx.can_id != config.can_id &&
                    rx.can_id != (0x100u + config.can_id) &&
                    rx.can_id != (0x200u + config.can_id)) {
                    continue;
                }
                if (rx.dlc == 8 && rx.data[0] == 0xFF && rx.data[6] == 0xFF) {
                    if (rx.data[7] == 0xFC) enabled = true;
                    else if (rx.data[7] == 0xFD) enabled = false;
                } else if (rx.can_id == config.can_id && rx.dlc == 8) {
                    uint16_t p_int = (static_cast<uint16_t>(rx.data[0]) << 8) | rx.data[1];
                    uint16_t v_int = (static_cast<uint16_t>(rx.data[2]) << 4) | (rx.data[3] >> 4);
                    uint16_t t_int = (static_cast<uint16_t>(rx.data[6] & 0x0F) << 8) | rx.data[7];
                    sim_pos = protocol::uintToFloat(p_int, -config.limits.p_max, config.limits.p_max, 16);
                    sim_vel = protocol::uintToFloat(v_int, -config.limits.v_max, config.limits.v_max, 12);
                    sim_tau = protocol::uintToFloat(t_int, -config.limits.t_max, config.limits.t_max, 12);
                }

                // Reply with feedback frame on master_id
                CanFrame fb;
                fb.can_id = config.master_id;
                fb.dlc = 8;
                uint8_t status = enabled ? static_cast<uint8_t>(MotorError::ENABLED)
                                         : static_cast<uint8_t>(MotorError::DISABLED);
                uint16_t p_u = protocol::floatToUint(sim_pos, -config.limits.p_max, config.limits.p_max, 16);
                uint16_t v_u = protocol::floatToUint(sim_vel, -config.limits.v_max, config.limits.v_max, 12);
                uint16_t t_u = protocol::floatToUint(sim_tau, -config.limits.t_max, config.limits.t_max, 12);

                fb.data[0] = static_cast<uint8_t>((status << 4) | (config.can_id & 0x0F));
                fb.data[1] = static_cast<uint8_t>((p_u >> 8) & 0xFF);
                fb.data[2] = static_cast<uint8_t>(p_u & 0xFF);
                fb.data[3] = static_cast<uint8_t>((v_u >> 4) & 0xFF);
                fb.data[4] = static_cast<uint8_t>(((v_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F));
                fb.data[5] = static_cast<uint8_t>(t_u & 0xFF);
                fb.data[6] = 29;
                fb.data[7] = 31;
                sim_can.send(fb);
            }
        });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    std::cout << "\n1) Enabling motor..." << std::endl;
    motor.enable();
    motor.pollFeedback(50);
    printState(motor.getState());

    std::cout << "\n2) Sending MIT control commands (pos=0.5 rad, vel=1.0 rad/s, kp=20, kd=1.0, tau=0.2 Nm)..." << std::endl;
    motor.setMIT(0.5f, 1.0f, 20.0f, 1.0f, 0.2f);
    motor.pollFeedback(50);
    printState(motor.getState());

    std::cout << "\n3) Disabling motor..." << std::endl;
    motor.disable();
    motor.pollFeedback(50);
    printState(motor.getState());

    g_running = false;
    if (sim_thread.joinable()) {
        sim_thread.join();
    }
    transport->close();
    return 0;
}
