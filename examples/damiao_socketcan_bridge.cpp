#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/damiao_usb_transport.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
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
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    std::string socketcan_if = "vcan0";
    std::string serial_port = "/dev/ttyACM0";
    int baudrate = 921600;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--socketcan" && i + 1 < argc) {
            socketcan_if = argv[++i];
        } else if (arg == "--serial" && i + 1 < argc) {
            serial_port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baudrate = std::stoi(argv[++i]);
        }
    }

    std::cout << "=== Damiao USB-CAN <-> Linux SocketCAN Bridge ===" << std::endl;
    std::cout << "SocketCAN Interface: " << socketcan_if << std::endl;
    std::cout << "USB-CAN Serial Port: " << serial_port << " @" << baudrate << std::endl;

    auto sock_can = std::make_shared<SocketCanTransport>(socketcan_if, true);
    auto usb_can = std::make_shared<DamiaoUsbCanTransport>(serial_port, baudrate);

    std::atomic<uint64_t> tx_count{0};
    std::atomic<uint64_t> rx_count{0};

    // Thread 1: SocketCAN (e.g. vcan0) -> USB-CAN (/dev/ttyACM0)
    std::thread sock_to_usb([sock_can, usb_can, &tx_count]() {
        CanFrame frame;
        while (g_running) {
            if (sock_can->receive(frame, 10)) {
                try {
                    usb_can->send(frame);
                    tx_count++;
                } catch (const std::exception& e) {
                    std::cerr << "[Bridge] SocketCAN -> USB error: " << e.what() << std::endl;
                }
            }
        }
    });

    // Thread 2: USB-CAN (/dev/ttyACM0) -> SocketCAN (vcan0)
    std::thread usb_to_sock([sock_can, usb_can, &rx_count]() {
        CanFrame frame;
        while (g_running) {
            if (usb_can->receive(frame, 10)) {
                try {
                    sock_can->send(frame);
                    rx_count++;
                } catch (const std::exception& e) {
                    std::cerr << "[Bridge] USB -> SocketCAN error: " << e.what() << std::endl;
                }
            }
        }
    });

    std::cout << "Bridge running. Press Ctrl+C to terminate." << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (sock_to_usb.joinable()) sock_to_usb.join();
    if (usb_to_sock.joinable()) usb_to_sock.join();

    sock_can->close();
    usb_can->close();

    std::cout << "Bridge stopped. Total forwarded: SocketCAN->USB: " << tx_count
              << ", USB->SocketCAN: " << rx_count << std::endl;
    return 0;
}
