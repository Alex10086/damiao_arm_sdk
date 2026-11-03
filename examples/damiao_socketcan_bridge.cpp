#include "damiao_arm_sdk/socketcan_bridge.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

using namespace damiao;

namespace {
std::atomic<bool> g_running{true};

void sigHandler(int) {
    g_running.store(false, std::memory_order_release);
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    BridgeConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--socketcan" && i + 1 < argc) {
            cfg.socketcan_if = argv[++i];
        } else if (arg == "--serial" && i + 1 < argc) {
            cfg.serial_port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            cfg.baudrate = std::stoi(argv[++i]);
        } else if (arg == "--no-reconnect") {
            cfg.auto_reconnect = false;
        } else if (arg == "--read-only" || arg == "--readonly") {
            cfg.read_only = true;
        } else if (arg == "--verbose" || arg == "-v") {
            cfg.verbose = true;
        }
    }

    std::cout << "=== Damiao USB-CAN <-> Linux SocketCAN Bridge ===" << std::endl;
    std::cout << "SocketCAN Interface: " << cfg.socketcan_if << std::endl;
    std::cout << "USB-CAN Serial Port: " << cfg.serial_port << " @" << cfg.baudrate << std::endl;
    std::cout << "Read-Only Filter   : " << (cfg.read_only ? "ENABLED (blocks motion/enable/write_param)" : "disabled") << std::endl;
    std::cout << "Auto Reconnect     : " << (cfg.auto_reconnect ? "enabled" : "disabled") << std::endl;

    try {
        SocketCanUsbBridge bridge(cfg);
        bridge.start();
        std::cout << "Bridge running. Press Ctrl+C to terminate." << std::endl;

        while (g_running.load(std::memory_order_acquire) && bridge.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        bridge.stop();
        auto st = bridge.getStats();
        std::cout << "Bridge stopped. Forwarded SocketCAN->USB: " << st.sock_to_usb_forwarded
                  << ", USB->SocketCAN: " << st.usb_to_sock_forwarded
                  << " | ReadOnly Blocked: " << st.sock_readonly_blocked
                  << " | Dropped(sock/usb): " << st.sock_rx_dropped << "/" << st.usb_rx_dropped
                  << " | Errors(sock/usb): " << st.sock_errors << "/" << st.usb_errors
                  << " | Reconnects: " << st.usb_reconnects << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Bridge Fatal] " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
