#pragma once

#include "damiao_arm_sdk/can_transport.hpp"
#include "damiao_arm_sdk/damiao_usb_transport.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace damiao {

struct BridgeConfig {
    std::string socketcan_if{"vcan0"};
    std::string serial_port{"/dev/ttyACM0"};
    int baudrate{921600};
    bool auto_reconnect{true};
    int reconnect_interval_ms{200};
    bool allow_extended_frames{false};
    bool allow_rtr_frames{false};
    bool verbose{false};
};

struct BridgeStats {
    uint64_t sock_to_usb_forwarded{0};
    uint64_t usb_to_sock_forwarded{0};
    uint64_t sock_rx_dropped{0};
    uint64_t usb_rx_dropped{0};
    uint64_t sock_errors{0};
    uint64_t usb_errors{0};
    uint64_t usb_reconnects{0};
    bool usb_connected{false};
    bool sock_connected{false};
    UsbTransportStats usb_transport_stats{};
};

class SocketCanUsbBridge {
public:
    explicit SocketCanUsbBridge(const BridgeConfig& config);
    ~SocketCanUsbBridge();

    SocketCanUsbBridge(const SocketCanUsbBridge&) = delete;
    SocketCanUsbBridge& operator=(const SocketCanUsbBridge&) = delete;

    // Start forwarding threads. Throws CanException if initial connection fails
    // (unless auto_reconnect is enabled for USB, though SocketCAN must exist).
    void start();

    // Cleanly stop worker threads and close transports.
    void stop() noexcept;

    bool isRunning() const noexcept { return running_.load(std::memory_order_acquire); }
    BridgeStats getStats() const noexcept;

    static bool isValidForwardFrame(const CanFrame& frame,
                                    bool allow_extended = false,
                                    bool allow_rtr = false) noexcept;

private:
    void sockToUsbLoop();
    void usbToSockLoop();
    bool tryReconnectUsb();

    BridgeConfig config_;
    std::shared_ptr<SocketCanTransport> sock_can_;
    std::shared_ptr<DamiaoUsbCanTransport> usb_can_;
    mutable std::mutex usb_reconnect_mutex_;

    std::atomic<bool> running_{false};
    std::thread sock_to_usb_thread_;
    std::thread usb_to_sock_thread_;

    std::atomic<uint64_t> sock_to_usb_forwarded_{0};
    std::atomic<uint64_t> usb_to_sock_forwarded_{0};
    std::atomic<uint64_t> sock_rx_dropped_{0};
    std::atomic<uint64_t> usb_rx_dropped_{0};
    std::atomic<uint64_t> sock_errors_{0};
    std::atomic<uint64_t> usb_errors_{0};
    std::atomic<uint64_t> usb_reconnects_{0};
};

} // namespace damiao
