#pragma once

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
    // P0 Safety: Default read_only is true! SocketCAN -> USB-CAN only allows
    // passive Damiao status refresh (0x7FF, D[2]=0xCC) and parameter query (0x7FF, D[2]=0x33).
    // Explicit authorization (read_only = false / --allow-motion) is required to forward motion/enable frames.
    bool read_only{true};
    bool verbose{false};
};

struct BridgeStats {
    uint64_t sock_to_usb_forwarded{0};
    uint64_t usb_to_sock_forwarded{0};
    uint64_t sock_rx_dropped{0};
    uint64_t sock_readonly_blocked{0};
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

    void start();
    void stop() noexcept;

    bool isRunning() const noexcept { return running_.load(std::memory_order_acquire); }
    BridgeStats getStats() const noexcept;

    void setReadOnly(bool read_only) noexcept { read_only_.store(read_only, std::memory_order_release); }
    bool isReadOnly() const noexcept { return read_only_.load(std::memory_order_acquire); }

    static bool isValidForwardFrame(const CanFrame& frame,
                                    bool allow_extended = false,
                                    bool allow_rtr = false) noexcept;

    // P0: Returns true ONLY if the frame is a safe read-only query frame:
    // - Standard 11-bit Data frame (not extended, not RTR, not error)
    // - CAN ID == 0x7FF, DLC == 8
    // - data[2] == 0xCC (status refresh) OR data[2] == 0x33 (parameter read)
    // - data[4..7] == 0x00
    static bool isReadOnlyAllowedTxFrame(const CanFrame& frame) noexcept;

private:
    void openSocketCan();
    void closeSocketCan() noexcept;
    void sendSocketCan(const CanFrame& frame);
    bool receiveSocketCan(CanFrame& frame, int timeout_ms);

    void sockToUsbLoop();
    void usbToSockLoop();
    bool tryReconnectUsb();

    BridgeConfig config_;
    std::atomic<bool> read_only_{true};

    int sock_fd_{-1};
    mutable std::mutex sock_mutex_;

    std::shared_ptr<DamiaoUsbCanTransport> usb_can_;
    mutable std::mutex usb_reconnect_mutex_;

    std::atomic<bool> running_{false};
    std::thread sock_to_usb_thread_;
    std::thread usb_to_sock_thread_;

    std::atomic<uint64_t> sock_to_usb_forwarded_{0};
    std::atomic<uint64_t> usb_to_sock_forwarded_{0};
    std::atomic<uint64_t> sock_rx_dropped_{0};
    std::atomic<uint64_t> sock_readonly_blocked_{0};
    std::atomic<uint64_t> usb_rx_dropped_{0};
    std::atomic<uint64_t> sock_errors_{0};
    std::atomic<uint64_t> usb_errors_{0};
    std::atomic<uint64_t> usb_reconnects_{0};
};

} // namespace damiao
