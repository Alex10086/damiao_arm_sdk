#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace damiao {

struct CanFrame {
    uint32_t can_id{0}; // 11-bit standard CAN ID (0x000 ~ 0x7FF) or 29-bit extended
    uint8_t dlc{8};     // Data length code (0 ~ 8)
    std::array<uint8_t, 8> data{0, 0, 0, 0, 0, 0, 0, 0};
    bool is_extended{false};
    bool is_rtr{false};
    bool is_error{false};
};

class CanException : public std::runtime_error {
public:
    explicit CanException(const std::string& msg) : std::runtime_error(msg) {}
};

struct UsbTransportStats {
    uint64_t tx_packets{0};
    uint64_t tx_partial_retries{0};
    uint64_t rx_frames{0};
    uint64_t rx_non_data_packets{0};
    uint64_t rx_malformed_frames{0};
    uint64_t rx_discarded_bytes{0};
};

// Direct USB-CAN Transport for Damiao official USB-to-CAN adapter (/dev/ttyACM*)
// Default baud rate for Damiao CDC virtual serial port is 921600.
class DamiaoUsbCanTransport {
public:
    DamiaoUsbCanTransport() = default;
    explicit DamiaoUsbCanTransport(const std::string& port_path, int baudrate = 921600);
    ~DamiaoUsbCanTransport();

    DamiaoUsbCanTransport(const DamiaoUsbCanTransport&) = delete;
    DamiaoUsbCanTransport& operator=(const DamiaoUsbCanTransport&) = delete;

    void open(const std::string& port_path);
    void close() noexcept;
    bool isOpen() const noexcept;

    void send(const CanFrame& frame);
    bool receive(CanFrame& frame, int timeout_ms = 10);

    std::string interfaceName() const;
    int baudrate() const noexcept { return baudrate_; }
    void setBaudrate(int baudrate) noexcept { baudrate_ = baudrate; }

    UsbTransportStats getStats() const noexcept;
    void resetStats() noexcept;

private:
    void closeUnlocked() noexcept;

    std::atomic<int> fd_{-1};
    std::string port_path_{};
    int baudrate_{921600};
    mutable std::mutex tx_mutex_;
    mutable std::mutex rx_mutex_;
    std::vector<uint8_t> rx_buf_{};

    std::atomic<uint64_t> tx_packets_{0};
    std::atomic<uint64_t> tx_partial_retries_{0};
    std::atomic<uint64_t> rx_frames_{0};
    std::atomic<uint64_t> rx_non_data_packets_{0};
    std::atomic<uint64_t> rx_malformed_frames_{0};
    std::atomic<uint64_t> rx_discarded_bytes_{0};
};

} // namespace damiao
