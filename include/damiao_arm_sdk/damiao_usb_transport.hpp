#pragma once

#include "damiao_arm_sdk/can_transport.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace damiao {

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
class DamiaoUsbCanTransport : public ICanTransport {
public:
    DamiaoUsbCanTransport() = default;
    explicit DamiaoUsbCanTransport(const std::string& port_path, int baudrate = 921600);
    ~DamiaoUsbCanTransport() override;

    DamiaoUsbCanTransport(const DamiaoUsbCanTransport&) = delete;
    DamiaoUsbCanTransport& operator=(const DamiaoUsbCanTransport&) = delete;

    void open(const std::string& port_path) override;
    void close() noexcept override;
    bool isOpen() const noexcept override;

    void send(const CanFrame& frame) override;
    bool receive(CanFrame& frame, int timeout_ms = 10) override;

    std::string interfaceName() const override;
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
