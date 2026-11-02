#pragma once

#include "damiao_arm_sdk/can_transport.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace damiao {

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

private:
    std::atomic<int> fd_{-1};
    std::string port_path_{};
    int baudrate_{921600};
    mutable std::mutex tx_mutex_;
    mutable std::mutex rx_mutex_;
    std::vector<uint8_t> rx_buf_{};
};

} // namespace damiao
