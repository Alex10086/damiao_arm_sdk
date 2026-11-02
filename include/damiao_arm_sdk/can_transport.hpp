#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace damiao {

struct CanFrame {
    uint32_t can_id{0}; // 11-bit standard CAN ID (0x000 ~ 0x7FF)
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

class ICanTransport {
public:
    virtual ~ICanTransport() = default;

    virtual void open(const std::string& interface_name) = 0;
    virtual void close() noexcept = 0;
    virtual bool isOpen() const noexcept = 0;

    virtual void send(const CanFrame& frame) = 0;
    // Returns true if a frame was read within timeout_ms; false on timeout (0 = non-blocking poll).
    // Throws CanException on transport error.
    virtual bool receive(CanFrame& frame, int timeout_ms = 10) = 0;

    virtual std::string interfaceName() const = 0;
};

class SocketCanTransport : public ICanTransport {
public:
    SocketCanTransport() = default;
    explicit SocketCanTransport(const std::string& interface_name, bool loopback = true);
    ~SocketCanTransport() override;

    SocketCanTransport(const SocketCanTransport&) = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;

    void open(const std::string& interface_name) override;
    void close() noexcept override;
    bool isOpen() const noexcept override;

    void send(const CanFrame& frame) override;
    bool receive(CanFrame& frame, int timeout_ms = 10) override;

    std::string interfaceName() const override;

    // Enable/disable local socket loopback & receive_own_msgs (useful for vcan0 self-testing)
    void setLoopback(bool enable, bool recv_own_msgs = false);

private:
    int sock_fd_{-1};
    std::string ifname_{};
    bool loopback_{true};
    bool recv_own_msgs_{false};
    mutable std::mutex mutex_;
};

} // namespace damiao
