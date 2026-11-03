#include "damiao_arm_sdk/damiao_usb_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace damiao {

#pragma pack(push, 1)
struct DamiaoUsbTxPacket {
    uint8_t header[2]{0x55, 0xAA};
    uint8_t frame_len{0x1E}; // 30 bytes
    uint8_t cmd{0x03};       // 0x03: send without ACK
    uint32_t send_times{1};
    uint32_t interval_us{10};
    uint8_t id_type{0};      // 0: standard 11-bit, 1: extended 29-bit
    uint32_t can_id{0};
    uint8_t frame_type{0};   // 0: data frame, 1: RTR
    uint8_t dlc{8};
    uint8_t id_acc{0};
    uint8_t data_acc{0};
    uint8_t data[8]{0};
    uint8_t crc{0};
};
#pragma pack(pop)
static_assert(sizeof(DamiaoUsbTxPacket) == 30, "DamiaoUsbTxPacket must be exactly 30 bytes");

DamiaoUsbCanTransport::DamiaoUsbCanTransport(const std::string& port_path, int baudrate)
    : baudrate_(baudrate) {
    open(port_path);
}

DamiaoUsbCanTransport::~DamiaoUsbCanTransport() {
    close();
}

void DamiaoUsbCanTransport::closeUnlocked() noexcept {
    int cur_fd = fd_.exchange(-1);
    if (cur_fd >= 0) {
        ::close(cur_fd);
    }
    rx_buf_.clear();
}

void DamiaoUsbCanTransport::open(const std::string& port_path) {
    std::lock_guard<std::mutex> lock_tx(tx_mutex_);
    std::lock_guard<std::mutex> lock_rx(rx_mutex_);

    if (port_path.empty()) {
        throw CanException("DamiaoUsbCanTransport::open: port_path cannot be empty");
    }
    closeUnlocked();

    int fd = ::open(port_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw CanException("DamiaoUsbCanTransport::open: open('" + port_path +
                           "') failed: " + std::string(std::strerror(errno)));
    }

    struct termios tty {};
    if (::tcgetattr(fd, &tty) != 0) {
        int err = errno;
        ::close(fd);
        throw CanException("DamiaoUsbCanTransport::open: tcgetattr failed: " +
                           std::string(std::strerror(err)));
    }

    speed_t speed = B921600;
    if (baudrate_ == 115200) speed = B115200;
    else if (baudrate_ == 460800) speed = B460800;
    else if (baudrate_ == 921600) speed = B921600;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable read
    tty.c_cflag &= ~(PARENB | PARODD);          // no parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no flow control

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;                      // raw output
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // raw input

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    ::tcflush(fd, TCIOFLUSH);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        int err = errno;
        ::close(fd);
        throw CanException("DamiaoUsbCanTransport::open: tcsetattr failed: " +
                           std::string(std::strerror(err)));
    }

    port_path_ = port_path;
    rx_buf_.clear();
    fd_.store(fd);
}

void DamiaoUsbCanTransport::close() noexcept {
    std::lock_guard<std::mutex> lock_tx(tx_mutex_);
    std::lock_guard<std::mutex> lock_rx(rx_mutex_);
    closeUnlocked();
}

bool DamiaoUsbCanTransport::isOpen() const noexcept {
    return fd_.load() >= 0;
}

std::string DamiaoUsbCanTransport::interfaceName() const {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    return port_path_;
}

UsbTransportStats DamiaoUsbCanTransport::getStats() const noexcept {
    UsbTransportStats st;
    st.tx_packets = tx_packets_.load(std::memory_order_relaxed);
    st.tx_partial_retries = tx_partial_retries_.load(std::memory_order_relaxed);
    st.rx_frames = rx_frames_.load(std::memory_order_relaxed);
    st.rx_non_data_packets = rx_non_data_packets_.load(std::memory_order_relaxed);
    st.rx_malformed_frames = rx_malformed_frames_.load(std::memory_order_relaxed);
    st.rx_discarded_bytes = rx_discarded_bytes_.load(std::memory_order_relaxed);
    return st;
}

void DamiaoUsbCanTransport::resetStats() noexcept {
    tx_packets_.store(0, std::memory_order_relaxed);
    tx_partial_retries_.store(0, std::memory_order_relaxed);
    rx_frames_.store(0, std::memory_order_relaxed);
    rx_non_data_packets_.store(0, std::memory_order_relaxed);
    rx_malformed_frames_.store(0, std::memory_order_relaxed);
    rx_discarded_bytes_.store(0, std::memory_order_relaxed);
}

void DamiaoUsbCanTransport::send(const CanFrame& frame) {
    if (frame.is_error) {
        throw std::invalid_argument("DamiaoUsbCanTransport::send: cannot transmit error frame");
    }
    if (frame.dlc > 8) {
        throw std::invalid_argument("DamiaoUsbCanTransport::send: invalid DLC (> 8)");
    }
    if (!frame.is_extended && frame.can_id > CAN_SFF_MASK) {
        throw std::invalid_argument("DamiaoUsbCanTransport::send: standard 11-bit CAN ID exceeds 0x7FF");
    }
    if (frame.is_extended && frame.can_id > CAN_EFF_MASK) {
        throw std::invalid_argument("DamiaoUsbCanTransport::send: extended 29-bit CAN ID exceeds 0x1FFFFFFF");
    }

    DamiaoUsbTxPacket pkt;
    pkt.id_type = frame.is_extended ? 1 : 0;
    pkt.can_id = frame.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    pkt.frame_type = frame.is_rtr ? 1 : 0;
    pkt.dlc = frame.dlc;
    std::memcpy(pkt.data, frame.data.data(), pkt.dlc);

    std::lock_guard<std::mutex> lock(tx_mutex_);
    int fd = fd_.load();
    if (fd < 0) {
        throw CanException("DamiaoUsbCanTransport::send: port is not open");
    }

    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&pkt);
    size_t remaining = sizeof(pkt);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

    while (remaining > 0) {
        ssize_t written = ::write(fd, ptr, remaining);
        if (written > 0) {
            if (static_cast<size_t>(written) < remaining) {
                tx_partial_retries_.fetch_add(1, std::memory_order_relaxed);
            }
            ptr += written;
            remaining -= static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        if (written == 0 || (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                throw CanException("DamiaoUsbCanTransport::send: write timeout after partial/blocked write");
            }
            int wait_ms = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
            if (wait_ms < 1) wait_ms = 1;

            struct pollfd pfd {};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            int ret = ::poll(&pfd, 1, wait_ms);
            if (ret < 0 && errno == EINTR) {
                continue;
            }
            if (ret <= 0) {
                throw CanException("DamiaoUsbCanTransport::send: poll(POLLOUT) timeout/failed");
            }
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                int saved_fd = fd_.exchange(-1);
                if (saved_fd >= 0) ::close(saved_fd);
                throw CanException("DamiaoUsbCanTransport::send: serial port disconnected (POLLHUP/POLLERR)");
            }
            tx_partial_retries_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // Fatal serial error (e.g. EIO, ENODEV, ENXIO, EBADF on USB unplug)
        int err = errno;
        int saved_fd = fd_.exchange(-1);
        if (saved_fd >= 0) {
            ::close(saved_fd);
        }
        throw CanException("DamiaoUsbCanTransport::send: write failed (port closed): " +
                           std::string(std::strerror(err)));
    }

    tx_packets_.fetch_add(1, std::memory_order_relaxed);
}

bool DamiaoUsbCanTransport::receive(CanFrame& frame, int timeout_ms) {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    int fd = fd_.load();
    if (fd < 0) {
        throw CanException("DamiaoUsbCanTransport::receive: port is not open");
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (true) {
        // Parse any complete 16-byte frames in rx_buf_
        while (rx_buf_.size() >= 16) {
            if (rx_buf_[0] == 0xAA && rx_buf_[15] == 0x55) {
                uint8_t cmd = rx_buf_[1];
                if (cmd == 0x11) { // 0x11 = valid CAN frame received from bus
                    uint8_t raw_dlc = rx_buf_[2] & 0x3F;
                    bool is_ext = (rx_buf_[2] & 0x40) != 0;
                    bool is_rtr = (rx_buf_[2] & 0x80) != 0;
                    uint32_t raw_can_id = 0;
                    std::memcpy(&raw_can_id, &rx_buf_[3], 4);

                    // Validate CAN frame fields before forwarding
                    bool valid_dlc = (raw_dlc <= 8);
                    bool valid_id = is_ext ? (raw_can_id <= CAN_EFF_MASK) : (raw_can_id <= CAN_SFF_MASK);

                    if (!valid_dlc || !valid_id) {
                        rx_malformed_frames_.fetch_add(1, std::memory_order_relaxed);
                        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 16);
                        continue;
                    }

                    frame.dlc = raw_dlc;
                    frame.is_extended = is_ext;
                    frame.is_rtr = is_rtr;
                    frame.is_error = false;
                    frame.can_id = raw_can_id;
                    frame.data.fill(0);
                    if (!is_rtr && raw_dlc > 0) {
                        std::memcpy(frame.data.data(), &rx_buf_[7], raw_dlc);
                    }

                    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 16);
                    rx_frames_.fetch_add(1, std::memory_order_relaxed);
                    return true;
                }
                // Non-0x11 packet (e.g. adapter internal ACK / status packet): consume and continue
                rx_non_data_packets_.fetch_add(1, std::memory_order_relaxed);
                rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 16);
                continue;
            }
            // Resynchronize to next 0xAA header byte
            auto next_aa = std::find(rx_buf_.begin() + 1, rx_buf_.end(), 0xAA);
            size_t dropped = static_cast<size_t>(std::distance(rx_buf_.begin(), next_aa));
            rx_discarded_bytes_.fetch_add(dropped, std::memory_order_relaxed);
            rx_buf_.erase(rx_buf_.begin(), next_aa);
        }

        // Prevent unbounded buffer growth if stream is corrupt
        if (rx_buf_.size() > 4096) {
            rx_discarded_bytes_.fetch_add(rx_buf_.size(), std::memory_order_relaxed);
            rx_buf_.clear();
        }

        int remaining_ms = 0;
        if (timeout_ms > 0) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return false;
            }
            remaining_ms = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
            if (remaining_ms < 0) remaining_ms = 0;
        }

        fd = fd_.load();
        if (fd < 0) {
            throw CanException("DamiaoUsbCanTransport::receive: port closed");
        }

        struct pollfd pfd {};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, remaining_ms);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            closeUnlocked();
            throw CanException("DamiaoUsbCanTransport::receive: poll failed: " +
                               std::string(std::strerror(errno)));
        }
        if (ret == 0) {
            return false; // Timeout
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            closeUnlocked();
            throw CanException("DamiaoUsbCanTransport::receive: serial device disconnected (POLLHUP/POLLERR)");
        }

        uint8_t tmp[256];
        ssize_t n = ::read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
        } else if (n == 0) {
            // EOF on tty device means USB-CDC disconnected
            closeUnlocked();
            throw CanException("DamiaoUsbCanTransport::receive: serial EOF (USB device disconnected)");
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            int err = errno;
            closeUnlocked();
            throw CanException("DamiaoUsbCanTransport::receive: read failed (port closed): " +
                               std::string(std::strerror(err)));
        }
    }
}

} // namespace damiao
