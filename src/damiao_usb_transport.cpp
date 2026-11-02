#include "damiao_arm_sdk/damiao_usb_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
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

void DamiaoUsbCanTransport::open(const std::string& port_path) {
    std::lock_guard<std::mutex> lock_tx(tx_mutex_);
    std::lock_guard<std::mutex> lock_rx(rx_mutex_);

    if (port_path.empty()) {
        throw CanException("DamiaoUsbCanTransport::open: port_path cannot be empty");
    }
    int cur_fd = fd_.load();
    if (cur_fd >= 0) {
        ::close(cur_fd);
        fd_.store(-1);
    }

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

    fd_.store(fd);
    port_path_ = port_path;
    rx_buf_.clear();
}

void DamiaoUsbCanTransport::close() noexcept {
    std::lock_guard<std::mutex> lock_tx(tx_mutex_);
    std::lock_guard<std::mutex> lock_rx(rx_mutex_);

    int cur_fd = fd_.exchange(-1);
    if (cur_fd >= 0) {
        ::close(cur_fd);
    }
    port_path_.clear();
    rx_buf_.clear();
}

bool DamiaoUsbCanTransport::isOpen() const noexcept {
    return fd_.load() >= 0;
}

std::string DamiaoUsbCanTransport::interfaceName() const {
    return port_path_;
}

void DamiaoUsbCanTransport::send(const CanFrame& frame) {
    int fd = fd_.load();
    if (fd < 0) {
        throw CanException("DamiaoUsbCanTransport::send: port is not open");
    }

    DamiaoUsbTxPacket pkt;
    pkt.id_type = frame.is_extended ? 1 : 0;
    pkt.can_id = frame.can_id;
    pkt.frame_type = frame.is_rtr ? 1 : 0;
    pkt.dlc = frame.dlc > 8 ? 8 : frame.dlc;
    std::memcpy(pkt.data, frame.data.data(), pkt.dlc);

    std::lock_guard<std::mutex> lock(tx_mutex_);
    ssize_t written = ::write(fd, &pkt, sizeof(pkt));
    if (written != static_cast<ssize_t>(sizeof(pkt))) {
        throw CanException("DamiaoUsbCanTransport::send: write failed: " +
                           std::string(std::strerror(errno)));
    }
}

bool DamiaoUsbCanTransport::receive(CanFrame& frame, int timeout_ms) {
    int fd = fd_.load();
    if (fd < 0) {
        throw CanException("DamiaoUsbCanTransport::receive: port is not open");
    }

    std::lock_guard<std::mutex> lock(rx_mutex_);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (true) {
        // First check if a complete 16-byte frame exists in rx_buf_
        while (rx_buf_.size() >= 16) {
            if (rx_buf_[0] == 0xAA && rx_buf_[15] == 0x55) {
                // Header (0xAA) and Footer (0x55) match
                uint8_t cmd = rx_buf_[1];
                if (cmd == 0x11) { // 0x11 = receive success
                    frame.dlc = rx_buf_[2] & 0x3F;
                    frame.is_extended = (rx_buf_[2] & 0x40) != 0;
                    frame.is_rtr = (rx_buf_[2] & 0x80) != 0;
                    frame.is_error = false;
                    std::memcpy(&frame.can_id, &rx_buf_[3], 4);
                    frame.data.fill(0);
                    uint8_t copy_len = frame.dlc > 8 ? 8 : frame.dlc;
                    std::memcpy(frame.data.data(), &rx_buf_[7], copy_len);

                    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 16);
                    return true;
                }
                // If CMD was not 0x11 (e.g. heartbeat or status ACK), consume and continue
                rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 16);
                continue;
            }
            // Discard leading invalid byte
            rx_buf_.erase(rx_buf_.begin());
        }

        // Need more data from serial port
        int remaining_ms = 0;
        if (timeout_ms > 0) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return false;
            }
            remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            if (remaining_ms < 0) remaining_ms = 0;
        }

        struct pollfd pfd {};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, remaining_ms);
        if (ret <= 0) {
            return false;
        }

        uint8_t tmp[128];
        ssize_t n = ::read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
        } else if (n < 0 && (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            throw CanException("DamiaoUsbCanTransport::receive: read failed: " +
                               std::string(std::strerror(errno)));
        }
    }
}

} // namespace damiao
