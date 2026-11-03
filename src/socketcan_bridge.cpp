#include "damiao_arm_sdk/socketcan_bridge.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace damiao {

namespace {
void logFrame(const char* dir, const CanFrame& f) {
    std::cout << "[Bridge " << dir << "] ID=0x" << std::hex << std::setw(3) << std::setfill('0')
              << f.can_id << std::dec << " DLC=" << static_cast<int>(f.dlc) << " Data=[";
    for (uint8_t i = 0; i < f.dlc && i < 8; ++i) {
        if (i > 0) std::cout << " ";
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(f.data[i]) << std::dec;
    }
    std::cout << "]" << std::endl;
}
} // namespace

SocketCanUsbBridge::SocketCanUsbBridge(const BridgeConfig& config)
    : config_(config),
      read_only_(config.read_only),
      usb_can_(std::make_shared<DamiaoUsbCanTransport>()) {
    usb_can_->setBaudrate(config_.baudrate);
}

SocketCanUsbBridge::~SocketCanUsbBridge() {
    stop();
}

bool SocketCanUsbBridge::isValidForwardFrame(const CanFrame& frame,
                                             bool allow_extended,
                                             bool allow_rtr) noexcept {
    if (frame.is_error) {
        return false;
    }
    if (frame.dlc > 8) {
        return false;
    }
    if (!allow_extended && frame.is_extended) {
        return false;
    }
    if (!allow_rtr && frame.is_rtr) {
        return false;
    }
    if (frame.is_extended) {
        if (frame.can_id > CAN_EFF_MASK) return false;
    } else {
        if (frame.can_id > CAN_SFF_MASK) return false;
    }
    return true;
}

bool SocketCanUsbBridge::isReadOnlyAllowedTxFrame(const CanFrame& frame) noexcept {
    if (frame.is_error || frame.is_extended || frame.is_rtr) {
        return false;
    }
    if (frame.can_id != 0x7FF || frame.dlc != 8) {
        return false;
    }
    // Target motor slave ID (little-endian uint16 in data[0..1]) must be non-zero
    uint16_t target_id = static_cast<uint16_t>(frame.data[0]) |
                         (static_cast<uint16_t>(frame.data[1]) << 8);
    if (target_id == 0 || target_id > 0x7FF) {
        return false;
    }
    // Trailing payload bytes [4..7] must be 0x00 for passive query commands
    if (frame.data[4] != 0x00 || frame.data[5] != 0x00 ||
        frame.data[6] != 0x00 || frame.data[7] != 0x00) {
        return false;
    }
    uint8_t sub_cmd = frame.data[2];
    if (sub_cmd == 0xCC) {
        // Passive status refresh requires data[3] == 0x00
        return frame.data[3] == 0x00;
    }
    if (sub_cmd == 0x33) {
        // Parameter query (data[3] is RID)
        return true;
    }
    return false;
}

void SocketCanUsbBridge::openSocketCan() {
    std::lock_guard<std::mutex> lock(sock_mutex_);
    if (config_.socketcan_if.empty()) {
        throw CanException("SocketCanUsbBridge: socketcan_if cannot be empty");
    }
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        throw CanException("SocketCanUsbBridge: socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: " +
                           std::string(std::strerror(errno)));
    }

    // Enable local socket loopback so OpenArm on the same host receives frames injected by Bridge,
    // and disable CAN_RAW_RECV_OWN_MSGS so Bridge never receives its own injected frames.
    int loopback_opt = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_opt, sizeof(loopback_opt)) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanUsbBridge: setsockopt(CAN_RAW_LOOPBACK) failed: " +
                           std::string(std::strerror(err)));
    }

    int recv_own_opt = 0;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_opt, sizeof(recv_own_opt)) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanUsbBridge: setsockopt(CAN_RAW_RECV_OWN_MSGS) failed: " +
                           std::string(std::strerror(err)));
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, config_.socketcan_if.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanUsbBridge: interface '" + config_.socketcan_if +
                           "' not found: " + std::string(std::strerror(err)));
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanUsbBridge: bind failed on '" + config_.socketcan_if +
                           "': " + std::string(std::strerror(err)));
    }

    sock_fd_ = fd;
}

void SocketCanUsbBridge::closeSocketCan() noexcept {
    std::lock_guard<std::mutex> lock(sock_mutex_);
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void SocketCanUsbBridge::sendSocketCan(const CanFrame& frame) {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        fd = sock_fd_;
    }
    if (fd < 0) {
        throw CanException("SocketCanUsbBridge::sendSocketCan: socket is not open");
    }

    struct can_frame raw {};
    raw.can_id = frame.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    if (frame.is_extended) {
        raw.can_id |= CAN_EFF_FLAG;
    }
    if (frame.is_rtr) {
        raw.can_id |= CAN_RTR_FLAG;
    }
    raw.can_dlc = frame.dlc;
    std::memcpy(raw.data, frame.data.data(), frame.dlc);

    ssize_t nbytes = ::write(fd, &raw, sizeof(raw));
    if (nbytes != static_cast<ssize_t>(sizeof(raw))) {
        throw CanException("SocketCanUsbBridge::sendSocketCan: write failed: " +
                           std::string(std::strerror(errno)));
    }
}

bool SocketCanUsbBridge::receiveSocketCan(CanFrame& frame, int timeout_ms) {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        fd = sock_fd_;
    }
    if (fd < 0) {
        throw CanException("SocketCanUsbBridge::receiveSocketCan: socket is not open");
    }

    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) {
            return false;
        }
        throw CanException("SocketCanUsbBridge::receiveSocketCan: poll failed: " +
                           std::string(std::strerror(errno)));
    }
    if (ret == 0) {
        return false;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw CanException("SocketCanUsbBridge::receiveSocketCan: socket poll error event");
    }

    struct can_frame raw {};
    ssize_t nbytes = ::read(fd, &raw, sizeof(raw));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return false;
        }
        throw CanException("SocketCanUsbBridge::receiveSocketCan: read failed: " +
                           std::string(std::strerror(errno)));
    }
    if (nbytes != static_cast<ssize_t>(sizeof(raw))) {
        throw CanException("SocketCanUsbBridge::receiveSocketCan: incomplete CAN frame read");
    }

    frame.is_extended = (raw.can_id & CAN_EFF_FLAG) != 0;
    frame.is_rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
    frame.is_error = (raw.can_id & CAN_ERR_FLAG) != 0;
    frame.can_id = raw.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    if (raw.can_dlc > 8) {
        throw CanException("SocketCanUsbBridge::receiveSocketCan: invalid CAN DLC (> 8)");
    }
    frame.dlc = raw.can_dlc;
    frame.data.fill(0);
    std::memcpy(frame.data.data(), raw.data, frame.dlc);
    return true;
}

void SocketCanUsbBridge::start() {
    stop();

    openSocketCan();

    try {
        usb_can_->open(config_.serial_port);
    } catch (const std::exception& e) {
        if (!config_.auto_reconnect) {
            closeSocketCan();
            throw;
        }
        std::cerr << "[Bridge] Initial USB-CAN open warning (" << config_.serial_port
                  << "): " << e.what() << " (will retry in background)" << std::endl;
    }

    running_.store(true, std::memory_order_release);
    sock_to_usb_thread_ = std::thread(&SocketCanUsbBridge::sockToUsbLoop, this);
    usb_to_sock_thread_ = std::thread(&SocketCanUsbBridge::usbToSockLoop, this);
}

void SocketCanUsbBridge::stop() noexcept {
    running_.store(false, std::memory_order_release);

    if (sock_to_usb_thread_.joinable()) {
        sock_to_usb_thread_.join();
    }
    if (usb_to_sock_thread_.joinable()) {
        usb_to_sock_thread_.join();
    }

    closeSocketCan();
    if (usb_can_) {
        usb_can_->close();
    }
}

bool SocketCanUsbBridge::tryReconnectUsb() {
    std::lock_guard<std::mutex> lock(usb_reconnect_mutex_);
    if (usb_can_->isOpen()) {
        return true;
    }
    try {
        usb_can_->open(config_.serial_port);
        usb_reconnects_.fetch_add(1, std::memory_order_relaxed);
        if (config_.verbose) {
            std::cout << "[Bridge] Reconnected to USB-CAN: " << config_.serial_port << std::endl;
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void SocketCanUsbBridge::sockToUsbLoop() {
    CanFrame frame;
    while (running_.load(std::memory_order_relaxed)) {
        bool received = false;
        try {
            received = receiveSocketCan(frame, 10);
        } catch (const std::exception& e) {
            sock_errors_.fetch_add(1, std::memory_order_relaxed);
            if (!running_.load(std::memory_order_relaxed)) {
                break;
            }
            if (config_.verbose) {
                std::cerr << "[Bridge] SocketCAN receive error: " << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (!received) {
            continue;
        }

        if (!isValidForwardFrame(frame, config_.allow_extended_frames, config_.allow_rtr_frames)) {
            sock_rx_dropped_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // P0: Read-only command filtering (blocks motion / enable / disable / write_param frames)
        if (read_only_.load(std::memory_order_relaxed) && !isReadOnlyAllowedTxFrame(frame)) {
            sock_readonly_blocked_.fetch_add(1, std::memory_order_relaxed);
            if (config_.verbose) {
                logFrame("BLOCKED(read-only)", frame);
            }
            continue;
        }

        if (!usb_can_->isOpen()) {
            sock_rx_dropped_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        try {
            usb_can_->send(frame);
            sock_to_usb_forwarded_.fetch_add(1, std::memory_order_relaxed);
            if (config_.verbose) {
                logFrame("vcan->usb", frame);
            }
        } catch (const std::exception& e) {
            usb_errors_.fetch_add(1, std::memory_order_relaxed);
            if (config_.verbose) {
                std::cerr << "[Bridge] SocketCAN -> USB send error: " << e.what() << std::endl;
            }
            if (!config_.auto_reconnect) {
                running_.store(false, std::memory_order_release);
                break;
            }
        }
    }
}

void SocketCanUsbBridge::usbToSockLoop() {
    CanFrame frame;
    while (running_.load(std::memory_order_relaxed)) {
        if (!usb_can_->isOpen()) {
            if (!config_.auto_reconnect) {
                running_.store(false, std::memory_order_release);
                break;
            }
            if (!tryReconnectUsb()) {
                int slept = 0;
                while (running_.load(std::memory_order_relaxed) &&
                       slept < config_.reconnect_interval_ms) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    slept += 20;
                }
                continue;
            }
        }

        bool received = false;
        try {
            received = usb_can_->receive(frame, 10);
        } catch (const std::exception& e) {
            usb_errors_.fetch_add(1, std::memory_order_relaxed);
            if (!running_.load(std::memory_order_relaxed)) {
                break;
            }
            if (config_.verbose) {
                std::cerr << "[Bridge] USB receive error / disconnected: " << e.what() << std::endl;
            }
            if (!config_.auto_reconnect) {
                running_.store(false, std::memory_order_release);
                break;
            }
            continue;
        }

        if (!received) {
            continue;
        }

        if (!isValidForwardFrame(frame, config_.allow_extended_frames, config_.allow_rtr_frames)) {
            usb_rx_dropped_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        try {
            sendSocketCan(frame);
            usb_to_sock_forwarded_.fetch_add(1, std::memory_order_relaxed);
            if (config_.verbose) {
                logFrame("usb->vcan", frame);
            }
        } catch (const std::exception& e) {
            sock_errors_.fetch_add(1, std::memory_order_relaxed);
            if (config_.verbose) {
                std::cerr << "[Bridge] USB -> SocketCAN send error: " << e.what() << std::endl;
            }
        }
    }
}

BridgeStats SocketCanUsbBridge::getStats() const noexcept {
    BridgeStats st;
    st.sock_to_usb_forwarded = sock_to_usb_forwarded_.load(std::memory_order_relaxed);
    st.usb_to_sock_forwarded = usb_to_sock_forwarded_.load(std::memory_order_relaxed);
    st.sock_rx_dropped = sock_rx_dropped_.load(std::memory_order_relaxed);
    st.sock_readonly_blocked = sock_readonly_blocked_.load(std::memory_order_relaxed);
    st.usb_rx_dropped = usb_rx_dropped_.load(std::memory_order_relaxed);
    st.sock_errors = sock_errors_.load(std::memory_order_relaxed);
    st.usb_errors = usb_errors_.load(std::memory_order_relaxed);
    st.usb_reconnects = usb_reconnects_.load(std::memory_order_relaxed);
    st.usb_connected = usb_can_ && usb_can_->isOpen();
    {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        st.sock_connected = (sock_fd_ >= 0);
    }
    if (usb_can_) {
        st.usb_transport_stats = usb_can_->getStats();
    }
    return st;
}

} // namespace damiao
