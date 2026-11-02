#include "damiao_arm_sdk/can_transport.hpp"

#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace damiao {

SocketCanTransport::SocketCanTransport(const std::string& interface_name, bool loopback)
    : loopback_(loopback) {
    open(interface_name);
}

SocketCanTransport::~SocketCanTransport() {
    close();
}

void SocketCanTransport::open(const std::string& interface_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (interface_name.empty()) {
        throw CanException("SocketCanTransport::open: interface_name cannot be empty");
    }
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        throw CanException("SocketCanTransport::open: socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: " +
                           std::string(std::strerror(errno)));
    }

    int loopback_opt = loopback_ ? 1 : 0;
    ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_opt, sizeof(loopback_opt));

    int recv_own_opt = recv_own_msgs_ ? 1 : 0;
    ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_opt, sizeof(recv_own_opt));

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanTransport::open: interface '" + interface_name +
                           "' not found: " + std::string(std::strerror(err)));
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        int err = errno;
        ::close(fd);
        throw CanException("SocketCanTransport::open: bind failed on '" + interface_name +
                           "': " + std::string(std::strerror(err)));
    }

    sock_fd_ = fd;
    ifname_ = interface_name;
}

void SocketCanTransport::close() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    ifname_.clear();
}

bool SocketCanTransport::isOpen() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return sock_fd_ >= 0;
}

std::string SocketCanTransport::interfaceName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ifname_;
}

void SocketCanTransport::setLoopback(bool enable, bool recv_own_msgs) {
    std::lock_guard<std::mutex> lock(mutex_);
    loopback_ = enable;
    recv_own_msgs_ = recv_own_msgs;
    if (sock_fd_ >= 0) {
        int loopback_opt = loopback_ ? 1 : 0;
        if (::setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_opt, sizeof(loopback_opt)) < 0) {
            throw CanException("SocketCanTransport::setLoopback: setsockopt(CAN_RAW_LOOPBACK) failed: " +
                               std::string(std::strerror(errno)));
        }
        int recv_own_opt = recv_own_msgs_ ? 1 : 0;
        if (::setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_opt, sizeof(recv_own_opt)) < 0) {
            throw CanException("SocketCanTransport::setLoopback: setsockopt(CAN_RAW_RECV_OWN_MSGS) failed: " +
                               std::string(std::strerror(errno)));
        }
    }
}

void SocketCanTransport::send(const CanFrame& frame) {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fd = sock_fd_;
    }
    if (fd < 0) {
        throw CanException("SocketCanTransport::send: CAN socket is not open");
    }
    if (frame.dlc > 8) {
        throw std::invalid_argument("SocketCanTransport::send: invalid DLC (> 8)");
    }
    if (!frame.is_extended && frame.can_id > CAN_SFF_MASK) {
        throw std::invalid_argument("SocketCanTransport::send: standard 11-bit CAN ID exceeds 0x7FF");
    }
    if (frame.is_extended && frame.can_id > CAN_EFF_MASK) {
        throw std::invalid_argument("SocketCanTransport::send: extended 29-bit CAN ID exceeds 0x1FFFFFFF");
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
        throw CanException("SocketCanTransport::send: write failed: " +
                           std::string(std::strerror(errno)));
    }
}

bool SocketCanTransport::receive(CanFrame& frame, int timeout_ms) {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fd = sock_fd_;
    }
    if (fd < 0) {
        throw CanException("SocketCanTransport::receive: CAN socket is not open");
    }

    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) {
            return false;
        }
        throw CanException("SocketCanTransport::receive: poll failed: " +
                           std::string(std::strerror(errno)));
    }
    if (ret == 0) {
        return false; // Timeout
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw CanException("SocketCanTransport::receive: socket poll error event");
    }

    struct can_frame raw {};
    ssize_t nbytes = ::read(fd, &raw, sizeof(raw));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return false;
        }
        throw CanException("SocketCanTransport::receive: read failed: " +
                           std::string(std::strerror(errno)));
    }
    if (nbytes != static_cast<ssize_t>(sizeof(raw))) {
        throw CanException("SocketCanTransport::receive: incomplete CAN frame read");
    }

    frame.is_extended = (raw.can_id & CAN_EFF_FLAG) != 0;
    frame.is_rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
    frame.is_error = (raw.can_id & CAN_ERR_FLAG) != 0;
    frame.can_id = raw.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    frame.dlc = raw.can_dlc > 8 ? 8 : raw.can_dlc;
    frame.data.fill(0);
    std::memcpy(frame.data.data(), raw.data, frame.dlc);
    return true;
}

} // namespace damiao
