#include "damiao_arm_sdk/socketcan_bridge.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <linux/can.h>

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
      sock_can_(std::make_shared<SocketCanTransport>()),
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

void SocketCanUsbBridge::start() {
    // Ensure any previous worker threads are joined before starting new ones
    stop();

    // Configure SocketCAN with loopback=true (so local OpenArm clients receive frames
    // injected by the bridge) and recv_own_msgs=false (so the bridge never receives
    // its own injected frames, preventing any loopback storm).
    sock_can_->setLoopback(true, false);
    sock_can_->open(config_.socketcan_if);

    try {
        usb_can_->open(config_.serial_port);
    } catch (const std::exception& e) {
        if (!config_.auto_reconnect) {
            sock_can_->close();
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

    if (sock_can_) {
        sock_can_->close();
    }
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
            received = sock_can_->receive(frame, 10);
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
            sock_can_->send(frame);
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
    st.usb_rx_dropped = usb_rx_dropped_.load(std::memory_order_relaxed);
    st.sock_errors = sock_errors_.load(std::memory_order_relaxed);
    st.usb_errors = usb_errors_.load(std::memory_order_relaxed);
    st.usb_reconnects = usb_reconnects_.load(std::memory_order_relaxed);
    st.usb_connected = usb_can_ && usb_can_->isOpen();
    st.sock_connected = sock_can_ && sock_can_->isOpen();
    if (usb_can_) {
        st.usb_transport_stats = usb_can_->getStats();
    }
    return st;
}

} // namespace damiao
