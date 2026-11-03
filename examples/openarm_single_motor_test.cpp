#include "damiao_arm_sdk/socketcan_bridge.hpp"

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using openarm::can::socket::OpenArm;
using openarm::damiao_motor::CallbackMode;
using openarm::damiao_motor::MotorType;
using openarm::damiao_motor::RID;

namespace {
std::atomic<bool> g_running{true};

void sigHandler(int) {
    g_running.store(false, std::memory_order_release);
}

int pollSingleResponse(OpenArm& arm, uint64_t prev_responses, int timeout_ms, double& rtt_us_out) {
    auto t0 = std::chrono::steady_clock::now();
    auto deadline = t0 + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        arm.recv_all(500); // wait up to 500 us per select call
        const auto& link = arm.get_arm().get_link_stats(0);
        if (link.responses > prev_responses) {
            auto t1 = std::chrono::steady_clock::now();
            rtt_us_out = std::chrono::duration<double, std::micro>(t1 - t0).count();
            return 1;
        }
    }
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    std::string can_if = "vcan0";
    std::string serial_port = "/dev/ttyACM0";
    uint32_t send_id = 0x01;
    uint32_t recv_id = 0x11;
    int samples = 200;
    int period_ms = 5;
    bool embed_bridge = false;
    bool test_disconnect = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            can_if = argv[++i];
        } else if (arg == "--serial" && i + 1 < argc) {
            serial_port = argv[++i];
        } else if (arg == "--send-id" && i + 1 < argc) {
            send_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--recv-id" && i + 1 < argc) {
            recv_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--samples" && i + 1 < argc) {
            samples = std::stoi(argv[++i]);
        } else if (arg == "--period-ms" && i + 1 < argc) {
            period_ms = std::stoi(argv[++i]);
        } else if (arg == "--with-bridge") {
            embed_bridge = true;
        } else if (arg == "--no-disconnect-test") {
            test_disconnect = false;
        }
    }

    std::cout << "====================================================================\n";
    std::cout << " Phase 1 Acceptance: Unmodified OpenArm CAN over Damiao USB-CAN Bridge\n";
    std::cout << "====================================================================\n";
    std::cout << "  OpenArm Interface : " << can_if << " (CAN 2.0, enable_fd = false)\n";
    std::cout << "  Target Motor      : J4310 (MotorType::DM4310)\n";
    std::cout << "  Send / Recv ID    : 0x" << std::hex << send_id << " / 0x" << recv_id << std::dec << "\n";
    std::cout << "  Embedded Bridge   : " << (embed_bridge ? ("YES (" + serial_port + ", read_only=true)") : "NO (external bridge daemon)") << "\n";
    std::cout << "--------------------------------------------------------------------\n";

    std::unique_ptr<damiao::SocketCanUsbBridge> bridge;
    if (embed_bridge) {
        damiao::BridgeConfig bcfg;
        bcfg.socketcan_if = can_if;
        bcfg.serial_port = serial_port;
        bcfg.baudrate = 921600;
        bcfg.auto_reconnect = true;
        bcfg.read_only = true; // P0 Safety: enforce read-only command filter in read-only acceptance test
        bridge = std::make_unique<damiao::SocketCanUsbBridge>(bcfg);
        bridge->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // 1. Initialize OpenArm("vcan0", false) with single J4310 motor
    // P0 Fix: Do NOT pass control_modes here so OpenArm does NOT write RID::CTRL_MODE (0x55) to the motor during read-only init!
    std::cout << "\n[Step 1] Initializing OpenArm(\"" << can_if << "\", false) with single J4310 motor (purely passive, no register writes)...\n";
    OpenArm openarm(can_if, false);
    openarm.init_arm_motors({MotorType::DM4310}, {send_id}, {recv_id});
    openarm.set_callback_mode_all(CallbackMode::STATE);

    // Drain any residual frames
    openarm.recv_all(1000);

    // 2. Hardware Read Test: Position, Velocity, Torque, MOS Temp, Rotor Temp, Fault/Status Code
    std::cout << "\n[Step 2] Reading initial J4310 state (Position, Velocity, Torque, Temperatures, Fault info)...\n";
    uint64_t prev_resp = openarm.get_arm().get_link_stats(0).responses;
    openarm.refresh_one(0);
    double first_rtt_us = 0.0;
    if (!pollSingleResponse(openarm, prev_resp, 100, first_rtt_us)) {
        std::cerr << "[FAIL] Did not receive response from J4310 within 100 ms!\n";
        return 1;
    }

    auto m0 = openarm.get_arm().get_motor(0);
    const auto& l0 = openarm.get_arm().get_link_stats(0);
    std::printf("  -> Status Code    : 0x%X (%s) | Enabled: %s | Has Error/Fault: %s\n",
                m0.get_error_code(),
                openarm::damiao_motor::motor_error_to_string(m0.get_error_code()),
                m0.is_enabled() ? "YES" : "NO",
                m0.has_error() ? "YES" : "NO");
    std::printf("  -> Position (q)   : %.5f rad (%.3f deg)\n",
                m0.get_position(), m0.get_position() * 180.0 / 3.141592653589793);
    std::printf("  -> Velocity (dq)  : %.5f rad/s\n", m0.get_velocity());
    std::printf("  -> Torque   (tau) : %.5f Nm\n", m0.get_torque());
    std::printf("  -> MOS Temp       : %d C\n", m0.get_state_tmos());
    std::printf("  -> Rotor Temp     : %d C\n", m0.get_state_trotor());
    std::printf("  -> First RTT      : %.1f us (%.3f ms)\n", first_rtt_us, first_rtt_us / 1000.0);
    std::printf("  -> Unmatched IDs  : %zu | Malformed frames: %u\n",
                openarm.get_unmatched_frames().size(), l0.malformed_frames);

    if (m0.get_state_tmos() <= 0 || m0.get_state_tmos() > 120 ||
        m0.get_state_trotor() <= 0 || m0.get_state_trotor() > 120) {
        std::cerr << "[FAIL] Abnormal temperature reading!\n";
        return 1;
    }

    // Also verify parameter reading (PMAX, VMAX, TMAX, MST_ID, ESC_ID) via OpenArm PARAM mode
    std::cout << "\n[Step 2b] Verifying OpenArm parameter query (RID::MST_ID, ESC_ID, PMAX, VMAX, TMAX)...\n";
    openarm.set_callback_mode_all(CallbackMode::PARAM);
    const int rids_to_query[] = {
        static_cast<int>(RID::ESC_ID),
        static_cast<int>(RID::MST_ID),
        static_cast<int>(RID::PMAX),
        static_cast<int>(RID::VMAX),
        static_cast<int>(RID::TMAX),
    };
    for (int rid : rids_to_query) {
        uint64_t before = openarm.get_arm().get_link_stats(0).responses;
        openarm.query_param_all(rid);
        double rtt = 0.0;
        pollSingleResponse(openarm, before, 50, rtt);
    }
    m0 = openarm.get_arm().get_motor(0);
    std::printf("  -> ESC_ID=%.0f, MST_ID=%.0f, PMAX=%.2f rad, VMAX=%.2f rad/s, TMAX=%.2f Nm\n",
                m0.get_param(static_cast<int>(RID::ESC_ID)),
                m0.get_param(static_cast<int>(RID::MST_ID)),
                m0.get_param(static_cast<int>(RID::PMAX)),
                m0.get_param(static_cast<int>(RID::VMAX)),
                m0.get_param(static_cast<int>(RID::TMAX)));

    openarm.set_callback_mode_all(CallbackMode::STATE);

    // 3. Continuous Feedback & Latency Benchmark
    std::cout << "\n[Step 3] Continuous feedback & RTT latency test (" << samples
              << " cycles @ " << period_ms << " ms period)...\n";
    std::vector<double> rtts_us;
    rtts_us.reserve(samples);
    int success_count = 0;
    int timeout_count = 0;

    auto loop_start = std::chrono::steady_clock::now();
    for (int i = 0; i < samples && g_running.load(std::memory_order_acquire); ++i) {
        auto iter_start = std::chrono::steady_clock::now();
        uint64_t before = openarm.get_arm().get_link_stats(0).responses;
        openarm.refresh_one(0);

        double rtt_us = 0.0;
        if (pollSingleResponse(openarm, before, 50, rtt_us)) {
            rtts_us.push_back(rtt_us);
            success_count++;
        } else {
            timeout_count++;
        }

        if ((i + 1) % 50 == 0 || i == samples - 1) {
            auto cur_m = openarm.get_arm().get_motor(0);
            std::printf("  [%3d/%3d] q=%+.4f rad, dq=%+.4f rad/s, tau=%+.4f Nm, T=%d/%d C, status=%s, last_rtt=%.2f ms\n",
                        i + 1, samples,
                        cur_m.get_position(), cur_m.get_velocity(), cur_m.get_torque(),
                        cur_m.get_state_tmos(), cur_m.get_state_trotor(),
                        openarm::damiao_motor::motor_error_to_string(cur_m.get_error_code()),
                        rtt_us / 1000.0);
        }

        auto elapsed = std::chrono::steady_clock::now() - iter_start;
        auto target = std::chrono::milliseconds(period_ms);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }
    auto loop_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(loop_end - loop_start).count();

    if (rtts_us.empty()) {
        std::cerr << "[FAIL] Zero responses collected during continuous feedback test!\n";
        return 1;
    }

    std::sort(rtts_us.begin(), rtts_us.end());
    double sum_us = std::accumulate(rtts_us.begin(), rtts_us.end(), 0.0);
    double mean_us = sum_us / rtts_us.size();
    double min_us = rtts_us.front();
    double max_us = rtts_us.back();
    double p50_us = rtts_us[rtts_us.size() * 50 / 100];
    double p95_us = rtts_us[rtts_us.size() * 95 / 100];
    double p99_us = rtts_us[rtts_us.size() * 99 / 100];

    std::printf("\n  --- Continuous Feedback & Latency Summary ---\n");
    std::printf("  Sent / Received   : %d / %d (Success Rate: %.2f%%, Timeouts: %d)\n",
                samples, success_count, 100.0 * success_count / samples, timeout_count);
    std::printf("  Effective Rate    : %.1f Hz (over %.2f s)\n", success_count / total_sec, total_sec);
    std::printf("  RTT Latency (ms)  : min=%.3f | avg=%.3f | p50=%.3f | p95=%.3f | p99=%.3f | max=%.3f\n",
                min_us / 1000.0, mean_us / 1000.0, p50_us / 1000.0,
                p95_us / 1000.0, p99_us / 1000.0, max_us / 1000.0);
    std::printf("  Unmatched Frames  : %zu ( confirms zero loopback / zero echo )\n",
                openarm.get_unmatched_frames().size());
    std::printf("  Malformed Frames  : %u\n", openarm.get_arm().get_link_stats(0).malformed_frames);

    // 4. Timeout, Staleness & Disconnect/Recovery Test
    if (embed_bridge && test_disconnect) {
        std::cout << "\n[Step 4] Testing Bridge Disconnect, Timeout/Staleness Detection, and Reconnection...\n";
        std::cout << "  -> Stopping Bridge to simulate USB-CAN disconnection...\n";
        bridge->stop();

        uint64_t resp_before_drop = openarm.get_arm().get_link_stats(0).responses;
        for (int i = 0; i < 5; ++i) {
            openarm.refresh_one(0);
            openarm.recv_all(5000); // 5ms timeout
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        uint64_t resp_after_drop = openarm.get_arm().get_link_stats(0).responses;
        const auto& link_drop = openarm.get_arm().get_link_stats(0);
        bool is_stale = link_drop.is_stale(std::chrono::milliseconds(100));
        double silence_ms = std::chrono::duration<double, std::milli>(link_drop.since_last_response()).count();

        std::printf("  -> During disconnect: new responses=%lu (expected 0), silence=%.1f ms, is_stale(100ms)=%s\n",
                    resp_after_drop - resp_before_drop, silence_ms, is_stale ? "TRUE" : "FALSE");

        if (resp_after_drop != resp_before_drop || !is_stale) {
            std::cerr << "[FAIL] Disconnect / staleness check failed!\n";
            return 1;
        }

        std::cout << "  -> Restarting Bridge to verify recovery...\n";
        bridge->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // Drain any queued vcan0 frames and verify fresh communication
        openarm.recv_all(2000);
        uint64_t resp_before_recover = openarm.get_arm().get_link_stats(0).responses;
        openarm.refresh_one(0);
        double rec_rtt_us = 0.0;
        bool recovered = pollSingleResponse(openarm, resp_before_recover, 100, rec_rtt_us);
        bool fresh_after_recover = !openarm.get_arm().get_link_stats(0).is_stale(std::chrono::milliseconds(100));

        std::printf("  -> Post-recovery    : response_ok=%s, rtt=%.3f ms, is_fresh=%s\n",
                    recovered ? "YES" : "NO", rec_rtt_us / 1000.0, fresh_after_recover ? "YES" : "NO");

        if (!recovered || !fresh_after_recover) {
            std::cerr << "[FAIL] Bridge recovery failed!\n";
            return 1;
        }
    }

    if (bridge) {
        auto bst = bridge->getStats();
        std::printf("\n  --- Bridge Forwarding Diagnostics ---\n");
        std::printf("  vcan0 -> USB forwarded : %lu\n", bst.sock_to_usb_forwarded);
        std::printf("  USB -> vcan0 forwarded : %lu\n", bst.usb_to_sock_forwarded);
        std::printf("  Read-Only Blocked      : %lu\n", bst.sock_readonly_blocked);
        std::printf("  Dropped (vcan0 / USB)  : %lu / %lu\n", bst.sock_rx_dropped, bst.usb_rx_dropped);
        std::printf("  Errors  (vcan0 / USB)  : %lu / %lu\n", bst.sock_errors, bst.usb_errors);
        bridge->stop();
    }

    bool pass = (success_count == samples) &&
                openarm.get_unmatched_frames().empty() &&
                (openarm.get_arm().get_link_stats(0).malformed_frames == 0);

    std::cout << "\n====================================================================\n";
    std::cout << " Phase 1 Acceptance Result: " << (pass ? "PASSED" : "FAILED") << "\n";
    std::cout << "====================================================================\n";
    return pass ? 0 : 1;
}
