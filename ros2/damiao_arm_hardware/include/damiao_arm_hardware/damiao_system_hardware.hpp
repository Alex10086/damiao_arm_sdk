#pragma once

#include "damiao_arm_sdk/arm.hpp"
#include "damiao_arm_sdk/socketcan_bridge.hpp"
#include "damiao_arm_sdk/types.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace damiao_arm_hardware {

class DamiaoSystemHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DamiaoSystemHardware)

    DamiaoSystemHardware() = default;
    ~DamiaoSystemHardware() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    enum ClaimBits : uint8_t {
        CLAIM_NONE = 0,
        CLAIM_POS  = 1 << 0,
        CLAIM_VEL  = 1 << 1,
        CLAIM_EFF  = 1 << 2,
        CLAIM_KP   = 1 << 3,
        CLAIM_KD   = 1 << 4
    };

    bool parseHardwareParameters(const hardware_interface::HardwareInfo& info);
    bool computeCandidateClaims(const std::vector<std::string>& start_interfaces,
                                const std::vector<std::string>& stop_interfaces,
                                std::vector<uint8_t>& candidate_claims) const;
    static bool isSupportedClaimMask(uint8_t mask) noexcept;
    void copyArmStateToBuffers(const damiao::ArmState& state);
    void latchAllCommandsToCurrentState();
    damiao::MITParam buildMitCommand(std::size_t idx, const damiao::JointConfig& jc);
    void teardownHardware() noexcept;

    damiao::ArmConfig arm_config_{};
    damiao::BridgeConfig bridge_config_{};
    bool use_usb_bridge_{false};

    std::unique_ptr<damiao::SocketCanUsbBridge> bridge_;
    std::unique_ptr<damiao::Arm> arm_;

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, std::size_t> joint_index_by_name_;

    std::vector<double> default_kp_;
    std::vector<double> default_kd_;

    std::vector<double> pos_states_;
    std::vector<double> vel_states_;
    std::vector<double> tau_states_;

    std::vector<double> pos_commands_;
    std::vector<double> vel_commands_;
    std::vector<double> tau_commands_;
    std::vector<double> kp_commands_;
    std::vector<double> kd_commands_;

    std::vector<double> pos_latched_;
    std::vector<uint8_t> claimed_mask_;

    std::chrono::steady_clock::time_point last_disarmed_refresh_{};
};

} // namespace damiao_arm_hardware
