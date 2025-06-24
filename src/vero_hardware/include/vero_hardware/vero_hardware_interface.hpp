#pragma once

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vero_hardware/motor_driver.hpp"
#include <memory>
#include <vector>

class VeroHardwareInterface : public hardware_interface::SystemInterface {
public:
    VeroHardwareInterface();
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    std::unique_ptr<MotorDriver> motor_driver_;
    
    // Hardware states
    double left_wheel_pos_;
    double left_wheel_vel_;
    double right_wheel_pos_;
    double right_wheel_vel_;
    
    // Hardware commands
    double left_wheel_cmd_;
    double right_wheel_cmd_;
};
