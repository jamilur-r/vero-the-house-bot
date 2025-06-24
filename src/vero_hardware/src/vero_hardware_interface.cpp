#include "vero_hardware/vero_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

VeroHardwareInterface::VeroHardwareInterface()
    : left_wheel_pos_(0.0), left_wheel_vel_(0.0), 
      right_wheel_pos_(0.0), right_wheel_vel_(0.0),
      left_wheel_cmd_(0.0), right_wheel_cmd_(0.0)
{
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    motor_driver_ = std::make_unique<MotorDriver>(0x40);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    motor_driver_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (motor_driver_) {
        motor_driver_->stop();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VeroHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (motor_driver_) {
        motor_driver_->stop();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VeroHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "left_wheel_joint", "position", &left_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "left_wheel_joint", "velocity", &left_wheel_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "right_wheel_joint", "position", &right_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "right_wheel_joint", "velocity", &right_wheel_vel_));
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VeroHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "left_wheel_joint", "velocity", &left_wheel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "right_wheel_joint", "velocity", &right_wheel_cmd_));
    
    return command_interfaces;
}

hardware_interface::return_type VeroHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // For now, just copy command to state (no real feedback)
    left_wheel_vel_ = left_wheel_cmd_;
    right_wheel_vel_ = right_wheel_cmd_;
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VeroHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    static int call_count = 0;
    call_count++;
    
    if (motor_driver_) {
        // Add comprehensive debug output - only show if commands are non-zero
        if (std::abs(left_wheel_cmd_) > 0.001 || std::abs(right_wheel_cmd_) > 0.001) {
            std::cout << "### HARDWARE INTERFACE WRITE (call #" << call_count << ") ###" << std::endl;
            std::cout << "Command values: left_wheel_cmd_=" << left_wheel_cmd_ << ", right_wheel_cmd_=" << right_wheel_cmd_ << std::endl;
            std::cout << "################################" << std::endl << std::flush;
            
            motor_driver_->set_wheel_velocities(left_wheel_cmd_, right_wheel_cmd_);
        }
    } else {
        std::cout << "ERROR: motor_driver_ is null!" << std::endl << std::flush;
    }
    return hardware_interface::return_type::OK;
}

// Export as plugin
PLUGINLIB_EXPORT_CLASS(VeroHardwareInterface, hardware_interface::SystemInterface)