#pragma once

#include "vero_hardware/motor_interface.hpp"
#include <memory>
#include <chrono>

class MotorDriver {
public: 
    MotorDriver(int address);
    void set_wheel_velocities(double left, double right);
    void stop();
    void check_command_timeout();  // Call this periodically to check for timeouts

private:
    int address_;
    std::shared_ptr<MotorDriverHAT> hat_;
    
    // Command timeout safety feature
    std::chrono::steady_clock::time_point last_command_time_;
    static constexpr double COMMAND_TIMEOUT_SECONDS = 0.5;  // 500ms timeout
    bool is_stopped_ = true;
};

