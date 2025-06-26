#include "vero_hardware/motor_driver.hpp"
#include "vero_hardware/motor_interface.hpp"  // Include your low-level HAT driver

// You may want to store a pointer or instance to your MotorDriverHAT
MotorDriver::MotorDriver(int address)
    : address_(address), is_stopped_(true)
{
    // Initialize the MotorDriverHAT (using default I2C device)
    hat_ = std::make_shared<MotorDriverHAT>("/dev/i2c-1", static_cast<uint8_t>(address_));
    if (!hat_->initialize(50)) {
        throw std::runtime_error("Failed to initialize MotorDriverHAT");
    }
    
    // Initialize the last command time
    last_command_time_ = std::chrono::steady_clock::now();
}

void MotorDriver::set_wheel_velocities(double left, double right) {
    // Update command timestamp
    last_command_time_ = std::chrono::steady_clock::now();
    
    // === Detect motion type ===
    bool is_in_place = (std::abs(left + right) < 1e-3) && (std::abs(left - right) > 1e-3);
    bool is_pure_forward = std::abs(left - right) < 0.05 * std::max(std::abs(left), std::abs(right));
    bool is_curve = !is_in_place && !is_pure_forward && !is_stopped;
    bool is_stopped = std::abs(left) < 1e-3 && std::abs(right) < 1e-3;

    // Debug: Let's see what values we're getting - ALWAYS SHOW FOR DEBUGGING
    std::cout << "MOTION DEBUG: left=" << left << ", right=" << right 
              << ", diff=" << std::abs(left - right) << std::endl;
    std::cout << "  is_pure_forward=" << is_pure_forward 
              << ", is_curve=" << is_curve 
              << ", is_in_place=" << is_in_place 
              << ", is_stopped=" << is_stopped << std::endl;

    int left_percent = 0, right_percent = 0;
    int left_percent_original = 0, right_percent_original = 0;
    int min_threshold = 0;
    double max_wheel_velocity = 1.0;
    double max_percentage = 0.0;

    if (is_stopped) {
        // Stopped - stop the motors immediately
        left_percent = 0;
        right_percent = 0;
        left_percent_original = 0;
        right_percent_original = 0;
        max_wheel_velocity = 0.0;
        max_percentage = 0.0;
        min_threshold = 0;
        
        hat_->stop();
        is_stopped_ = true;
        
        // Log stop command
        std::cout << "=== MOTOR STOP COMMAND ===" << std::endl;
        std::cout << "Input velocities (rad/s): Left=" << left << ", Right=" << right << std::endl;
        std::cout << "Motion: STOPPED" << std::endl;
        std::cout << "=========================" << std::endl << std::flush;
        return;
        
    } else if (is_in_place) {
        // In-place spin: optimized for tested values
        double angular_mag = std::min(std::abs(left), 0.5053951031089063);  // Optimal turn value
        int static_turn_power = static_cast<int>((angular_mag / 0.5053951031089063) * 30); // Scale to 30% max
        if (static_turn_power < 15) static_turn_power = 15; // Minimum 15% for reliable turning

        left_percent = (left < 0) ? -static_turn_power : static_turn_power;
        right_percent = (right < 0) ? -static_turn_power : static_turn_power;

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        max_wheel_velocity = 0.5053951031089063; // Use optimal turn value for logging
        max_percentage = static_cast<double>(static_turn_power); // debug only
        min_threshold = 0;

    } else if (is_pure_forward) {
        // Pure straight motion (i and , keys)
        max_wheel_velocity = 1.0717944050000008;  // Optimal speed based on testing
        max_percentage = 40.0;
        min_threshold = 20;
        
        // For pure forward/backward, use the average and apply to both wheels equally
        double avg_speed = (left + right) / 2.0;
        double scaled_speed = std::max(-1.0, std::min(1.0, avg_speed / max_wheel_velocity));
        
        left_percent = static_cast<int>(scaled_speed * max_percentage);
        right_percent = static_cast<int>(scaled_speed * max_percentage);
        
        left_percent_original = left_percent;
        right_percent_original = right_percent;

        // Apply minimum power threshold
        if (std::abs(left_percent) > 0 && std::abs(left_percent) < min_threshold)
            left_percent = (left_percent > 0) ? min_threshold : -min_threshold;
        if (std::abs(right_percent) > 0 && std::abs(right_percent) < min_threshold)
            right_percent = (right_percent > 0) ? min_threshold : -min_threshold;
            
    } else if (is_curve) {
        // Curve/arc motion (u, o, m, . keys) - PRESERVE INDIVIDUAL WHEEL SPEEDS
        max_wheel_velocity = 1.0717944050000008;  // Optimal speed based on testing
        max_percentage = 50.0;  // Slightly higher for curves
        min_threshold = 20;
        
        // Apply scaled output - KEEP LEFT AND RIGHT SEPARATE
        double left_scaled = std::max(-1.0, std::min(1.0, left / max_wheel_velocity));
        double right_scaled = std::max(-1.0, std::min(1.0, right / max_wheel_velocity));

        left_percent = static_cast<int>(left_scaled * max_percentage);
        right_percent = static_cast<int>(right_scaled * max_percentage);

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        // Apply minimum power threshold
        if (std::abs(left_percent) > 0 && std::abs(left_percent) < min_threshold)
            left_percent = (left_percent > 0) ? min_threshold : -min_threshold;
        if (std::abs(right_percent) > 0 && std::abs(right_percent) < min_threshold)
            right_percent = (right_percent > 0) ? min_threshold : -min_threshold;
    } else {
        // We have a movement command
        is_stopped_ = false;
    }

    // === Logging ===
    std::cout << "=== MOTOR COMMAND RECEIVED ===" << std::endl;
    std::cout << "Input velocities (rad/s): Left=" << left << ", Right=" << right << std::endl;
    std::cout << "Motion: "
              << (is_in_place ? "IN_PLACE_TURN"
                              : (is_pure_forward ? "STRAIGHT"
                                                 : (is_curve ? "CURVE" : "OTHER")))
              << std::endl;
    std::cout << "Scaling: max_velocity=" << max_wheel_velocity
              << ", max_percent=" << max_percentage << "%" << std::endl;
    std::cout << "Original %: Left=" << left_percent_original
              << "%, Right=" << right_percent_original << "%" << std::endl;
    std::cout << "Min threshold: " << min_threshold << "%" << std::endl;
    std::cout << "Final %: Left=" << left_percent
              << "%, Right=" << right_percent << "%" << std::endl;
    std::cout << "=============================" << std::endl << std::flush;

    hat_->set_wheel_speeds(left_percent, right_percent);
}




void MotorDriver::stop() {
    hat_->stop();
    is_stopped_ = true;
}

void MotorDriver::check_command_timeout() {
    if (!is_stopped_) {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_);
        
        if (duration.count() > (COMMAND_TIMEOUT_SECONDS * 1000)) {
            std::cout << "=== COMMAND TIMEOUT - STOPPING ROBOT ===" << std::endl;
            std::cout << "No command received for " << duration.count() << "ms" << std::endl;
            std::cout << "=======================================" << std::endl << std::flush;
            
            hat_->stop();
            is_stopped_ = true;
        }
    }
}