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
    last_command_time_ = std::chrono::steady_clock::now();

    // === Classify Motion Type ===
    bool is_stopped = std::abs(left) < 0.01 && std::abs(right) < 0.01;

    bool same_direction = (left * right > 0); // both positive or both negative
    bool similar_magnitude = std::abs(left - right) < 0.15;

    bool is_pure_straight = same_direction && similar_magnitude;  // forward or backward
    bool is_in_place = !same_direction && std::abs(left + right) < 0.1;
    bool is_curve = !is_stopped && !is_in_place && !is_pure_straight;

    int left_percent = 0, right_percent = 0;
    int left_percent_original = 0, right_percent_original = 0;
    int min_threshold = 0;
    double max_wheel_velocity = 1.0;
    double max_percentage = 0.0;

    if (is_stopped) {
        // STOP
        left_percent = 0;
        right_percent = 0;
        max_wheel_velocity = 0.0;
        max_percentage = 0.0;
        min_threshold = 0;

        hat_->stop();
        is_stopped_ = true;

        std::cout << "=== MOTOR STOP COMMAND ===" << std::endl;
        std::cout << "Input velocities (rad/s): Left=" << left << ", Right=" << right << std::endl;
        std::cout << "Motion: STOPPED" << std::endl;
        std::cout << "=========================" << std::endl << std::flush;
        return;

    } else if (is_in_place) {
        // IN-PLACE TURN
        double angular_mag = std::min(std::abs(left), 0.5053951031089063);
        int static_turn_power = static_cast<int>((angular_mag / 0.5053951031089063) * 30);
        if (static_turn_power < 15) static_turn_power = 15;

        left_percent = (left < 0) ? -static_turn_power : static_turn_power;
        right_percent = (right < 0) ? -static_turn_power : static_turn_power;

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        max_wheel_velocity = 0.5053951031089063;
        max_percentage = static_cast<double>(static_turn_power);
        min_threshold = 0;

    } else if (is_pure_straight) {
        // FORWARD or BACKWARD motion
        max_wheel_velocity = 1.0717944050000008;
        max_percentage = 40.0;
        min_threshold = 20;

        double avg_speed = (left + right) / 2.0;
        double scaled_speed = std::max(-1.0, std::min(1.0, avg_speed / max_wheel_velocity));

        left_percent = static_cast<int>(scaled_speed * max_percentage);
        right_percent = static_cast<int>(scaled_speed * max_percentage);

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        if (std::abs(left_percent) > 0 && std::abs(left_percent) < min_threshold)
            left_percent = (left_percent > 0) ? min_threshold : -min_threshold;
        if (std::abs(right_percent) > 0 && std::abs(right_percent) < min_threshold)
            right_percent = (right_percent > 0) ? min_threshold : -min_threshold;

    } else if (is_curve) {
        // CURVE / ARC
        max_wheel_velocity = 1.0717944050000008;
        max_percentage = 50.0;
        min_threshold = 20;

        double left_scaled = std::max(-1.0, std::min(1.0, left / max_wheel_velocity));
        double right_scaled = std::max(-1.0, std::min(1.0, right / max_wheel_velocity));

        left_percent = static_cast<int>(left_scaled * max_percentage);
        right_percent = static_cast<int>(right_scaled * max_percentage);

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        if (std::abs(left_percent) > 0 && std::abs(left_percent) < min_threshold)
            left_percent = (left_percent > 0) ? min_threshold : -min_threshold;
        if (std::abs(right_percent) > 0 && std::abs(right_percent) < min_threshold)
            right_percent = (right_percent > 0) ? min_threshold : -min_threshold;
    }

    is_stopped_ = false;

    // === LOGGING ===
    std::cout << "=== MOTOR COMMAND RECEIVED ===" << std::endl;
    std::cout << "Input velocities (rad/s): Left=" << left << ", Right=" << right << std::endl;
    std::cout << "Motion: "
              << (is_in_place ? "IN_PLACE_TURN"
                              : (is_pure_straight ? "STRAIGHT"
                                                  : (is_curve ? "CURVE" : "UNKNOWN")))
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