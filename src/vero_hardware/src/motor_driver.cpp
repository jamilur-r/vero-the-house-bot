#include "vero_hardware/motor_driver.hpp"
#include "vero_hardware/motor_interface.hpp"  // Include your low-level HAT driver

// You may want to store a pointer or instance to your MotorDriverHAT
MotorDriver::MotorDriver(int address)
    : address_(address)
{
    // Initialize the MotorDriverHAT (using default I2C device)
    hat_ = std::make_shared<MotorDriverHAT>("/dev/i2c-1", static_cast<uint8_t>(address_));
    if (!hat_->initialize(100)) {
        throw std::runtime_error("Failed to initialize MotorDriverHAT");
    }
}

void MotorDriver::set_wheel_velocities(double left, double right) {
    // === Detect motion type ===
    bool is_in_place = (std::abs(left + right) < 1e-3) && (std::abs(left - right) > 1e-3);
    bool is_pure_forward = std::abs(left - right) < 1e-2;
    bool is_curve = !is_in_place && !is_pure_forward;
    bool is_stopped = std::abs(left) < 1e-3 && std::abs(right) < 1e-3;

    int left_percent = 0, right_percent = 0;
    int left_percent_original = 0, right_percent_original = 0;
    int min_threshold = 0;
    double max_wheel_velocity = 1.0;
    double max_percentage = 0.0;

    if (is_in_place) {
        // In-place spin: aggressive power
        double angular_mag = std::min(std::abs(left), 1.0);
        int static_turn_power = static_cast<int>(angular_mag * 100);
        if (static_turn_power < 50) static_turn_power = 50;

        left_percent = (left < 0) ? -static_turn_power : static_turn_power;
        right_percent = (right < 0) ? -static_turn_power : static_turn_power;

        left_percent_original = left_percent;
        right_percent_original = right_percent;

        max_wheel_velocity = 0.0; // not used
        max_percentage = static_cast<double>(static_turn_power); // debug only
        min_threshold = 0;

    } else if (is_pure_forward) {
        // Straight motion
        max_wheel_velocity = 1.0;
        max_percentage = 40.0;
        min_threshold = 20;
    } else if (is_curve) {
        // Curve/arc motion: blended power
        max_wheel_velocity = 1.0;
        max_percentage = 50.0;
        min_threshold = 20;
    }

    // Apply scaled output for curve or forward
    if (!is_in_place && !is_stopped) {
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
    }

    // === Logging ===
    std::cout << "=== MOTOR COMMAND RECEIVED ===" << std::endl;
    std::cout << "Input velocities (rad/s): Left=" << left << ", Right=" << right << std::endl;
    std::cout << "Motion: "
              << (is_in_place ? "IN_PLACE_TURN"
                              : (is_pure_forward ? "STRAIGHT"
                                                 : (is_curve ? "CURVE" : "STOPPED")))
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
`



void MotorDriver::stop() {
    hat_->stop();
}