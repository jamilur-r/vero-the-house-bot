#pragma once

#include <iostream>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>

class MotorDriverHAT {
public:
    MotorDriverHAT(const std::string& i2c_device = "/dev/i2c-1", uint8_t addr = 0x40);
    ~MotorDriverHAT();

    // Initialize the driver (call once, e.g. in configure())
    bool initialize(uint16_t pwm_freq = 50);

    // Set wheel velocities as a percentage (-100 to 100)
    void set_wheel_speeds(int left_percent, int right_percent);

    // Stop both wheels
    void stop();

    // Optionally: stub feedback for ros2_control (no encoders)
    double get_left_position() const { return left_position_; }
    double get_right_position() const { return right_position_; }
    double get_left_velocity() const { return left_velocity_; }
    double get_right_velocity() const { return right_velocity_; }

private:
    int i2c_fd_;
    uint8_t pca9685_addr_;
    bool initialized_;

    // Feedback state (stubbed)
    double left_position_ = 0.0;
    double right_position_ = 0.0;
    double left_velocity_ = 0.0;
    double right_velocity_ = 0.0;

    // --- Low-level PCA9685 helpers (as in your original code) ---
    void writeByte(uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t reg);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setPWMFrequency(uint16_t freq);
    void setPWMDutyCycle(uint8_t channel, uint16_t duty_cycle);
    void setLevel(uint8_t channel, uint16_t level);

    // --- Motor control helpers ---
    void runMotor(uint8_t motor, int speed_percent);
    void stopMotor(uint8_t motor);

    // --- Constants ---
    static constexpr uint8_t MOTOR_A = 0;
    static constexpr uint8_t MOTOR_B = 1;
    static constexpr uint8_t PWMA = 0;
    static constexpr uint8_t AIN1 = 1;
    static constexpr uint8_t AIN2 = 2;
    static constexpr uint8_t PWMB = 5;
    static constexpr uint8_t BIN1 = 3;
    static constexpr uint8_t BIN2 = 4;
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06;
    static constexpr uint8_t LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08;
    static constexpr uint8_t LED0_OFF_H = 0x09;
};