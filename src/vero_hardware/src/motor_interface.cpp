#include "vero_hardware/motor_interface.hpp"

MotorDriverHAT::MotorDriverHAT(const std::string& i2c_device, uint8_t addr)
    : i2c_fd_(-1), pca9685_addr_(addr), initialized_(false)
{
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        throw std::runtime_error("Failed to open I2C device");
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, pca9685_addr_) < 0) {
        close(i2c_fd_);
        throw std::runtime_error("Failed to set I2C address");
    }
}

MotorDriverHAT::~MotorDriverHAT() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

bool MotorDriverHAT::initialize(uint16_t pwm_freq) {
    try {
        writeByte(MODE1, 0x00); // Normal mode
        setPWMFrequency(pwm_freq);
        initialized_ = true;
        return true;
    } catch (...) {
        initialized_ = false;
        return false;
    }
}

void MotorDriverHAT::set_wheel_speeds(int left_percent, int right_percent) {
    // Clamp input
    left_percent = std::max(-100, std::min(100, left_percent));
    right_percent = std::max(-100, std::min(100, right_percent));

    // HARDWARE FIX: Motors are connected to swapped channels
    // Left motors (Channel A) - was Channel B
    runMotor(MOTOR_A, left_percent);

    // Right motors (Channel B) - was Channel A
    runMotor(MOTOR_B, right_percent);

    // Stub feedback (no encoders)
    left_velocity_ = left_percent / 100.0;
    right_velocity_ = right_percent / 100.0;
    // Debug output
    std::cout << "Set wheel speeds: L=" << left_percent << "%, R=" << right_percent << "%" << std::endl;
}

void MotorDriverHAT::stop() {
    stopMotor(MOTOR_A);
    stopMotor(MOTOR_B);
    left_velocity_ = 0.0;
    right_velocity_ = 0.0;
}

// --- Low-level PCA9685 helpers ---

void MotorDriverHAT::writeByte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(i2c_fd_, buf, 2) != 2) {
        throw std::runtime_error("Failed to write byte to I2C");
    }
}

uint8_t MotorDriverHAT::readByte(uint8_t reg) {
    if (write(i2c_fd_, &reg, 1) != 1) {
        throw std::runtime_error("Failed to write reg for read");
    }
    uint8_t value;
    if (read(i2c_fd_, &value, 1) != 1) {
        throw std::runtime_error("Failed to read byte from I2C");
    }
    return value;
}

void MotorDriverHAT::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    writeByte(LED0_ON_L + 4 * channel, on & 0xFF);
    writeByte(LED0_ON_H + 4 * channel, on >> 8);
    writeByte(LED0_OFF_L + 4 * channel, off & 0xFF);
    writeByte(LED0_OFF_H + 4 * channel, off >> 8);
}

void MotorDriverHAT::setPWMFrequency(uint16_t freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= static_cast<float>(freq);
    prescaleval -= 1.0;
    uint8_t prescale = static_cast<uint8_t>(prescaleval + 0.5);

    uint8_t oldmode = readByte(MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    writeByte(MODE1, newmode);
    writeByte(PRESCALE, prescale);
    writeByte(MODE1, oldmode);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    writeByte(MODE1, oldmode | 0x80); // restart
}

void MotorDriverHAT::setPWMDutyCycle(uint8_t channel, uint16_t duty_cycle) {
    duty_cycle = std::min<uint16_t>(duty_cycle, 4095);
    setPWM(channel, 0, duty_cycle);
}

void MotorDriverHAT::setLevel(uint8_t channel, uint16_t level) {
    if (level)
        setPWM(channel, 4096, 0);
    else
        setPWM(channel, 0, 4096);
}

// --- Motor control helpers ---

void MotorDriverHAT::runMotor(uint8_t motor, int speed_percent) {
    uint8_t pwm, in1, in2;
    if (motor == MOTOR_A) {
        pwm = PWMA; in1 = AIN1; in2 = AIN2;
    } else {
        pwm = PWMB; in1 = BIN1; in2 = BIN2;
    }

    int speed = std::abs(speed_percent);
    speed = std::min(speed, 100);
    uint16_t duty = static_cast<uint16_t>(speed * 40.95); // 100% = 4095

    if (speed_percent > 0) {
        setLevel(in1, 1);
        setLevel(in2, 0);
        setPWMDutyCycle(pwm, duty);
    } else if (speed_percent < 0) {
        setLevel(in1, 0);
        setLevel(in2, 1);
        setPWMDutyCycle(pwm, duty);
    } else {
        setLevel(in1, 0);
        setLevel(in2, 0);
        setPWMDutyCycle(pwm, 0);
    }
}

void MotorDriverHAT::stopMotor(uint8_t motor) {
    uint8_t pwm, in1, in2;
    if (motor == MOTOR_A) {
        pwm = PWMA; in1 = AIN1; in2 = AIN2;
    } else {
        pwm = PWMB; in1 = BIN1; in2 = BIN2;
    }
    setLevel(in1, 0);
    setLevel(in2, 0);
    setPWMDutyCycle(pwm, 0);
}