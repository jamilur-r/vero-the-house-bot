#pragma once

#include "vero_hardware/motor_interface.hpp"
#include <memory>

class MotorDriver {
public: 
    MotorDriver(int address);
    void set_wheel_velocities(double left, double right);
    void stop();

private:
    int address_;
    std::shared_ptr<MotorDriverHAT> hat_;
};

