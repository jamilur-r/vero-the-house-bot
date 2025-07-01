# Drive Mechanics - Distributed ROS2 Robot Control

A ROS2 package for controlling differential drive robots using Waveshare Motor Driver Hat with PCA9685 in a distributed architecture.

## System Architecture

### Pi Zero (Hardware Interface)
- **Motor control** via PCA9685 PWM driver
- **Real-time** motor commands processing
- **Diagnostics** and status publishing
- **Lightweight** operation for embedded system

### PC (Navigation & Control)
- **Nav2** autonomous navigation
- **RViz** visualization and monitoring  
- **Teleop** remote control
- **Path planning** and mapping

## Quick Start

### 1. Pi Zero Setup
```bash
# Build and install
cd /home/xlab/projects/vero_ws
colcon build --packages-select drive_mechanics
source install/setup.bash

# Run hardware interface
ros2 launch drive_mechanics pi_zero.launch.py
```

### 2. PC Setup
```bash
# Install and build on PC
git clone <your-repo> ~/ros2_ws/src/drive_mechanics
cd ~/ros2_ws
colcon build --packages-select drive_mechanics
source install/setup.bash

# Run teleop interface
ros2 launch drive_mechanics pc.launch.py enable_teleop:=true

# Or run with nav2
ros2 launch drive_mechanics pc.launch.py enable_nav2:=true enable_rviz:=true
```

## Network Configuration

Both systems must be on the same network with matching ROS_DOMAIN_ID:
```bash
export ROS_DOMAIN_ID=0
```

See `docs/NETWORK_SETUP.md` for detailed network configuration.

## Robot Specifications

- **Wheel diameter:** 75mm
- **Wheel separation:** 89mm  
- **Drive type:** Differential (4-wheel skid-steer)
- **Motor control:** PCA9685 PWM driver
- **Max speed:** 0.3 m/s (configurable)

## Topics

### Pi Zero Publishers
- `/odom` - Robot odometry
- `/diagnostics` - System diagnostics
- `/motor_status` - Motor feedback

### Pi Zero Subscribers  
- `/cmd_vel` - Velocity commands
- `/emergency_stop` - Safety stop

## Launch Files

- `pi_zero.launch.py` - Hardware interface for Pi Zero
- `pc.launch.py` - Navigation and control for PC
- `drive_mechanics.launch.py` - Combined (single machine)
- `drive_mechanics_minimal.launch.py` - Minimal setup

## Configuration

Edit `config/motor_config.yaml` to adjust:
- Motor channel mapping
- Speed limits and safety parameters
- Robot physical dimensions
- Nav2 integration parameters

## Dependencies

### Pi Zero
- `rclpy`, `geometry_msgs`, `std_msgs`
- `smbus2` for I2C communication
- `diagnostic_msgs` for system monitoring

### PC  
- Full ROS2 desktop installation
- `nav2_bringup` (for navigation)
- `rviz2` (for visualization)
- `teleop_twist_keyboard` (for manual control)

## Testing

### Manual Control
```bash
# Test motor movement
ros2 run drive_mechanics motor_test

# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Network Verification
```bash
# Check topics across network
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

## Troubleshooting

1. **Network Issues:** Check `docs/NETWORK_SETUP.md`
2. **Motor Problems:** Run `ros2 run drive_mechanics motor_test`
3. **I2C Errors:** Verify connections and enable I2C on Pi Zero

## Contributing

1. Fork the repository
2. Create feature branch
3. Test on both Pi Zero and PC
4. Submit pull request

## License

Apache License 2.0
