# 🤖 Vero The House Bot 🚀

Welcome to **Vero**, your friendly neighborhood house bot!  
She moves, she grooves, and now she responds to your keyboard commands!  
Let's get this party started! 🎉

---

## 🛠️ Hardware Setup

### Required Hardware:
- **Raspberry Pi 4** (or compatible)
- **Motor Driver HAT** (PCA9685-based, default I2C address 0x40)
- **DC Motors** (connected to Motor A and Motor B channels)
- **Power Supply** for motors
- **I2C enabled** on Raspberry Pi

### I2C Configuration:
```bash
# Enable I2C on Raspberry Pi
sudo raspi-config
# Navigate to: Interface Options > I2C > Enable

# Verify I2C devices
i2cdetect -y 1
# Should show device at address 0x40
```

---

## 🚀 Building and Deployment

### 1. Build the Workspace
```bash
cd /home/xlab/projects/vero_ws
colcon build
source install/setup.bash
```

### 2. Deploy to Raspberry Pi
```bash
# From your development machine
./deploy.sh
```

### 3. Build on Raspberry Pi
```bash
# SSH into Raspberry Pi
ssh vero@vero.local

# Navigate to workspace and build
cd /home/vero/projects/vero_ws
colcon build
source install/setup.bash
```

---

## 🏁 Starting the Robot

### 1. Launch the Hardware Interface
```bash
# On Raspberry Pi
ros2 launch vero_hardware vero_robot.launch.py
```

### 2. Start Teleop Keyboard Control
```bash
# On your development machine or Pi
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---

## 🎮 Keyboard Controls

### Movement Keys:
```
        u    i    o
        j    k    l
        m    ,    .
```

**Movement Mapping:**
- **u, i, o**: Forward movement (u=forward-left, i=forward, o=forward-right)
- **j, k, l**: Turning (j=turn-left, k=stop, l=turn-right)
- **m, ,, .**: Backward movement (m=backward-left, ,=backward, .=backward-right)

### Speed Control:
- **q/z**: Increase/decrease linear speed
- **w/x**: Increase/decrease angular speed
- **k**: Emergency stop
- **Ctrl+C**: Quit

### Safety Features:
- **Auto-stop**: Robot automatically stops after 500ms of no new commands
- **Press and hold**: Hold keys for continuous movement, release to stop

---

## 🔧 Technical Details

### Motor Control Features:
- **PWM-based motor control** (0-4095 range)
- **Motion type detection**: In-place turns, straight movement, curves
- **Command timeout safety**: Auto-stops after 500ms of no commands
- **Optimized speed values**: 
  - Max linear velocity: 1.07 rad/s
  - Max angular velocity: 0.51 rad/s

### Control Architecture:
1. **ROS2 Control Framework**: `diff_drive_controller`
2. **Hardware Interface**: `VeroHardwareInterface`
3. **Motor Driver**: PWM control via I2C
4. **Safety Features**: Command timeout, error handling

---

## 📡 ROS2 Topics

### Published Topics:
- `/joint_states`: Joint position and velocity feedback
- `/vero_hardware/status`: Hardware status information

### Subscribed Topics:
- `/diff_drive_controller/cmd_vel_unstamped`: Velocity commands from teleop

### Manual Control:
```bash
# Move forward
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Turn left
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.3}}'

# Stop
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

---

## � Troubleshooting

### Common Issues:

1. **Robot doesn't move**: 
   - Check I2C connection: `i2cdetect -y 1`
   - Verify motor power supply
   - Check hardware interface logs

2. **Robot keeps moving after key release**:
   - Command timeout feature should auto-stop after 500ms
   - Press 'k' key for emergency stop

3. **Jerky movement**:
   - Adjust speed scaling in `motor_driver.cpp`
   - Check power supply stability

4. **Build errors**:
   - Ensure all dependencies installed: `rosdep install --from-paths src --ignore-src -r -y`
   - Check ROS2 Humble installation

---

## 🛠️ Development

### Useful Commands:

#### Creating a new package:
```bash
cd src
ros2 pkg create --build-type ament_cmake <package_name>
```

#### Building specific package:
```bash
colcon build --packages-select vero_hardware
```

#### Checking topics:
```bash
ros2 topic list
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
```

#### Node debugging:
```bash
ros2 node list
ros2 node info /vero_hardware_interface
```

---

## 🐸 Bonus Meme

> "Why did the robot cross the road?  
> To get to the ROS-side!"

---

Happy botting! If Vero starts plotting world domination, just press 'k' for emergency stop! 🤖✨
