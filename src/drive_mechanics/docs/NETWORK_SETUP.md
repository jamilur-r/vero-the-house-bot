# Network Configuration for Distributed ROS2 System

## System Architecture

```
┌─────────────────┐    Network     ┌─────────────────┐
│   Pi Zero       │◄──────────────►│      PC         │
│                 │   (WiFi/Eth)   │                 │
│ - drive_node    │                │ - nav2          │
│ - motor_control │                │ - rviz          │
│ - diagnostics   │                │ - teleop        │
│                 │                │ - planning      │
└─────────────────┘                └─────────────────┘
```

## Topic Flow

### Pi Zero → PC (Publishers)
- `/odom` - Odometry data
- `/diagnostics` - System health
- `/motor_status` - Motor feedback
- `/tf` - Transform data (optional)

### PC → Pi Zero (Publishers)  
- `/cmd_vel` - Velocity commands
- `/emergency_stop` - Safety commands

## Network Setup

### 1. ROS Domain Configuration
Both systems must use the same ROS_DOMAIN_ID:

**Pi Zero:**
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

**PC:**
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 2. Network Discovery
Ensure both devices can see each other:

**Test connectivity:**
```bash
# From PC, ping Pi Zero
ping pi-zero-hostname.local

# From Pi Zero, ping PC
ping pc-hostname.local
```

### 3. Firewall Configuration

**Ubuntu/PC (if needed):**
```bash
sudo ufw allow 7400:7500/udp  # ROS2 DDS ports
sudo ufw allow 7400:7500/tcp
```

**Pi Zero (Raspberry Pi OS):**
```bash
sudo iptables -A INPUT -p udp --dport 7400:7500 -j ACCEPT
sudo iptables -A INPUT -p tcp --dport 7400:7500 -j ACCEPT
```

### 4. DDS Configuration (Optional)
For better network performance, create DDS config:

**Both systems - `~/.ros/fastdds_profile.xml`:**
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_participant" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration>
                        <sec>10</sec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

## Launch Sequence

### 1. Start Pi Zero (Hardware)
```bash
# On Pi Zero
cd /home/xlab/projects/vero_ws
source install/setup.bash
ros2 launch drive_mechanics pi_zero.launch.py
```

### 2. Start PC (Navigation/Control)
```bash
# On PC
cd /path/to/your/workspace
source install/setup.bash

# For teleop testing
ros2 launch drive_mechanics pc.launch.py enable_teleop:=true

# For nav2 integration
ros2 launch drive_mechanics pc.launch.py enable_nav2:=true enable_rviz:=true
```

## Verification Commands

### Check Topics Across Network
```bash
# On PC - should see Pi Zero topics
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /diagnostics

# On Pi Zero - should see PC commands  
ros2 topic list
ros2 topic echo /cmd_vel
```

### Test Communication
```bash
# On PC - send movement command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# On Pi Zero - should see motors moving
```

### Monitor Network
```bash
# Check ROS2 daemon
ros2 daemon status

# Monitor node graph
ros2 node list
ros2 node info /drive_mechanics_node
```

## Troubleshooting

### 1. Nodes Not Discovering Each Other
- Check `ROS_DOMAIN_ID` matches
- Verify network connectivity
- Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

### 2. High Network Latency
- Reduce publishing frequencies in config
- Use DDS configuration
- Check WiFi signal strength

### 3. Topic Publishing Issues
- Verify topic names match exactly
- Check node namespaces
- Use `ros2 topic info <topic_name>` to debug

## Performance Optimization

### Pi Zero (Lightweight)
- Disable unnecessary diagnostics
- Lower publishing frequencies
- Minimal logging
- Use compiled optimizations

### PC (Full Features)
- Full nav2 stack
- High-frequency planning
- Rich visualization
- Comprehensive logging

## Security Considerations

### 1. Network Security
- Use WPA3 WiFi encryption
- Consider VPN for remote operation
- Isolate robot network if possible

### 2. ROS2 Security
- Enable ROS2 security if needed
- Use SROS2 for production deployments
- Consider DDS security plugins
