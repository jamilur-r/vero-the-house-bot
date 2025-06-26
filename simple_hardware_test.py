#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

print("🔍 Hardware Debug Test 🔍")
print("This will send specific commands to help diagnose hardware issues")
print("Watch your robot carefully during each test")
print()

rclpy.init()

class HardwareTester(Node):
    def __init__(self):
        super().__init__('hardware_tester')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        time.sleep(1)  # Allow publisher to connect

    def send_command(self, linear_x, angular_z, description):
        print(f"Test: {description}")
        print(f"  Sending: linear.x={linear_x}, angular.z={angular_z}")
        
        # Calculate expected wheel speeds for reference
        wheel_separation = 0.0916
        wheel_radius = 0.036
        left_expected = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
        right_expected = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius
        print(f"  Expected wheel speeds: left={left_expected:.3f}, right={right_expected:.3f}")
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        
        print("  Watch the robot now...")
        time.sleep(3)
        
        # Stop
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        time.sleep(1)
        print()

def main():
    tester = HardwareTester()
    
    # Test sequence
    tests = [
        (0.1, 0.0, "Forward Straight - Both wheels should move at same speed"),
        (0.1, 0.5, "Forward Left Turn - Right wheel should be faster"),
        (0.1, -0.5, "Forward Right Turn - Left wheel should be faster"),
        (0.0, 0.3, "In-place Left Turn - Wheels should turn opposite directions"),
        (0.0, -0.3, "In-place Right Turn - Wheels should turn opposite directions"),
        (-0.1, 0.0, "Backward Straight - Both wheels should move backward at same speed"),
    ]
    
    for linear, angular, description in tests:
        input(f"Press ENTER to start: {description}")
        tester.send_command(linear, angular, description)
        
        response = input("Did the motion match the expected behavior? (y/n/describe): ").strip()
        if response.lower().startswith('n'):
            print("❌ Issue detected! This will help narrow down the problem.")
        elif not response.lower().startswith('y'):
            print(f"Note: {response}")
        print("-" * 60)
    
    print("\n🔧 HARDWARE DIAGNOSIS COMPLETE 🔧")
    print("\nCommon Issues:")
    print("1. If both wheels always move at same speed:")
    print("   - Check motor channel wiring (A/B might be swapped)")
    print("   - Verify motor driver HAT connections")
    print("   - Check if both motors are connected to same channel")
    
    print("\n2. If one wheel doesn't move:")
    print("   - Check power supply")
    print("   - Verify motor connections")
    print("   - Test motor driver HAT channels individually")
    
    print("\n3. If wheels move in wrong directions:")
    print("   - Check motor polarity (swap motor wires)")
    print("   - Verify motor channel assignment in code")
    
    print("\n4. If speeds are too slow/fast:")
    print("   - Adjust power scaling in motor_driver.cpp")
    print("   - Check power supply voltage")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
