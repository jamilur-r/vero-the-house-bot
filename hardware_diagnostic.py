#!/usr/bin/env python3

import time
import sys
import os

# Add the build path to find the Python module
sys.path.append('/home/xlab/projects/vero_ws/build/vero_hardware')

print("🔧 Vero Hardware Diagnostic Tool 🔧")
print("=" * 50)

# Test 1: I2C Connection
print("\n1. Testing I2C Connection...")
try:
    result = os.system("i2cdetect -y 1 | grep -q '40'")
    if result == 0:
        print("✅ I2C device found at address 0x40")
    else:
        print("❌ I2C device NOT found at address 0x40")
        print("   Check I2C connections and power supply")
        exit(1)
except Exception as e:
    print(f"❌ I2C test failed: {e}")

# Test 2: Direct Motor Control
print("\n2. Testing Direct Motor Control...")
print("This will test each motor individually with different speeds")
print("Watch the robot carefully - motors should move at different speeds")

try:
    # Try to import and test the motor interface directly
    import rclpy
    from geometry_msgs.msg import Twist
    
    rclpy.init()
    
    # Create a simple publisher
    from rclpy.node import Node
    
    class MotorTester(Node):
        def __init__(self):
            super().__init__('motor_tester')
            self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
            
        def test_sequence(self):
            tests = [
                ("Stop", 0.0, 0.0),
                ("Forward Straight", 0.1, 0.0),
                ("Forward Left Curve", 0.1, 0.3),
                ("Forward Right Curve", 0.1, -0.3),
                ("In-place Left Turn", 0.0, 0.5),
                ("In-place Right Turn", 0.0, -0.5),
                ("Backward Straight", -0.1, 0.0),
                ("Backward Left Curve", -0.1, -0.3),
                ("Backward Right Curve", -0.1, 0.3),
                ("Stop", 0.0, 0.0),
            ]
            
            for i, (description, linear, angular) in enumerate(tests):
                print(f"\nTest {i+1}/10: {description}")
                print(f"  Command: linear.x={linear}, angular.z={angular}")
                
                # Calculate expected wheel speeds
                wheel_separation = 0.0916
                wheel_radius = 0.036
                left_wheel_expected = (linear - angular * wheel_separation / 2.0) / wheel_radius
                right_wheel_expected = (linear + angular * wheel_separation / 2.0) / wheel_radius
                
                print(f"  Expected: left_wheel={left_wheel_expected:.3f}, right_wheel={right_wheel_expected:.3f}")
                
                # Publish command
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                self.publisher.publish(twist)
                
                # Wait and observe
                input(f"  Press ENTER when you've observed the motion (or lack thereof)...")
    
    tester = MotorTester()
    tester.test_sequence()
    
    # Final stop
    twist = Twist()
    tester.publisher.publish(twist)
    
    rclpy.shutdown()
    
    print("\n" + "=" * 50)
    print("3. Hardware Diagnosis Questions:")
    print("=" * 50)
    
    print("\nBased on the tests above, please answer:")
    print("a) Did BOTH motors move during 'Forward Straight'? (y/n):")
    both_move = input().lower().startswith('y')
    
    print("b) During 'Forward Left Curve', did the RIGHT motor move faster? (y/n):")
    right_faster = input().lower().startswith('y')
    
    print("c) During 'Forward Right Curve', did the LEFT motor move faster? (y/n):")
    left_faster = input().lower().startswith('y')
    
    print("d) Did the 'In-place' turns work correctly? (y/n):")
    turns_work = input().lower().startswith('y')
    
    print("\n" + "=" * 50)
    print("4. Diagnosis Results:")
    print("=" * 50)
    
    if not both_move:
        print("❌ PROBLEM: One or both motors not responding")
        print("   Possible causes:")
        print("   - Power supply insufficient or disconnected")
        print("   - Motor driver HAT not properly connected")
        print("   - Faulty motor or wiring")
        print("   - I2C communication issues")
        
    elif not (right_faster and left_faster):
        print("❌ PROBLEM: Differential drive not working correctly")
        print("   Possible causes:")
        print("   - Motor channels swapped (A/B)")
        print("   - Motor wiring polarity incorrect")
        print("   - Motor driver HAT channel configuration wrong")
        print("   - Mechanical coupling issues")
        
    elif not turns_work:
        print("❌ PROBLEM: In-place turns not working")
        print("   Possible causes:")
        print("   - Motor direction configuration incorrect")
        print("   - Insufficient power for turning")
        print("   - Mechanical friction too high")
        
    else:
        print("✅ All tests passed! The hardware appears to be working correctly.")
        print("   The issue might be in:")
        print("   - Teleop key mapping (use custom teleop script)")
        print("   - Speed scaling in motor_driver.cpp")
        print("   - User perception of differential speeds")
    
    print("\n" + "=" * 50)
    print("5. Recommended Next Steps:")
    print("=" * 50)
    
    if not both_move:
        print("1. Check power supply voltage and current capacity")
        print("2. Verify I2C connections: SDA, SCL, VCC, GND")
        print("3. Test motors individually with direct HAT control")
        
    else:
        print("1. Double-check motor channel assignments in motor_interface.cpp")
        print("2. Verify motor polarity and wiring")
        print("3. Test with higher speed differences to make curves more obvious")
        print("4. Use the custom teleop script to ensure correct Twist commands")

except Exception as e:
    print(f"❌ Test failed with error: {e}")
    print("Make sure the ROS2 system is running and controllers are active")

print("\n🔧 Hardware diagnostic complete! 🔧")
