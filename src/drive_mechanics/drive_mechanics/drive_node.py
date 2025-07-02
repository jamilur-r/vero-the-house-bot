import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

from drive_mechanics.motor_controller import MotorController


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_mechanics_node')  # Match the config file

        # Declare parameters with default values
        self.declare_parameter('motor_control.max_speed', 0.6)
        self.declare_parameter('safety.max_linear_velocity', 0.3)
        self.declare_parameter('safety.max_angular_velocity', 2.0)
        self.declare_parameter('robot.wheel_separation', 0.089)

        # Angular scaling parameters
        self.declare_parameter(
            'motor_control.angular_scaling.enable_scaling', True)
        self.declare_parameter(
            'motor_control.angular_scaling.in_place_scale_factor', 4.0)
        self.declare_parameter(
            'motor_control.angular_scaling.mixed_motion_scale', 2.0)
        self.declare_parameter(
            'motor_control.angular_scaling.linear_threshold', 0.01)

        # Get parameter values with fallback defaults
        self.max_speed = self.get_parameter(
            'motor_control.max_speed').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter(
            'safety.max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter(
            'safety.max_angular_velocity').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            'robot.wheel_separation').get_parameter_value().double_value

        # Angular scaling parameters
        self.enable_angular_scaling = self.get_parameter(
            'motor_control.angular_scaling.enable_scaling').get_parameter_value().bool_value
        self.in_place_scale_factor = self.get_parameter(
            'motor_control.angular_scaling.in_place_scale_factor').get_parameter_value().double_value
        self.mixed_motion_scale = self.get_parameter(
            'motor_control.angular_scaling.mixed_motion_scale').get_parameter_value().double_value
        self.linear_threshold = self.get_parameter(
            'motor_control.angular_scaling.linear_threshold').get_parameter_value().double_value

        self.motor_controller = MotorController()

        # QoS settings - use default for compatibility with teleop_twist_keyboard
        qos_profile = QoSProfile(depth=10)

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        self.get_logger().info("DriveNode initialized and subscribed to cmd_vel")
        self.get_logger().info(f"Parameters - max_speed: {self.max_speed}, "
                               f"max_linear_vel: {self.max_linear_vel}, "
                               f"max_angular_vel: {self.max_angular_vel}, "
                               f"wheel_separation: {self.wheel_separation}")
        self.get_logger().info(f"Angular scaling - enabled: {self.enable_angular_scaling}, "
                               f"in_place_factor: {self.in_place_scale_factor}, "
                               f"mixed_motion_scale: {self.mixed_motion_scale}")

    def cmd_vel_callback(self, msg: Twist):
        # Apply velocity limits
        linear_x = max(
            min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(
            min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # Calculate wheel speeds with improved angular scaling
        left_speed_percent, right_speed_percent = self.calculate_wheel_speeds(
            linear_x, angular_z)

        # Send to motor controller (note: you had left/right swapped, keeping that)
        self.motor_controller.set_motor_speed(
            right_speed_percent, left_speed_percent)

        # Enhanced logging
        motion_type = "in-place" if abs(
            linear_x) < self.linear_threshold else "mixed" if angular_z != 0 else "linear"
        self.get_logger().info(f'Cmd: linear={linear_x:.2f}, angular={angular_z:.2f}, '
                               f'left_cmd={left_speed_percent:.1f}%, right_cmd={right_speed_percent:.1f}% [{motion_type}]')

    def calculate_wheel_speeds(self, linear_x, angular_z):
        """Calculate wheel speeds with configurable angular scaling"""

        # Determine motion type and scaling factor
        is_in_place = abs(linear_x) < self.linear_threshold

        if not self.enable_angular_scaling:
            # Original differential drive calculation without scaling
            angular_contribution = angular_z * (self.wheel_separation / 2.0)
            left_speed = linear_x - angular_contribution
            right_speed = linear_x + angular_contribution

            # Convert to percentage
            left_speed_percent = (left_speed / self.max_linear_vel) * 80.0
            right_speed_percent = (right_speed / self.max_linear_vel) * 80.0

        elif is_in_place:
            # Pure rotation with enhanced scaling
            angular_percent = (angular_z / self.max_angular_vel) * 100.0
            scaling_factor = self.in_place_scale_factor

            left_speed_percent = -angular_percent * scaling_factor
            right_speed_percent = angular_percent * scaling_factor

        else:
            # Mixed movement with differential drive + scaling
            angular_contribution = angular_z * (self.wheel_separation / 2.0)
            left_speed = linear_x - angular_contribution
            right_speed = linear_x + angular_contribution

            # Apply mixed motion scaling if there's significant angular component
            if abs(angular_z) > 0.1:
                angular_dominance = abs(angular_z) / (abs(linear_x) + 0.01)
                scale_factor = 1.0 + \
                    (angular_dominance * (self.mixed_motion_scale - 1.0))
            else:
                scale_factor = 1.0

            # Convert to percentage with scaling
            left_speed_percent = (
                left_speed / self.max_linear_vel) * 80.0 * scale_factor
            right_speed_percent = (
                right_speed / self.max_linear_vel) * 80.0 * scale_factor

        # Apply global max_speed multiplier
        left_speed_percent *= self.max_speed
        right_speed_percent *= self.max_speed

        # Clamp to motor limits
        left_speed_percent = max(min(left_speed_percent, 100.0), -100.0)
        right_speed_percent = max(min(right_speed_percent, 100.0), -100.0)

        return left_speed_percent, right_speed_percent


def main(args=None):
    rclpy.init(args=args)
    drive_node = DriveNode()
    rclpy.spin(drive_node)

    # Cleanup
    drive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# This code is a ROS2 node that subscribes to the 'cmd_vel' topic and controls the motors based on the received Twist messages.
