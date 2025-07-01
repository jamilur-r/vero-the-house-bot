import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from drive_mechanics.motor_controller import MotorController


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_mechanics_node')  # Match the config file

        # Declare parameters with default values
        self.declare_parameter('motor_control.max_speed', 0.6)
        self.declare_parameter('safety.max_linear_velocity', 0.3)
        self.declare_parameter('safety.max_angular_velocity', 2.0)
        self.declare_parameter('robot.wheel_separation', 0.089)

        # Get parameter values with fallback defaults
        self.max_speed = self.get_parameter(
            'motor_control.max_speed').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter(
            'safety.max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter(
            'safety.max_angular_velocity').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            'robot.wheel_separation').get_parameter_value().double_value

        self.motor_controller = MotorController()

        # QoS settings for durability
        qos_profile = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

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

    def cmd_vel_callback(self, msg: Twist):
        # Apply velocity limits
        linear_x = max(
            min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(
            min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # Differential drive kinematics using wheel_separation parameter
        left_speed = linear_x - angular_z * (self.wheel_separation / 2.0)
        right_speed = linear_x + angular_z * (self.wheel_separation / 2.0)

        # Convert to percentage for motor controller (expects 0-100)
        # Scale based on max_linear_vel to get percentage
        max_wheel_speed = self.max_linear_vel + \
            self.max_angular_vel * (self.wheel_separation / 2.0)
        left_speed_percent = (
            left_speed / max_wheel_speed) * 100.0 * self.max_speed
        right_speed_percent = (
            right_speed / max_wheel_speed) * 100.0 * self.max_speed

        # Clamp to -100 to 100 range
        left_speed_percent = max(min(left_speed_percent, 100.0), -100.0)
        right_speed_percent = max(min(right_speed_percent, 100.0), -100.0)

        self.motor_controller.set_motor_speed(
            left_speed_percent, right_speed_percent)

        self.get_logger().debug(f'Cmd: linear={linear_x:.2f}, angular={angular_z:.2f}, '
                                f'left_speed={left_speed_percent:.1f}%, right_speed={right_speed_percent:.1f}%')


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
