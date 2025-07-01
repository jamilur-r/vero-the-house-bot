import rclpy
from geometry_msgs.msg import Twist
from motor_controller import MotorController
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
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

    def cmd_vel_callback(self, msg: Twist):
        left_speed = msg.linear.x - msg.angular.z * 0.5
        right_speed = msg.linear.x + msg.angular.z * 0.5

        self.motor_controller.set_motor_speed(left_speed, right_speed)


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
