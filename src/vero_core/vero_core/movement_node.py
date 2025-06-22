import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vero_core.motor_driver import MotorDriver  # Adjust import if needed

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.motor = MotorDriver(0x40)

        self.subscriber = self.create_subscription(
            String,
            'move_cmd',
            self.command_callback,
            10
        )
        self.get_logger().info("🚀 MovementNode ready and listening on /move_cmd")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"📥 Received command: {command}")

        if command == 'forward':
            self.motor.move_forward()
        elif command == 'backward':
            self.motor.move_backward()
        elif command == 'left':
            self.motor.turn_left()
        elif command == 'right':
            self.motor.turn_right()
        elif command == 'stop':
            self.motor.stop()
        else:
            self.get_logger().warn(f"⚠️ Unknown command: '{command}'")

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down MovementNode...")
    finally:
        node.motor.stop()
        node.destroy_node()
        rclpy.shutdown()
