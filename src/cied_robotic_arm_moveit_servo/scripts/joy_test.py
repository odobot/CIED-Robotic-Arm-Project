import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.servo_pub = self.create_publisher(TwistStamped, '/servo_commands', 10)
        self.mode = 'joint_control'  # Default mode

    def joy_callback(self, msg):
        # Example mapping logic:
        if msg.buttons[0]:  # A button
            self.mode = 'joint_control'
            self.get_logger().info('Switched to Joint Control Mode')
        elif msg.buttons[1]:  # B button
            self.mode = 'cartesian_control'
            self.get_logger().info('Switched to Cartesian Control Mode')

        if self.mode == 'cartesian_control':
            twist = TwistStamped()
            twist.twist.linear.x = msg.axes[0]  # LeftX
            twist.twist.linear.y = msg.axes[1]  # LeftY
            twist.twist.angular.z = msg.axes[2]  # RightX
            self.servo_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
