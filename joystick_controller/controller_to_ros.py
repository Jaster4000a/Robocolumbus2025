import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from logitechController import Logitech

class TeleopDriver(Node):
    def __init__(self):
        super().__init__('teleop_driver')
        self.publisher_ = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('teleop node started, use joystick to drive the robot.')
        self.controller = Logitech()

    def timer_callback(self):
        inputs=self.controller.getLatestInputs()
        
        msg = Twist()
        msg.linear.x = inputs['axis'][1] if abs(inputs['axis'][1]) > 0.1 else 0.0  # forward/backward speed (m/s)
        msg.angular.z = inputs['axis'][0] if abs(inputs['axis'][0]) > 0.1 else 0.0  # turning speed (rad/s)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()