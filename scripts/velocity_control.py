#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException

class velocity_control(Node):

    def __init__(self):
        super().__init__("velocity_control_node")
        self.get_logger().info("cmd_vel publisher node initialized.")
        self.i = 0
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.cmd_vel_pub = self.create_publisher(
            Twist, "hexapod/cmd_vel", 10
        )
    
    def send_velocity_command(self):
        self.i += 0.1
        msg = Twist()
        msg.linear.x = 1.0 + self.i
        msg.angular.z = 1.5
        self.cmd_vel_pub.publish(msg)


def main(args = None):

    rclpy.init(args=args)

    node = velocity_control()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()