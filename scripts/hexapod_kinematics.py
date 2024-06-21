#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException

class hexapod(Node):

    def __init__(self):
        super().__init__("hexapod_node")
        self.get_logger().info("cmd_vel listener node initialized.")
        self.cmd_vel_sub = self.create_subscription(
            Twist, "hexapod/cmd_vel", self.hexapod_callback, 10
        )
    
    def hexapod_callback(self, msg):
        self.get_logger().info(f"Received velocity: {str(msg)}")


def main(args = None):

    rclpy.init(args=args)

    node = hexapod()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()