#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ROSInterruptException
import math
from helper_functions import calc_velocity
from hexapod import Hexapod
import serial

class vel_sub_pub_node(Node):
    '''
    Converts a commanded velocity (Twist msg, linear/angular) from a gamepad/joystick published at topic '/cmd_vel' to
    a hexapod velocity (String msg, speed/direction) published at topic '/hexapod_vel'
    '''

    def __init__(self, hexapod):
        super().__init__("vel_sub_pub")

        self.cmd_vel_subscriber_  = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.listener_callback, 
            10)
        
        self.hexapod_vel_publisher_ = self.create_publisher(
            String,
            '/hexapod_vel',
            10
        )
        self.data = None
        self.hexapod = hexapod
        self.timer = self.create_timer(0.02,self.timer_callback) # every 20 ms

    def listener_callback(self, msg: Twist):
        self.data = msg

    def publish_angle_list(self):
        msg = String()
        msg.data = str(self.hexapod.angle_list)
        self.hexapod_vel_publisher_.publish(msg)

    def timer_callback(self):
        if self.data:
            x = self.data.linear.x
            y = self.data.linear.y
            # rx = self.data.angular.yaw
            rz = self.data.angular.z

            if abs(rz) < 0.01:
                speed, direction = calc_velocity(x,y)
                self.hexapod.move_linear(speed, direction)
            else: 
                speed = rz # abs(rz) maybe
                if rz >= 0:
                    direction = 1 # ccw
                else:
                    direction = -1 # cw
                self.hexapod.move_angular(speed, direction)


            self.publish_angle_list()
            
            # For debugging:
            self.get_logger().info(str(self.hexapod.angle_list) + 
                                   f"\n Leg2 pos: {self.hexapod.legs[1].joints[2]}" +
                                   f"\n Leg5 pos: {self.hexapod.legs[4].joints[2]}" + 
                                   f"\n t: {self.hexapod.t % 1}" +
                                   f"\n direction: {self.hexapod.current_direction}")
            # self.get_logger().info(str(self.data))

def main(args=None):
    rclpy.init(args=args)

    hexapod = Hexapod(num_legs = 6,
                  coxa_length=43,
                  femur_length=60,
                  tibia_length=104)

    node = vel_sub_pub_node(hexapod)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException,ROSInterruptException):
    # except ROSInterruptException:
        pass
    finally:
        # if node.serial_connection.is_open:
        #     node.serial_connection.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
