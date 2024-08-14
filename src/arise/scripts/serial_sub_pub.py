#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ROSInterruptException
import serial

from helper_functions import find_port

class serial_sub_pub_node(Node):

    def __init__(self, active_port):
        super().__init__("serial_sub_pub")

        self.hexapod_vel_subscriber_  = self.create_subscription(
            String, 
            '/hexapod_vel', 
            self.listener_callback, 
            10)
        
        self.data = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
        self.timer = self.create_timer(0.02,self.timer_callback) # every 20 ms
        self.port_string = active_port
        try:
            self.serial_connection = serial.Serial(port=f"/dev/tty{self.port_string}", 
                                                   parity=serial.PARITY_EVEN, 
                                                   stopbits=serial.STOPBITS_ONE,
                                                #    baudrate=115200,
                                                   timeout=0.005
                                                   )
            self.serial_connection.flush()
            self.get_logger().info(f'Serial connection established at /dev/tty{self.port_string}. Huzzah!')
            # self.serial_connection.reset_input_buffer()
        
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port /dev/tty{self.port_string}: {e}")

        self.counter = 0

    def listener_callback(self, msg: String):
        self.data = msg.data

    def timer_callback(self):
        # if self.data:
        
        angle_msg = str(self.data) + "\r"
        # self.get_logger().info(angle_msg) # for debugging
        self.serial_connection.write(angle_msg.encode())
        # self.serial_connection.flush()
        self.counter += 1
        # self.get_logger().info("Counter: " + str(self.counter))

        if self.counter == 1:
            # serial connection seems to become unstable after about 80 writes, so reconnecting every 50 writes seems to work (here be dragons)
            #TODO: figure out why this works this way and make it better
            self.serial_connection = serial.Serial(port=f"/dev/tty{self.port_string}", 
                                                   parity=serial.PARITY_EVEN, 
                                                   stopbits=serial.STOPBITS_ONE,
                                                #    baudrate=115200,
                                                   timeout=0.005
                                                   )
            self.serial_connection.flush()

            self.counter = 0
        

def main(args=None):
    rclpy.init(args=args)

    active_port = find_port()
    node = serial_sub_pub_node(active_port)

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
