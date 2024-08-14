from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'arise'

    joy_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'joystick.yaml')

    joy_node = Node(
        package = 'joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable='teleop_node',
        name = 'teleop_node',
        parameters=[joy_params],
    )

    serial_sub_pub_node = Node(
        package = pkg_name,
        executable = 'serial_sub_pub.py',
        output = 'screen',
    )

    vel_sub_pub_node = Node(
        package = pkg_name,
        executable = 'vel_sub_pub.py',
        output = 'screen',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        vel_sub_pub_node,
        serial_sub_pub_node,
    ])