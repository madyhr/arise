from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('arise'), 'config', 'joystick.yaml')

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

    return LaunchDescription([
        joy_node,
        teleop_node
    ])