from launch import LaunchDescription

import launch.actions
from launch_ros.actions import Node
import math

def generate_launch_description():
    pkg_name = 'arise'
    
    node_velocity_control = Node(
        package = pkg_name,
        executable = 'velocity_control.py',
        output = 'screen',
    )

    return LaunchDescription([
        node_velocity_control
        ])