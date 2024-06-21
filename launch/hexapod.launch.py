from launch import LaunchDescription

import launch.actions
from launch_ros.actions import Node
import math

def generate_launch_description():
    pkg_name = 'arise'
    


    node_hexapod_kinematics = Node(
        package = pkg_name,
        executable = 'hexapod_kinematics.py',
        output = 'screen',
    )

    return LaunchDescription([
        node_hexapod_kinematics
        ])