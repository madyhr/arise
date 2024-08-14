from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'arise'
    
    vel_sub_pub_node = Node(
        package = pkg_name,
        executable = 'vel_sub_pub.py',
        output = 'screen',
    )

    return LaunchDescription([
        vel_sub_pub_node
        ])