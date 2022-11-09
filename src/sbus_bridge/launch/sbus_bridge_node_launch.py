import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbus_bridge'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package="sbus_bridge",
            executable="sbus_bridge_node",
            name="sbus_bridge_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])