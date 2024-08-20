from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoint_manager_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'waypoint_manager.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='waypoint_manager',
            name='waypoint_manager_node',
            output='screen',
            parameters=[waypoint_manager_config]
        ),
    ])