from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoint_saver_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'waypoint_saver.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='waypoint_saver',
            name='waypoint_saver_node',
            output='screen',
            remappings=[('current_pose', 'current_pose')],
            parameters=[waypoint_saver_config]
        ),

         Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
        ),    
    ])