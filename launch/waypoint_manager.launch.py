from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the paths to the waypoint CSV and the configuration file
    waypoints_csv_path = '/home/shugo/3dnav_ws/src/waypoint_manager/waypoints/241002.csv'
    config_file_path = os.path.join(
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
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),
        Node(
            package='waypoint_manager',
            executable='waypoint_visualizer',
            name='waypoint_visualizer_node',
            output='screen',
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),
        Node(
            package='waypoint_manager',
            executable='waypoint_skipper',
            name='waypoint_skipper_node',
            output='screen',
            remappings=[('/current_pose', '/current_pose'),
                        ('/scan', '/scan')
            ],
            parameters=[config_file_path, {
                'waypoints_csv' : waypoints_csv_path,
            }]
        ),
    ])
