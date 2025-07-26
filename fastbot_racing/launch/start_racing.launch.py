from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='racetrack_controller',
            executable='racetrack_controller',
            name='racetrack_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])