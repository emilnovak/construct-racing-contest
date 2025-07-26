from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev_mode = LaunchConfiguration('dev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dev',
            default_value='false',
            description='Launch in development mode with RViz'
        ),

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
            parameters=[{'use_sim_time': False,
                        'start_paused': dev_mode,
            }]
        ),
        Node(
            condition=IfCondition(dev_mode),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/construct-racing-contest/fastbot_racing/fastbot_racing/config/developer_conf.rviz']
        )
    ])