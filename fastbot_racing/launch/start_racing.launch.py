from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    PKG_DIR = FindPackageShare('fastbot_racing')
    CONFIG_DIR = PathJoinSubstitution([PKG_DIR, 'config'])

    use_sim_time = True
    map_file = '/home/user/maps/map.yaml'

    nav2_config = PathJoinSubstitution([CONFIG_DIR, 'navigation.yaml'])
    nav2_launch_dir = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch'])
    launch_dir = PathJoinSubstitution([PKG_DIR, 'launch'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'navigation_bringup.launch.py'])),
            launch_arguments={
                'map': map_file,
                'use_sim_time': str(use_sim_time),
                'use_namespace': 'true',
                'namespace': 'fastbot_1',
                'params_file': nav2_config
            }.items(),
        ),
    ])