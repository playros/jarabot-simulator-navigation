from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    map_yaml     = LaunchConfiguration('map')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    pkg_dir = get_package_share_directory('jarabot_sim_navigation2')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'map.yaml')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': params_file,
            }.items()
        ),
    ])
