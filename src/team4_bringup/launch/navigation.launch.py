from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    team4_bringup_dir = get_package_share_directory('team4_bringup')

    # Varsayılan harita yolu (floor1)
    map_yaml_file = '/home/diakamos/ros2_ws/src/team4_multifloor/team4_maps/maps/floor1.yaml'

    # Team4'ün kendi Nav2 param dosyası (kalıcı fix)
    params_file = os.path.join(team4_bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'true',
                'params_file': params_file,
            }.items()
        )
    ])

