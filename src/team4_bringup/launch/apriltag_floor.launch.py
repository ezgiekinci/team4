from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    tags_yaml = os.path.join(
        get_package_share_directory('team4_bringup'),
        'config',
        'tags.yaml'
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            tags_yaml,
            # bazı sistemlerde bu parametre faydalı:
            {'image_transport': 'raw'},
        ],
        remappings=[
            # apriltag_ros çoğunlukla image_rect + camera_info bekler
            ('image_rect', '/zed2i_depth_camera/image_raw'),
            ('camera_info', '/zed2i_depth_camera/camera_info'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        apriltag_node
    ])

