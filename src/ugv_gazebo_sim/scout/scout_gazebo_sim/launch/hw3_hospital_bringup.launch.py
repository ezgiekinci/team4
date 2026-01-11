from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Trial koordinatları (senin verdiğin)
TRIALS = {
    "1": {"x":  8.470379, "y": 14.438200, "z": 3.234843, "yaw": 0.0},  # 2.kat
    "2": {"x": -5.607995, "y": -11.467832, "z": 3.234845, "yaw": 0.0}, # 2.kat
    "3": {"x": -5.3511, "y": -12.348, "z": 0.234843, "yaw": 0.0}, # 1.kat
}


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    scout_pkg = get_package_share_directory('scout_gazebo_sim')

    # Hospital world
    hospital_world = os.path.join(
        os.path.expanduser('~/ros2_ws'),
        'src', 'dataset_worlds', 'worlds', 'hospital', 'hospital_two_floors.world'
    )

    # RViz config
    rviz_config = os.path.join(scout_pkg, 'config', 'hw3_hospital.rviz')

    # Launch arg: trial
    trial_arg = DeclareLaunchArgument(
        'trial',
        default_value='1',
        description='Trial ID for spawn pose (1, 2, or 3)'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': hospital_world}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    def _make_spawn(context):
        trial = LaunchConfiguration('trial').perform(context).strip()
        if trial not in TRIALS:
            # yanlış trial girilirse 1'e düş
            trial = "1"

        pose = TRIALS[trial]
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_pkg, 'launch', 'scout_v2_spawn.launch.py')
            ),
            launch_arguments={
                'start_x': str(pose["x"]),
                'start_y': str(pose["y"]),
                'start_z': str(pose["z"]),
                'start_yaw': str(pose["yaw"]),
                'robot_name': 'scout_v2'
            }.items()
        )]

    spawn_action = OpaqueFunction(function=_make_spawn)

    return LaunchDescription([
        trial_arg,
        gazebo_launch,
        spawn_action,
        rviz_node
    ])

