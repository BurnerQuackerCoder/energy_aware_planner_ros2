from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = os.path.expanduser('~/ros2_ws/src/energy_aware_planner')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # 1. Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'autostart': 'True',
                'map': os.path.join(pkg_dir, 'maps', 'complex_maze.yaml'),
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
            }.items()
        ),

        # 2. Stable Static Transforms (Pre-positioned for the Demo)
        # We set the 'map' to 'odom' translation to -4.0 to move the robot to the left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['-4.0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # 3. Battery Simulator
        Node(package='energy_aware_planner', executable='battery_sim.py', name='battery_sim'),

    ])