import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')

    use_probability = LaunchConfiguration('probability', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_mapper = LaunchConfiguration('mapper', default=False)
    synchronization = LaunchConfiguration('synchronization', default=False)
    fill_map = LaunchConfiguration('fill_map', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default='epuck_world.wbt')
    mapping_time = LaunchConfiguration('mapping_time', default=5)
    map_path = LaunchConfiguration('path', default=str(Path.home()))

    # Webots
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'robot_launch.py')
        ),
        launch_arguments={
            'synchronization': synchronization,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Base configuration
    tools_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'robot_tools_launch.py')
        ),
        launch_arguments={
            'nav': use_nav,
            'rviz': use_rviz,
            'mapper': use_mapper,
            'use_sim_time': use_sim_time,
            'world': world
        }.items()
    )

    return LaunchDescription([
        webots_launch,
        tools_launch,
        Node(
            package='mapping_controller',
            executable='random_bounce',
            output='log'
        ),
        Node(
            package='mapping_controller',
            executable='mission_controller',
            output='screen',
            parameters=[{'use_probability': use_probability,
                         'mapping_time': mapping_time,
                         'mapper': use_mapper,
                         'path': map_path}]
        ),
        Node(
            package='mapping_controller',
            executable='probability_mapper',
            output='log',
            parameters=[{'use_sim_time': use_sim_time, 'fill_map': fill_map}],
            condition=launch.conditions.IfCondition(use_probability)
        ),
    ])
