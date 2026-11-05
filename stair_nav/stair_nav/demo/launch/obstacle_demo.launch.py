"""Flat-only live point-cloud stop/resume demonstration; no hardware adapters."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('stair_nav'))
    route = str(share/'demo'/'config'/'obstacle_route.yaml')
    mission = str(share/'demo'/'config'/'obstacle_mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('auto_obstacle', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('obstacle_duration', default_value='5.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/'demo.launch.py')),
            launch_arguments={'route_file': route, 'mission_file': mission,
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': str(share/'demo'/'rviz'/'obstacle_demo.rviz'),
                              'autostart': LaunchConfiguration('autostart')}.items()),
        Node(package='stair_nav', executable='obstacle_monitor', output='screen',
             parameters=[str(share/'demo'/'config'/'obstacle_monitor.yaml'), {'route_file': route}]),
        Node(package='stair_nav', executable='demo_obstacles', output='screen',
             parameters=[{'route_file': route,
                          'auto_obstacle': ParameterValue(LaunchConfiguration('auto_obstacle'), value_type=bool),
                          'obstacle_duration': ParameterValue(LaunchConfiguration('obstacle_duration'), value_type=float)}]),
    ])
