"""Synthetic closed-loop demo: no real robot SDK or control bridge is started."""
from pathlib import Path
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('stair_nav'))
    route = LaunchConfiguration('route_file')
    mission = LaunchConfiguration('mission_file')
    faults = ('not_ready', 'missing_exit', 'unstable',
              'execution_fault', 'localization_dropout')
    sim_parameters = {'route_file': route, 'initial_pose': ParameterValue(
        LaunchConfiguration('initial_pose'), value_type=List[float])}
    sim_parameters.update({name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                           for name in faults})
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=str(share/'demo'/'config'/'route.yaml')),
        DeclareLaunchArgument('mission_file', default_value=str(share/'demo'/'config'/'mission.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('initial_pose', default_value='[]',
                             description='Simulation-only [x, y, z, yaw]; [] starts at route origin O'),
        *[DeclareLaunchArgument(name, default_value='false') for name in faults],
        Node(package='stair_nav', executable='sim_executor', output='screen',
             parameters=[sim_parameters]),
        Node(package='stair_nav', executable='segment_navigation', output='screen',
             parameters=[{'route_file': route}]),
        Node(package='stair_nav', executable='task_manager', output='screen',
             parameters=[{'route_file': route, 'mission_file': mission, 'autostart': ParameterValue(
                 LaunchConfiguration('autostart'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(share/'demo'/'rviz'/'demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
