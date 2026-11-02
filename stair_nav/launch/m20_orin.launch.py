"""External Orin: production state machines plus firmware-matched M20 bridges."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from stair_nav.core.config import load_route


def _start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        result = value(name).lower()
        if result not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return result == 'true'

    route, mission = value('route_file'), value('mission_file')
    frame = load_route(route)['frame']
    normal = int(value('normal_location_status'))
    if normal not in (-1, 0, 1):
        raise ValueError('normal_location_status must be -1 (blocked), 0, or 1')
    share = Path(get_package_share_directory('stair_nav'))
    topics = {key: value(key) for key in ('pose_topic', 'motion_status_topic', 'command_topic')}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'navigation.launch.py')),
            launch_arguments=dict(route_file=route, mission_file=mission, **topics).items()),
        Node(package='stair_nav', executable='m20_localization', output='screen',
             parameters=[value('adapter_config'), {
                 'route_frame': frame, 'normal_location_status': normal,
                 'output_pose_topic': topics['pose_topic'],
                 'navigation_reference_frame': value('navigation_reference_frame')}]),
        Node(package='stair_nav', executable='m20_motion_adapter', output='screen',
             parameters=[value('adapter_config'), {
                 'route_file': route, 'normal_location_status': normal,
                 'dry_run': boolean('dry_run'),
                 'navigation_mode_confirmed': boolean('navigation_mode_confirmed'),
                 'planner_stopped': boolean('planner_stopped'),
                 'command_topic': topics['command_topic'],
                 'motion_status_topic': topics['motion_status_topic']}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('stair_nav'))
    return LaunchDescription([
        DeclareLaunchArgument('route_file', description='Validated site route YAML; no demo default'),
        DeclareLaunchArgument('mission_file', description='Site mission YAML'),
        DeclareLaunchArgument('adapter_config', default_value=str(share / 'config' / 'm20_orin.yaml')),
        DeclareLaunchArgument('normal_location_status', default_value='-1',
                              description='Confirm firmware normal code 0 or 1; -1 blocks navigation'),
        DeclareLaunchArgument('navigation_reference_frame', default_value='navigation_ground_reference'),
        DeclareLaunchArgument('dry_run', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('navigation_mode_confirmed', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('planner_stopped', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('motion_status_topic', default_value='/motion/status'),
        DeclareLaunchArgument('command_topic', default_value='/navigation/command'),
        OpaqueFunction(function=_start),
    ])
