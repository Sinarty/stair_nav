"""Navigation only. A separate, calibrated robot adapter must provide the interfaces."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route = LaunchConfiguration('route_file')
    return LaunchDescription([
        # Intentionally no real-map default and no automatic task start.
        DeclareLaunchArgument('route_file', description='Validated route YAML for this robot/site'),
        DeclareLaunchArgument('mission_file', description='Mission YAML referencing this navigation route'),
        Node(package='stair_nav', executable='segment_navigation', output='screen',
             parameters=[{'route_file': route}]),
        Node(package='stair_nav', executable='task_manager', output='screen',
             parameters=[{'route_file': route,
                          'mission_file': LaunchConfiguration('mission_file'), 'autostart': False}]),
    ])
