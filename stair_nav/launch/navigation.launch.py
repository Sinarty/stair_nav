"""Navigation only. A separate, calibrated robot adapter must provide the interfaces."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route = LaunchConfiguration('route_file')
    # Remap the shared endpoints consistently on both nodes. Names remain the
    # existing public contract unless a deployment explicitly overrides them.
    interfaces = {
        'pose_topic': ('/localization/pose', 'Calibrated ground-projection PoseStamped'),
        'motion_status_topic': ('/motion/status', 'Adapter MotionStatus feedback'),
        'command_topic': ('/navigation/command', 'Adapter ControlCommand input'),
        'channel_event_topic': ('/navigation/channel_event', 'External channel blocking events'),
        'obstacle_status_topic': ('/navigation/obstacle_status', 'Leased flat-ground obstacle perception'),
        'navigate_segment_action': ('/navigate_segment', 'Internal NavigateSegment action'),
        'navigation_state_service': ('/navigation/get_state', 'Navigation release snapshot service'),
        'execute_mission_action': ('/execute_mission', 'Public ExecuteMission action'),
        'navigation_status_topic': ('/navigation/status', 'Segment status snapshots'),
        'reference_path_topic': ('/navigation/reference_path', 'Planned path visualization'),
        'reference_point_topic': ('/navigation/reference_point', 'Tracking reference visualization'),
        'scene_topic': ('/navigation/scene', 'Route scene visualization'),
        'mission_status_topic': ('/mission/status', 'Mission status snapshots'),
        'mission_events_topic': ('/mission/events', 'Mission transition events'),
        'task_status_topic': ('/task/status', 'Compatibility segment status snapshots'),
        'start_service': ('/task_manager/start', 'Explicit task start service'),
        'cancel_service': ('/task_manager/cancel', 'Task cancellation service'),
        'reset_service': ('/task_manager/reset', 'Guarded task reset service'),
    }
    remappings = [(original, LaunchConfiguration(name))
                  for name, (original, _) in interfaces.items()]
    qos = LaunchConfiguration('pose_qos_reliability')
    return LaunchDescription([
        # Intentionally no real-map default and no automatic task start.
        DeclareLaunchArgument('route_file', description='Validated route YAML for this robot/site'),
        DeclareLaunchArgument('mission_file', description='Mission YAML referencing this navigation route'),
        DeclareLaunchArgument('pose_qos_reliability', default_value='reliable',
                              choices=['reliable', 'best_effort'],
                              description='Localization subscription reliability on both nodes'),
        *[DeclareLaunchArgument(name, default_value=original, description=description)
          for name, (original, description) in interfaces.items()],
        Node(package='stair_nav', executable='segment_navigation', output='screen',
             parameters=[{'route_file': route, 'pose_qos_reliability': qos}],
             remappings=remappings),
        Node(package='stair_nav', executable='task_manager', output='screen',
             parameters=[{'route_file': route,
                          'mission_file': LaunchConfiguration('mission_file'), 'autostart': False,
                          'pose_qos_reliability': qos}], remappings=remappings),
    ])
