#!/usr/bin/env python3
"""Deployment interface checks; source ROS and built stair_nav_interfaces first.

Uses UUID-prefixed ROS endpoints, never starts a mission or a robot adapter.
Kept outside test_*.py collection so the core suite remains ROS-independent.
"""
import importlib.util
from pathlib import Path
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from launch.actions import DeclareLaunchArgument
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger

from stair_nav.ros_utils import pose_qos
from stair_nav.segment_node import SegmentNavigation
from stair_nav.task_node import TaskManager
from stair_nav_interfaces.action import ExecuteMission, NavigateSegment
from stair_nav_interfaces.srv import GetNavigationState


ROOT = Path(__file__).resolve().parents[2]
INTERFACES = {
    'pose_topic': '/localization/pose',
    'motion_status_topic': '/motion/status',
    'command_topic': '/navigation/command',
    'channel_event_topic': '/navigation/channel_event',
    'navigate_segment_action': '/navigate_segment',
    'navigation_state_service': '/navigation/get_state',
    'execute_mission_action': '/execute_mission',
    'navigation_status_topic': '/navigation/status',
    'reference_path_topic': '/navigation/reference_path',
    'reference_point_topic': '/navigation/reference_point',
    'scene_topic': '/navigation/scene',
    'mission_status_topic': '/mission/status',
    'mission_events_topic': '/mission/events',
    'task_status_topic': '/task/status',
    'start_service': '/task_manager/start',
    'cancel_service': '/task_manager/cancel',
    'reset_service': '/task_manager/reset',
}


class DeploymentInterfaces(unittest.TestCase):
    def test_localization_qos_validation(self):
        self.assertEqual(pose_qos().reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(pose_qos('best_effort').reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(pose_qos('best_effort').durability, DurabilityPolicy.VOLATILE)
        self.assertEqual(pose_qos().depth, 5)
        for invalid in ('', 'sensor_data', 'BEST_EFFORT', None, 1, []):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(
                    ValueError, 'pose_qos_reliability'):
                pose_qos(invalid)

    def test_launch_preserves_interface_defaults(self):
        spec = importlib.util.spec_from_file_location(
            'deployment_launch', ROOT/'launch'/'navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        arguments = {item.name: item for item in module.generate_launch_description().entities
                     if isinstance(item, DeclareLaunchArgument)}
        for name, original in INTERFACES.items():
            with self.subTest(name=name):
                self.assertEqual(''.join(value.text for value in arguments[name].default_value),
                                 original)
        qos = arguments['pose_qos_reliability']
        self.assertEqual(''.join(value.text for value in qos.default_value), 'reliable')
        self.assertEqual(qos.choices, ['reliable', 'best_effort'])

    def test_reliable_pose_and_remapped_interfaces(self):
        self.check_ros_interfaces('reliable')

    def test_best_effort_pose_and_remapped_interfaces(self):
        self.check_ros_interfaces('best_effort')

    def check_ros_interfaces(self, reliability):
        prefix = '/deployment_test_' + uuid.uuid4().hex
        targets = {name: prefix + original for name, original in INTERFACES.items()}
        arguments = ['--ros-args',
                     '-p', f'route_file:={ROOT / "stair_nav/demo/config/route.yaml"}',
                     '-p', f'mission_file:={ROOT / "stair_nav/demo/config/mission.yaml"}',
                     '-p', f'pose_qos_reliability:={reliability}']
        for name, original in INTERFACES.items():
            arguments += ['-r', f'{original}:={targets[name]}']
        nodes, actions = [], []
        rclpy.init(args=arguments)
        executor = SingleThreadedExecutor()
        try:
            segment = SegmentNavigation()
            nodes.append(segment)
            task = TaskManager()
            nodes.append(task)
            probe = Node('deployment_probe_' + uuid.uuid4().hex, use_global_arguments=False)
            nodes.append(probe)
            for node in nodes:
                executor.add_node(node)

            def wait(predicate, timeout=5.):
                deadline = time.monotonic() + timeout
                while not predicate():
                    if time.monotonic() > deadline:
                        self.fail(f'ROS discovery/feedback timeout ({reliability}); '
                                  f'actions={get_action_names_and_types(probe)}; '
                                  f'services={probe.get_service_names_and_types()}')
                    executor.spin_once(timeout_sec=.02)

            segment_action = ActionClient(probe, NavigateSegment, targets['navigate_segment_action'])
            mission_action = ActionClient(probe, ExecuteMission, targets['execute_mission_action'])
            actions += [segment_action, mission_action]
            wait(lambda: segment_action.server_is_ready() and mission_action.server_is_ready()
                 and task.client.server_is_ready() and task.state_client.service_is_ready())
            triggers = [probe.create_client(Trigger, targets[name])
                        for name in ('start_service', 'cancel_service', 'reset_service')]
            wait(lambda: all(client.service_is_ready() for client in triggers))

            publisher = probe.create_publisher(PoseStamped, targets['pose_topic'], pose_qos(reliability))
            wait(lambda: publisher.get_subscription_count() == 2)
            message = PoseStamped()
            message.header.frame_id = 'map'
            message.pose.orientation.w = 1.
            message.pose.position.x = .123
            message.header.stamp = probe.get_clock().now().to_msg()
            # Retry publication during discovery without renewing observation time.
            deadline = time.monotonic() + 3.
            while segment.core.pose is None or task.pose is None:
                self.assertLess(time.monotonic(), deadline, 'localization was not delivered')
                publisher.publish(message)
                executor.spin_once(timeout_sec=.02)
            self.assertAlmostEqual(segment.core.pose.x, .123)
            self.assertAlmostEqual(task.pose.pose.position.x, .123)
            infos = probe.get_subscriptions_info_by_topic(targets['pose_topic'])
            self.assertEqual(len(infos), 2)
            self.assertTrue(all(info.qos_profile.reliability == pose_qos(reliability).reliability
                                for info in infos))

            for name in ('command_topic', 'navigation_status_topic', 'reference_path_topic',
                         'reference_point_topic', 'scene_topic', 'mission_status_topic',
                         'mission_events_topic', 'task_status_topic'):
                with self.subTest(publisher=name):
                    wait(lambda: bool(probe.get_publishers_info_by_topic(targets[name])))
            for name in ('motion_status_topic', 'channel_event_topic'):
                with self.subTest(subscription=name):
                    wait(lambda: bool(probe.get_subscriptions_info_by_topic(targets[name])))
            command_qos = probe.get_publishers_info_by_topic(targets['command_topic'])[0].qos_profile
            self.assertEqual(command_qos.reliability, ReliabilityPolicy.RELIABLE)
            self.assertEqual(command_qos.durability, DurabilityPolicy.VOLATILE)
            for name in ('navigation_status_topic', 'mission_status_topic', 'mission_events_topic'):
                status_qos = probe.get_publishers_info_by_topic(targets[name])[0].qos_profile
                self.assertEqual(status_qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)

            # A remapped synchronization request remains read-only and reports
            # no release without actual adapter feedback.
            future = task.state_client.call_async(GetNavigationState.Request())
            wait(future.done)
            self.assertFalse(future.result().busy)
            self.assertFalse(future.result().control_released)
            self.assertEqual(task.core.status.state, 'IDLE')
        finally:
            for action in actions:
                action.destroy()
            executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
