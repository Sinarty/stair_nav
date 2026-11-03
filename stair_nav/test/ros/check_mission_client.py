#!/usr/bin/env python3
"""Chinese CLI checks using real ROS Actions and an isolated, local stub server.

Source ROS and the built stair_nav_interfaces package before running this file.
No robot nodes, vendor adapters, or physical control topics are started.
"""
from contextlib import redirect_stdout
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from stair_nav_interfaces.action import ExecuteMission
from stair_nav_interfaces.msg import MissionStatus


SOURCE_PACKAGE = Path(__file__).resolve().parents[2]


class MissionServer:
    """A goal-specific server; cancellation is observed before returning a result."""

    def __init__(self, mode):
        self.mode = mode
        suffix = uuid.uuid4().hex
        self.action_name = '/mission_client_check_' + suffix
        self.node = Node('mission_client_server_' + suffix)
        self.goal_requested = threading.Event()
        self.executing = threading.Event()
        self.stop_requested = threading.Event()
        self.result_returned = threading.Event()
        self.goal_count = 0
        self.cancel_count = 0
        self.route_ids = []
        self.server = ActionServer(
            self.node, ExecuteMission, self.action_name, self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_goal(self, request):
        self.goal_count += 1
        self.route_ids.append(request.route_id)
        self.goal_requested.set()
        if self.mode == 'reject':
            return GoalResponse.REJECT
        if self.mode == 'cancel_before_accept':
            self.stop_requested.wait(.8)
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    def feedback(self, handle, state, segment_state='', progress=0.):
        value = MissionStatus()
        value.header.frame_id = 'map'
        value.header.stamp = self.node.get_clock().now().to_msg()
        value.task_id = 'client-check-task'
        value.route_id = 'inspection_demo'
        value.execution_id = 'client-check-execution'
        value.segment_id = 'AB'
        value.segment_index = value.segment_count = 1
        value.state, value.segment_state = state, segment_state
        value.segment_progress = progress
        handle.publish_feedback(ExecuteMission.Feedback(status=value))

    def execute(self, handle):
        self.executing.set()
        result = ExecuteMission.Result()
        result.task_id = 'client-check-task'
        result.route_id = 'inspection_demo'
        if self.mode.startswith('cancel'):
            self.feedback(handle, 'RUNNING', 'EXECUTING', .25)
            deadline = time.monotonic() + 10.
            while not handle.is_cancel_requested and not self.stop_requested.is_set():
                if time.monotonic() >= deadline:
                    break
                self.stop_requested.wait(.02)
            if handle.is_cancel_requested:
                self.feedback(handle, 'CANCELING', 'CANCELED', .25)
                if self.mode == 'cancel_no_result':
                    # A cancel acknowledgement alone does not supply a result.
                    while not self.stop_requested.wait(.02):
                        pass
                    result.state = 'FAILED'
                    result.error_code = 'TEST_STOP'
                    handle.abort()
                    self.result_returned.set()
                    return result
                # The client must wait for this result after cancel acknowledgement.
                self.stop_requested.wait(.2)
                result.navigation_stopped = True
                if self.mode == 'cancel_unconfirmed':
                    result.state = 'FAULT_BLOCKED'
                    result.error_code = 'CLEANUP_TIMEOUT'
                    result.error_message = 'physical hold is still unconfirmed'
                    result.hold_confirmed = False
                    handle.abort()
                else:
                    result.state = 'CANCELED'
                    result.error_code = 'CANCELED'
                    result.hold_confirmed = True
                    handle.canceled()
            else:
                result.state = 'FAILED'
                result.error_code = 'TEST_DEADLINE'
                handle.abort()
        elif self.mode == 'failure':
            self.feedback(handle, 'RUNNING', 'EXECUTING', .25)
            self.stop_requested.wait(.15)
            result.state = 'FAULT_BLOCKED'
            result.error_code = 'VENDOR_CUSTOM'
            result.error_message = 'original firmware reason'
            result.navigation_stopped = True
            result.hold_confirmed = False
            handle.abort()
        else:
            for state, segment, progress in (
                    ('VALIDATING', '', 0.),
                    ('RUNNING', 'PLANNING', 0.),
                    ('RUNNING', 'EXECUTING', .25),
                    ('RUNNING', 'EXECUTING', .75),
                    ('RUNNING', 'ARRIVAL_CONFIRM', 1.)):
                self.feedback(handle, state, segment, progress)
                self.stop_requested.wait(.12)
            result.success = True
            result.state = 'SUCCEEDED'
            result.navigation_stopped = True
            result.hold_confirmed = self.mode != 'success_unconfirmed'
            handle.succeed()
        self.result_returned.set()
        return result

    def close(self):
        self.stop_requested.set()
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        self.server.destroy()
        self.node.destroy_node()


class MissionClientChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Keep both server and client away from an inherited robot Domain.
        os.environ['ROS_DOMAIN_ID'] = str(160 + os.getpid() % 40)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.logs = tempfile.TemporaryDirectory(prefix='stair_nav_client_checks_', dir='/tmp')
        os.environ['ROS_LOG_DIR'] = cls.logs.name
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        cls.logs.cleanup()

    def client(self, action_name, *options):
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        env['PYTHONDONTWRITEBYTECODE'] = '1'
        env['PYTHONPATH'] = str(SOURCE_PACKAGE) + os.pathsep + env.get('PYTHONPATH', '')
        process = subprocess.Popen(
            [sys.executable, '-m', 'stair_nav.mission_client',
             '--route-id', 'inspection_demo', '--action', action_name,
             '--server-timeout', '4', '--cancel-timeout', '4', *options],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        self.addCleanup(self.stop_client, process)
        return process

    @staticmethod
    def stop_client(process):
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=3.)

    def completed(self, process, timeout=12.):
        try:
            output, _ = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=3.)
            self.fail('Chinese mission client did not finish:\n' + output)
        return process.returncode, output

    def server(self, mode):
        server = MissionServer(mode)
        self.addCleanup(server.close)
        return server

    def test_success_uses_feedback_and_returns_zero(self):
        server = self.server('success')
        code, output = self.completed(self.client(server.action_name))
        self.assertEqual(code, 0, output)
        self.assertEqual(server.goal_count, 1, output)
        self.assertEqual(server.route_ids, ['inspection_demo'])
        self.assertIn('inspection_demo', output)
        self.assertIn('AB', output)
        self.assertRegex(output, '规划|跟踪')
        self.assertIn('成功', output)
        self.assertNotIn('nanosec:', output)

    def test_private_context_preserves_existing_ros_context_and_sigint_handler(self):
        from stair_nav.mission_client import main

        server = self.server('success')
        previous_handler = signal.getsignal(signal.SIGINT)
        output = io.StringIO()
        with redirect_stdout(output):
            code = main(['--route-id', 'inspection_demo',
                         '--action', server.action_name, '--server-timeout', '4'])
        self.assertEqual(code, 0, output.getvalue())
        self.assertEqual(server.goal_count, 1)
        self.assertTrue(rclpy.ok(), 'client shutdown affected the caller ROS context')
        self.assertEqual(signal.getsignal(signal.SIGINT), previous_handler)

    def test_private_context_cleanup_restores_sigint_after_node_creation_error(self):
        from unittest.mock import patch
        from stair_nav.mission_client import main

        previous_handler = signal.getsignal(signal.SIGINT)
        output = io.StringIO()
        with patch('stair_nav.mission_client.MissionClient',
                   side_effect=RuntimeError('client construction failed')):
            with redirect_stdout(output):
                code = main(['--server-timeout', '.1'])
        self.assertEqual(code, 1, output.getvalue())
        self.assertIn('client construction failed', output.getvalue())
        self.assertTrue(rclpy.ok(), 'failed client affected the caller ROS context')
        self.assertEqual(signal.getsignal(signal.SIGINT), previous_handler)

    def test_success_without_hold_confirmation_is_not_reported_as_success(self):
        server = self.server('success_unconfirmed')
        code, output = self.completed(self.client(server.action_name))
        self.assertEqual(code, 1, output)
        self.assertEqual(server.goal_count, 1)
        self.assertTrue(server.result_returned.is_set())
        self.assertIn('未确认', output)
        self.assertNotIn('巡检成功', output)

    def test_action_endpoint_accepts_standard_ros_remapping(self):
        server = self.server('success')
        code, output = self.completed(self.client(
            server.action_name, '--action', '/execute_mission', '--ros-args',
            '-r', '/execute_mission:=' + server.action_name))
        self.assertEqual(code, 0, output)
        self.assertEqual(server.goal_count, 1, output)
        self.assertEqual(server.route_ids, ['inspection_demo'])

    def test_rejected_goal_is_not_retried(self):
        server = self.server('reject')
        code, output = self.completed(self.client(server.action_name))
        self.assertNotEqual(code, 0, output)
        self.assertEqual(server.goal_count, 1, output)
        self.assertFalse(server.executing.is_set())
        self.assertIn('拒绝', output)

    def test_unknown_failure_preserves_original_diagnostics(self):
        server = self.server('failure')
        code, output = self.completed(self.client(server.action_name))
        self.assertEqual(code, 1, output)
        self.assertEqual(server.goal_count, 1)
        self.assertIn('VENDOR_CUSTOM', output)
        self.assertIn('original firmware reason', output)
        self.assertRegex(output, '阻塞|故障')
        self.assertIn('未确认', output)
        self.assertNotIn('已停稳', output)

    def check_interrupt(self, mode, before_accept=False, expected_code=130):
        server = self.server(mode)
        process = self.client(server.action_name)
        boundary = server.goal_requested if before_accept else server.executing
        self.assertTrue(boundary.wait(timeout=8.), 'client never sent an Action goal')
        # In the delayed acceptance case on_goal is still waiting at this point.
        process.send_signal(signal.SIGINT)
        code, output = self.completed(process)
        self.assertEqual(code, expected_code, output)
        self.assertEqual(server.goal_count, 1, output)
        self.assertEqual(server.cancel_count, 1, output)
        self.assertTrue(server.result_returned.is_set(), output)
        self.assertIn('取消', output)
        return output

    def test_interrupt_cancels_own_goal_and_waits_for_result(self):
        self.check_interrupt('cancel')

    def test_interrupt_before_acceptance_cancels_late_handle(self):
        self.check_interrupt('cancel_before_accept', before_accept=True)

    def test_cancel_acknowledgement_is_not_reported_as_confirmed_hold(self):
        output = self.check_interrupt('cancel_unconfirmed', expected_code=1)
        self.assertIn('CLEANUP_TIMEOUT', output)
        self.assertIn('未确认', output)
        self.assertNotIn('已停稳', output)

    def test_cancel_without_result_times_out_as_uncertain(self):
        server = self.server('cancel_no_result')
        process = self.client(server.action_name, '--cancel-timeout', '.5')
        self.assertTrue(server.executing.wait(timeout=8.))
        process.send_signal(signal.SIGINT)
        code, output = self.completed(process, timeout=5.)
        self.assertEqual(code, 1, output)
        self.assertEqual(server.goal_count, 1)
        self.assertEqual(server.cancel_count, 1)
        self.assertFalse(server.result_returned.is_set())
        self.assertIn('结果未确认', output)
        self.assertIn('超时', output)
        self.assertIn('不代表机器人已停止', output)
        self.assertNotIn('[任务取消]', output)
        self.assertNotIn('运控保持：已确认', output)

    def test_missing_server_times_out_without_a_goal(self):
        action_name = '/missing_client_check_' + uuid.uuid4().hex
        code, output = self.completed(self.client(
            action_name, '--server-timeout', '.3'), timeout=6.)
        self.assertNotEqual(code, 0, output)
        self.assertRegex(output, '超时|未发现|不可用|未找到')


if __name__ == '__main__':
    unittest.main()
