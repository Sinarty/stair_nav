#!/usr/bin/env python3
"""Deterministic Action client callback race tests (requires built ROS interfaces).

No ROS graph or hardware is used. Real TaskManager callback methods receive
controlled futures/messages, including deliberately late generations.
"""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from threading import RLock

from stair_nav.task_node import TaskManager
from stair_nav.sim_node import SimExecutor
from stair_nav.segment_node import SegmentNavigation
from stair_nav.ros_utils import stamp
from stair_nav.core.models import Status, Command, ExecutionOutput
from rclpy.action import CancelResponse
from stair_nav_interfaces.action import NavigateSegment
from stair_nav_interfaces.msg import SegmentStatus, ControlCommand, MotionStatus


class Handle:
    accepted = True
    def __init__(self):
        self.cancel_calls = 0
        self.result_future = Future()
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return Future()
    def get_result_async(self):
        return self.result_future


class Harness:
    feedback = TaskManager.feedback
    accepted = TaskManager.accepted
    result = TaskManager.result
    finish = TaskManager.finish
    timer = TaskManager.timer
    def __init__(self):
        self.generation = 2
        self.active = True
        self.index = 0
        self.sequence = ['AB', 'BC', 'CD']
        self.status = SegmentStatus(task_id='task-current', execution_id='current',
                                    segment_id='AB', state='EXECUTING', progress=0.4)
        self.cancel_requested = False
        self.transport_error = ''
        self.last_contact = 10.0
        self.transport_timeout = 5.0
        self.autostart_pending = False
        self.goal_handle = None
        self.published = 0
        self.dispatched = []
    def now(self):
        return 10.0
    def publish(self):
        self.published += 1
    def dispatch(self, generation):
        self.dispatched.append(generation)


def done(value):
    future = Future()
    future.set_result(value)
    return future


def feedback(state='EXECUTING', execution_id='current', task_id='task-current'):
    return SimpleNamespace(feedback=SimpleNamespace(status=SegmentStatus(
        task_id=task_id, execution_id=execution_id, segment_id='AB', state=state,
        progress=1.0 if state == 'SUCCEEDED' else 0.6)))


def success():
    return done(SimpleNamespace(result=NavigateSegment.Result(
        success=True, navigation_stopped=True, hold_confirmed=True)))


class CallbackTests(unittest.TestCase):
    def test_segment_success_feedback_is_not_task_success(self):
        manager = Harness()
        manager.feedback(feedback('SUCCEEDED'), 2, 'current')
        self.assertEqual(manager.status.state, 'SEGMENT_SUCCEEDED')
        self.assertTrue(manager.active)
        self.assertEqual(manager.index, 0)

    def test_old_feedback_cannot_replace_current_status(self):
        for generation, execution_id in ((1, 'current'), (2, 'old')):
            manager = Harness()
            original = manager.status
            manager.feedback(feedback('SUCCEEDED', execution_id), generation, execution_id)
            self.assertIs(manager.status, original)
            self.assertEqual(manager.published, 0)

    def test_old_result_cannot_finish_or_advance_current_task(self):
        manager = Harness()
        manager.result(success(), 1, 'old')
        self.assertTrue(manager.active)
        self.assertEqual(manager.status.execution_id, 'current')
        self.assertEqual(manager.index, 0)
        self.assertEqual(manager.dispatched, [])

    def test_late_old_goal_acceptance_is_canceled(self):
        manager = Harness()
        handle = Handle()
        manager.accepted(done(handle), 1, 'old')
        self.assertEqual(handle.cancel_calls, 1)
        self.assertIsNone(manager.goal_handle)

    def test_cancel_before_goal_acceptance_cancels_when_accepted(self):
        manager = Harness()
        manager.cancel_requested = True
        handle = Handle()
        manager.accepted(done(handle), 2, 'current')
        self.assertEqual(handle.cancel_calls, 1)
        self.assertIs(manager.goal_handle, handle)
        self.assertTrue(manager.active)

    def test_only_final_segment_result_completes_task(self):
        manager = Harness()
        manager.result(success(), 2, 'current')
        self.assertEqual(manager.index, 1)
        self.assertEqual(manager.dispatched, [2])
        self.assertTrue(manager.active)
        manager.index = 2
        manager.result(success(), 2, 'current')
        self.assertEqual(manager.status.state, 'SUCCEEDED')
        self.assertFalse(manager.active)

    def test_transport_timeout_reports_unknown_stop_and_keeps_busy(self):
        manager = Harness()
        manager.last_contact = 0.0
        handle = Handle()
        manager.goal_handle = handle
        manager.timer()
        self.assertEqual(manager.status.state, 'TRANSPORT_TIMEOUT')
        self.assertEqual(manager.status.error_code, 'ACTION_TRANSPORT_TIMEOUT')
        self.assertFalse(manager.status.navigation_stopped)
        self.assertFalse(manager.status.hold_confirmed)
        self.assertTrue(manager.active)
        self.assertEqual(handle.cancel_calls, 1)


class EarlyCancelTests(unittest.TestCase):
    def test_cancel_between_accept_and_execute_never_emits_valid_command(self):
        calls = []
        class Core:
            active = False
            def start(self, task_id, execution_id, segment_id, now):
                calls.append('start')
                self.active = True
                return True
            def cancel(self, now):
                calls.append('cancel')
                self.active = False
            def tick(self, now, dt):
                calls.append('tick')
                return ExecutionOutput(
                    Status(execution_id='current', segment_id='AB', state='CANCELED',
                           error_code='CANCELED', navigation_stopped=True),
                    Command(now, now+.2, 'task', 'current', 'AB', '', 'stair'))
        class Goal:
            request = SimpleNamespace(task_id='task', execution_id='current', segment_id='AB')
            is_cancel_requested = True
            is_active = True
            canceled_called = False
            def canceled(self):
                self.canceled_called = True
        published = []
        harness = SimpleNamespace(
            lock=RLock(), reserved=True, reserved_execution_id='current',
            pending_cancel=False, goal_handle=None, completion=None,
            core=Core(), now=lambda: 10.0, period=.05,
            publish_output=published.append,
            get_logger=lambda: SimpleNamespace(error=lambda message: None))
        goal = Goal()
        response = SegmentNavigation.on_cancel(harness, goal)
        self.assertEqual(response, CancelResponse.ACCEPT)
        self.assertTrue(harness.pending_cancel)
        self.assertEqual(calls, [])
        coroutine = SegmentNavigation.execute(harness, goal)
        coroutine.send(None)  # Advance the real async callback until its Future await.
        self.assertEqual(calls, ['start', 'cancel', 'tick'])
        self.assertTrue(published)
        self.assertFalse(any(output.command.valid for output in published))
        harness.completion.set_result(published[-1].status)
        with self.assertRaises(StopIteration) as completed:
            coroutine.send(None)
        self.assertFalse(completed.exception.value.success)
        self.assertEqual(completed.exception.value.error_code, 'CANCELED')
        self.assertTrue(goal.canceled_called)
        self.assertFalse(harness.reserved)
        self.assertFalse(harness.pending_cancel)

    def test_old_execution_cannot_cancel_reserved_goal(self):
        harness = SimpleNamespace(lock=RLock(), reserved=True,
                                  reserved_execution_id='current', pending_cancel=False)
        goal = SimpleNamespace(request=SimpleNamespace(execution_id='old'))
        self.assertEqual(SegmentNavigation.on_cancel(harness, goal), CancelResponse.REJECT)
        self.assertFalse(harness.pending_cancel)

    def test_completed_core_result_wins_over_late_cancel(self):
        for terminal in ('SUCCEEDED', 'FAILED'):
            goal = SimpleNamespace(request=SimpleNamespace(execution_id='current'))
            harness = SimpleNamespace(lock=RLock(), reserved=True,
                                      reserved_execution_id='current', pending_cancel=False,
                                      goal_handle=goal, core=SimpleNamespace(
                                          active=False, status=Status(state=terminal)))
            self.assertEqual(SegmentNavigation.on_cancel(harness, goal), CancelResponse.REJECT)
            self.assertFalse(harness.pending_cancel)


class IngressTests(unittest.TestCase):
    def command_harness(self):
        accepted = []
        harness = SimpleNamespace(
            route={'frame': 'map'}, now=lambda: 10.0,
            get_logger=lambda: SimpleNamespace(warning=lambda message: None),
            sim=SimpleNamespace(accept=accepted.append))
        return harness, accepted

    def command(self, issued=10.0, expires=10.2, vx=0.3):
        message = ControlCommand(execution_id='current', segment_id='AB',
                                 valid=True, behavior='TRACK', vx=vx, wz=0.0)
        message.header.frame_id = 'map'
        message.header.stamp = stamp(issued)
        message.valid_until = stamp(expires)
        return message

    def test_future_expired_and_nonfinite_commands_do_not_enter_stamp_cache(self):
        for message in (self.command(issued=11.0, expires=11.2),
                        self.command(issued=9.0, expires=9.2),
                        self.command(vx=float('nan'))):
            harness, accepted = self.command_harness()
            SimExecutor.command(harness, message)
            self.assertEqual(accepted, [])

    def test_fresh_command_is_forwarded(self):
        harness, accepted = self.command_harness()
        SimExecutor.command(harness, self.command())
        self.assertEqual(len(accepted), 1)
        self.assertEqual(accepted[0].execution_id, 'current')
        self.assertEqual(accepted[0].vx, 0.3)

    def test_wrong_frame_motion_feedback_is_ignored(self):
        accepted = []
        harness = SimpleNamespace(route={'frame': 'map'}, lock=RLock(),
                                  core=SimpleNamespace(update_motion=accepted.append))
        message = MotionStatus(execution_id='current', segment_id='AB')
        message.header.frame_id = 'wrong_frame'
        SegmentNavigation.on_motion(harness, message)
        self.assertEqual(accepted, [])
        message.header.frame_id = 'map'
        SegmentNavigation.on_motion(harness, message)
        self.assertEqual(len(accepted), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
