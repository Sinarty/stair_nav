#!/usr/bin/env python3
"""Deterministic Action client callback race tests (requires built ROS interfaces).

No ROS graph or hardware is used. Real TaskManager callback methods receive
controlled futures/messages, including deliberately late generations.
"""
from concurrent.futures import Future
from dataclasses import replace
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
        self.cancel_future = Future()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Harness:
    feedback = TaskManager.feedback
    accepted = TaskManager.accepted
    result = TaskManager.result
    cancel_segment = TaskManager.cancel_segment
    cancel_response = TaskManager.cancel_response
    readiness = TaskManager.readiness
    _fresh = TaskManager._fresh
    on_motion = TaskManager.on_motion

    def __init__(self):
        from pathlib import Path
        from stair_nav.core.config import load_route
        from stair_nav.core.mission import MissionCore, Readiness, load_mission
        root = Path(__file__).resolve().parents[1]
        route = load_route(root/'stair_nav/config/demo_route.yaml')
        config = load_mission(root/'stair_nav/config/inspection_mission.yaml', route)
        self.route, self.config = route, config
        ids = iter(('current', 'next', 'last'))
        self.core = MissionCore(route, config, id_factory=lambda: next(ids))
        self.lock = RLock()
        self.handles = {}
        self.pending_cancels = set()
        self.result_received = set()
        self.last_result_at = {}
        self.segment_goal_ids = {('task-current', 'current'): bytes(range(16)).hex()}
        self.ready = Readiness(ready=True, released=True, stamp=10., server_instance='server',
                               target_segment_id='AB')
        self.core.start('inspection_demo', 'task-current', 10.)
        self.core.tick(10., self.ready, True)
        self.key = ('task-current', 'current')

    def now(self):
        return 10.

    def tick(self, now=10.):
        effects = self.core.tick(now, self.ready, True)
        for effect in effects:
            if effect.kind == 'CANCEL':
                self.cancel_segment((effect.task_id, effect.execution_id))
        return effects

    def running(self):
        handle = Handle()
        self.accepted(done(handle), *self.key)
        self.tick()
        return handle


def feedback(state='EXECUTING', execution_id='current', task_id='task-current'):
    message = SegmentStatus(task_id=task_id, execution_id=execution_id, segment_id='AB',
                            state=state, phase='tracking', progress=.6)
    message.header.stamp = stamp(10.)
    return SimpleNamespace(feedback=SimpleNamespace(status=message))


def success():
    return done(SimpleNamespace(status=4, result=NavigateSegment.Result(
        success=True, navigation_stopped=True, hold_confirmed=True)))


class CallbackTests(unittest.TestCase):
    def test_mission_cancel_completion_waits_for_ros_canceling_transition(self):
        from stair_nav.core.mission import Effect, MissionStatus
        completion = Future()
        handle = SimpleNamespace(is_cancel_requested=False)
        manager = SimpleNamespace(
            core=SimpleNamespace(status=MissionStatus(task_id='task', state='CANCELED')),
            mission_task_id='task', mission_completion=completion,
            action_cancel_pending=True, mission_handle=handle, pending_terminal=None)
        effects = [Effect('TERMINAL', 'task', '', '', {'state': 'CANCELED'})]
        TaskManager.apply_effects(manager, effects)
        self.assertFalse(completion.done())
        TaskManager.deliver_terminal(manager)
        self.assertFalse(completion.done())
        handle.is_cancel_requested = True
        TaskManager.deliver_terminal(manager)
        self.assertTrue(completion.done())
        self.assertEqual(completion.result().state, 'CANCELED')
        self.assertIsNone(manager.pending_terminal)

    def test_feedback_is_only_adjudicated_by_core_tick(self):
        manager = Harness()
        manager.running()
        manager.feedback(feedback(), *manager.key)
        self.assertEqual(manager.core.status.segment_progress, 0.)
        manager.tick()
        self.assertEqual(manager.core.status.segment_progress, .6)
        self.assertEqual(manager.core.status.state, 'RUNNING')

    def test_segment_success_feedback_is_not_task_success(self):
        manager = Harness()
        manager.running()
        manager.feedback(feedback('SUCCEEDED'), *manager.key)
        manager.tick()
        self.assertEqual(manager.core.status.state, 'RUNNING')
        self.assertEqual(manager.core.status.segment_state, 'SUCCEEDED')
        self.assertEqual(manager.core.status.completed_segments, 0)

    def test_wrong_embedded_feedback_ids_are_ignored(self):
        manager = Harness()
        manager.running()
        manager.feedback(feedback(task_id='old-task'), *manager.key)
        manager.feedback(feedback(execution_id='old'), *manager.key)
        wrong_segment = feedback()
        wrong_segment.feedback.status.segment_id = 'BC'
        manager.feedback(wrong_segment, *manager.key)
        manager.tick()
        self.assertEqual(manager.core.status.segment_progress, 0.)

    def test_old_feedback_and_result_cannot_change_current_task(self):
        manager = Harness()
        manager.running()
        manager.feedback(feedback(execution_id='old', task_id='retired'), 'retired', 'old')
        manager.result(success(), 'retired', 'old')
        manager.tick()
        self.assertEqual(manager.core.status.task_id, 'task-current')
        self.assertEqual(manager.core.status.execution_id, 'current')
        self.assertEqual(manager.core.status.completed_segments, 0)
        self.assertEqual(manager.core.status.segment_progress, 0.)

    def test_old_accepted_handle_is_registered_then_canceled(self):
        manager = Harness()
        manager.running()
        handle = Handle()
        manager.accepted(done(handle), 'retired', 'old')
        self.assertIs(manager.handles[('retired', 'old')], handle)
        self.assertEqual(handle.cancel_calls, 0)
        manager.tick()
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(manager.core.status.execution_id, 'current')
        # Result collection remains wired up even for a stale accepted handle.
        handle.result_future.set_result(SimpleNamespace(status=5, result=NavigateSegment.Result(
            success=False, navigation_stopped=True)))
        self.assertIn(('retired', 'old'), manager.result_received)
        self.assertNotIn(('retired', 'old'), manager.handles)

    def test_cancel_before_goal_acceptance_cancels_late_handle(self):
        manager = Harness()
        self.assertTrue(manager.core.cancel(10.))
        manager.core.drain_effects()
        handle = Handle()
        manager.accepted(done(handle), *manager.key)
        manager.tick()
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(manager.core.status.state, 'CANCELING')
        self.assertTrue(manager.core.status.cancel_requested)

    def test_pending_cancel_request_is_not_lost_when_handle_arrives(self):
        manager = Harness()
        manager.cancel_segment(manager.key)
        self.assertIn(manager.key, manager.pending_cancels)
        handle = Handle()
        manager.accepted(done(handle), *manager.key)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertNotIn(manager.key, manager.pending_cancels)

    def test_acceptance_timeout_late_handle_is_still_canceled(self):
        manager = Harness()
        manager.tick(14.)
        manager.tick(20.)
        self.assertEqual(manager.core.status.state, 'FAULT_BLOCKED')
        handle = Handle()
        manager.accepted(done(handle), *manager.key)
        manager.tick(21.)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(manager.core.status.state, 'FAULT_BLOCKED')

    def test_duplicate_success_only_advances_one_segment(self):
        manager = Harness()
        manager.running()
        manager.result(success(), *manager.key)
        manager.result(success(), *manager.key)
        manager.tick()
        self.assertEqual(manager.core.status.completed_segments, 1)
        self.assertEqual(manager.core.status.segment_id, 'BC')
        self.assertEqual(manager.core.status.state, 'WAITING_READY')
        self.assertEqual(manager.core.status.execution_id, '')
        manager.ready = replace(manager.ready, target_segment_id='BC')
        manager.tick()
        self.assertEqual(manager.core.status.execution_id, 'next')

    def test_old_segment_result_does_not_invalidate_next_readiness_snapshot(self):
        manager = Harness()
        manager.running()
        manager.result(success(), *manager.key)
        manager.tick()
        self.assertEqual(manager.last_result_at['task-current'], 10.)
        manager.now = lambda: 11.
        manager.result(success(), *manager.key)
        self.assertEqual(manager.last_result_at['task-current'], 10.)

    def test_cancel_ack_checks_ros_goal_uuid(self):
        from unique_identifier_msgs.msg import UUID
        for expected, identifier in ((False, [9]*16), (True, list(range(16)))):
            manager = Harness()
            manager.running()
            manager.core.cancel(10.)
            manager.core.drain_effects()
            response = SimpleNamespace(goals_canceling=[SimpleNamespace(goal_id=UUID(uuid=identifier))])
            manager.cancel_response(done(response), *manager.key)
            manager.tick()
            self.assertEqual(manager.core.status.cancel_accepted, expected)
            self.assertEqual(manager.core.status.state,
                             'CANCELING' if expected else 'FAULT_BLOCKED')

    def test_failed_result_future_becomes_event_and_cleanup(self):
        manager = Harness()
        handle = manager.running()
        failed = Future()
        failed.set_exception(RuntimeError('test transport failure'))
        manager.result(failed, *manager.key)
        self.assertEqual(manager.core.status.state, 'RUNNING')
        manager.tick()
        self.assertEqual(manager.core.status.state, 'CANCELING')
        self.assertEqual(manager.core.status.error_code, 'ACTION_TRANSPORT')
        self.assertEqual(handle.cancel_calls, 1)

    def test_accepted_handle_result_registration_failure_still_cancels(self):
        class BrokenHandle(Handle):
            def get_result_async(self):
                raise RuntimeError('result registration failed')
        manager = Harness()
        handle = BrokenHandle()
        manager.accepted(done(handle), *manager.key)
        self.assertIs(manager.handles[manager.key], handle)
        manager.tick()
        self.assertEqual(manager.core.status.state, 'CANCELING')
        self.assertEqual(manager.core.status.error_code, 'ACTION_TRANSPORT')
        self.assertEqual(handle.cancel_calls, 1)


class ReadinessTests(unittest.TestCase):
    def manager(self):
        from stair_nav_interfaces.srv import GetNavigationState
        from stair_nav.core.models import Pose
        from stair_nav.ros_utils import pose_msg
        manager = Harness()
        manager.pose = pose_msg(Pose(0., 0., 0., 0., 10., 'map'), 'map', 10.)
        manager.motion = MotionStatus(
            execution_id='current', segment_id='AB', mode='flat', ready=True,
            stable=True, hold_confirmed=True,
            capabilities=['stair_motion', 'stair_yaw_correction', 'turn_in_place'],
            supported_modes=['stair', 'flat'])
        manager.motion.header.frame_id = 'map'
        manager.motion.header.stamp = stamp(10.)
        manager.snapshot = GetNavigationState.Response(
            server_instance='server', busy=False, control_released=True,
            status=SegmentStatus(execution_id='current', segment_id='AB',
                                 state='CANCELED', navigation_stopped=True))
        manager.snapshot.header.frame_id = 'map'
        manager.snapshot.header.stamp = stamp(10.)
        return manager

    def test_preflight_allows_supported_mode_before_segment_switch(self):
        manager = self.manager()
        facts = manager.readiness(10.)
        self.assertTrue(facts.ready)
        self.assertTrue(facts.released)
        self.assertEqual(facts.execution_id, 'current')
        self.assertEqual(facts.target_segment_id, 'AB')

    def test_stale_pose_motion_and_navigation_block_preflight(self):
        for source in ('pose', 'motion', 'snapshot'):
            manager = self.manager()
            getattr(manager, source).header.stamp = stamp(8.)
            self.assertFalse(manager.readiness(10.).ready, source)

    def test_release_requires_matching_stable_hold_terminal_and_idle(self):
        changes = (
            lambda m: setattr(m.motion, 'execution_id', 'retired'),
            lambda m: setattr(m.motion, 'segment_id', 'BC'),
            lambda m: setattr(m.motion, 'stable', False),
            lambda m: setattr(m.motion, 'hold_confirmed', False),
            lambda m: setattr(m.snapshot, 'busy', True),
            lambda m: setattr(m.snapshot.status, 'state', 'EXECUTING'),
            lambda m: setattr(m.snapshot.status, 'navigation_stopped', False),
        )
        for change in changes:
            manager = self.manager()
            change(manager)
            facts = manager.readiness(10.)
            self.assertFalse(facts.released)
            self.assertFalse(facts.ready)

    def test_new_motion_revokes_cached_release_before_dispatch(self):
        from stair_nav.core.mission import MissionCore
        changes = (('stable', False), ('hold_confirmed', False),
                   ('execution_id', 'other-execution'), ('segment_id', 'BC'))
        for field, value in changes:
            with self.subTest(field=field):
                manager = self.manager()
                manager.core = MissionCore(manager.route, manager.config,
                                           id_factory=lambda: 'new-execution')
                self.assertTrue(manager.core.start('inspection_demo', 'new-task', 10.))
                previous = getattr(manager.motion, field)
                setattr(manager.motion, field, value)
                # The released snapshot is still fresh, but a newer physical
                # observation now contradicts its release evidence.
                manager.motion.header.stamp = stamp(10.1)
                facts = manager.readiness(10.1)
                self.assertFalse(facts.released)
                self.assertFalse(facts.ready)
                effects = manager.core.tick(10.1, facts, True)
                self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
                self.assertEqual(manager.core.status.state, 'WAITING_READY')
                # A fresh matching release permits exactly one dispatch.
                setattr(manager.motion, field, previous)
                manager.motion.header.stamp = stamp(10.2)
                facts = manager.readiness(10.2)
                self.assertTrue(facts.ready)
                effects = manager.core.tick(10.2, facts, True)
                self.assertEqual([effect.segment_id for effect in effects
                                  if effect.kind == 'SEND'], ['AB'])
                self.assertFalse(any(effect.kind == 'SEND' for effect in
                                     manager.core.tick(10.3, facts, True)))

    def test_initial_idle_release_does_not_require_hold_confirmation(self):
        manager = self.manager()
        manager.snapshot.status = SegmentStatus(state='IDLE', navigation_stopped=True)
        manager.motion.execution_id = ''
        manager.motion.segment_id = ''
        manager.motion.hold_confirmed = False
        facts = manager.readiness(10.)
        self.assertTrue(facts.released)
        self.assertTrue(facts.ready)
        manager.motion.stable = False
        self.assertFalse(manager.readiness(10.).ready)

    def test_next_segment_waits_for_its_own_capabilities(self):
        from stair_nav.core.models import Pose
        from stair_nav.ros_utils import pose_msg
        manager = self.manager()
        manager.running()
        manager.result(success(), *manager.key)
        effects = manager.core.tick(10., manager.readiness(10.), True)
        self.assertEqual(manager.core.status.segment_id, 'BC')
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        effects = manager.core.tick(10., manager.readiness(10.), True)
        self.assertEqual([effect.segment_id for effect in effects
                          if effect.kind == 'SEND'], ['BC'])
        bc_key = ('task-current', manager.core.status.execution_id)
        manager.accepted(done(Handle()), *bc_key)
        manager.core.tick(10., manager.readiness(10.), True)
        self.assertEqual(manager.core.status.state, 'RUNNING')
        # The BC result and a post-result released snapshot both arrive before
        # the timer. Its input readiness still evaluates BC capabilities.
        manager.result(success(), *bc_key)
        manager.snapshot.status.execution_id = bc_key[1]
        manager.snapshot.status.segment_id = 'BC'
        manager.snapshot.status.state = 'SUCCEEDED'
        manager.motion.execution_id = bc_key[1]
        manager.motion.segment_id = 'BC'
        manager.motion.capabilities = ['stair_motion', 'stair_yaw_correction']
        manager.pose = pose_msg(Pose(6., 0., 2., 0., 10., 'map'), 'map', 10.)
        previous_facts = manager.readiness(10.)
        self.assertEqual(previous_facts.target_segment_id, 'BC')
        self.assertTrue(previous_facts.ready)
        self.assertTrue(previous_facts.released)
        effects = manager.core.tick(10., previous_facts, True)
        self.assertEqual(manager.core.status.segment_id, 'CD')
        self.assertEqual(manager.core.status.state, 'WAITING_READY')
        self.assertEqual(manager.core.status.completed_segments, 2)
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        # Even reusing the old facts on a later tick cannot authorize CD.
        self.assertFalse(any(effect.kind == 'SEND' for effect in
                             manager.core.tick(10., previous_facts, True)))
        current_facts = manager.readiness(10.)
        self.assertEqual(current_facts.target_segment_id, 'CD')
        self.assertFalse(current_facts.ready)
        self.assertIn('capabilities', current_facts.reason)
        effects = manager.core.tick(10., current_facts, True)
        self.assertEqual(manager.core.status.state, 'WAITING_READY')
        self.assertFalse(manager.core.status.error_code)
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        manager.motion.capabilities.append('turn_in_place')
        current_facts = manager.readiness(10.)
        self.assertTrue(current_facts.ready)
        effects = manager.core.tick(10., current_facts, True)
        self.assertEqual([effect.segment_id for effect in effects
                          if effect.kind == 'SEND'], ['CD'])
        manager.result(success(), *bc_key)  # Duplicate BC success cannot send CD again.
        effects = manager.core.tick(10., current_facts, True)
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))

    def test_localization_dropout_does_not_prevent_confirmed_cleanup(self):
        manager = self.manager()
        manager.pose.header.stamp = stamp(8.)
        facts = manager.readiness(10.)
        self.assertFalse(facts.ready)
        self.assertTrue(facts.released)

    def test_snapshot_older_than_result_cannot_release_or_dispatch(self):
        manager = self.manager()
        manager.last_result_at['task-current'] = 10.1
        facts = manager.readiness(10.2)
        self.assertFalse(facts.ready)
        self.assertFalse(facts.released)

    def test_future_or_wrong_frame_motion_does_not_poison_global_cache(self):
        manager = self.manager()
        original = manager.motion
        for frame, issued in (('map', 20.), ('wrong', 10.)):
            message = MotionStatus(ready=False)
            message.header.frame_id = frame
            message.header.stamp = stamp(issued)
            manager.on_motion(message)
            self.assertIs(manager.motion, original)


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
        harness = SimpleNamespace(route={'frame': 'map'}, lock=RLock(), now=lambda: 10.,
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
