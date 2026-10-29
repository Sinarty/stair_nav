#!/usr/bin/env python3
"""Deterministic Action client callback race tests (requires built ROS interfaces).

No ROS graph or hardware is used. Real TaskManager callback methods receive
controlled futures/messages, including deliberately late generations.
"""
from concurrent.futures import Future
from copy import deepcopy
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
        # These existing callback races exercise the original three route legs.
        # Approach behavior has its own tests below and in the core suite.
        config.pop('approach_segment', None)
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


class StatusEventHarness:
    """Run real status/event adapters without a ROS graph or DDS scheduling."""
    publish = TaskManager.publish
    publish_transition = TaskManager.publish_transition
    apply_effects = TaskManager.apply_effects

    def __init__(self):
        base = Harness()
        self.core, self.route, self.config = base.core, base.route, base.config
        self.core.status.state = 'IDLE'
        self.core.status.execution_id = ''
        self.mission_goal_id = 'goal-1'
        self.segment_goal_ids = {}
        self.latest_segment_feedback = None
        self.source_instance = 'manager-instance-1'
        self.event_sequence = 0
        self.snapshots, self.legacy, self.events, self.feedbacks, self.logs = [], [], [], [], []
        def publisher(items):
            return SimpleNamespace(publish=lambda message: items.append(deepcopy(message)))
        self.publisher = publisher(self.snapshots)
        self.legacy_publisher = publisher(self.legacy)
        self.event_publisher = publisher(self.events)
        self.mission_handle = SimpleNamespace(
            is_active=True,
            publish_feedback=lambda message: self.feedbacks.append(deepcopy(message)))

    def now(self):
        return 20.

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)


class StatusEventTests(unittest.TestCase):
    def test_same_tick_transitions_keep_original_snapshots_and_order(self):
        from stair_nav.core.mission import MissionCore, Readiness
        manager = StatusEventHarness()
        manager.core = MissionCore(manager.route, manager.config, id_factory=lambda: 'execution')
        self.assertTrue(manager.core.start('inspection_demo', 'new-task', 10.))
        effects = manager.core.tick(10., Readiness(
            ready=True, released=True, stamp=10., target_segment_id='AB'), True)
        transitions = [effect for effect in effects if effect.kind == 'TRANSITION']
        self.assertEqual([effect.snapshot.state for effect in transitions],
                         ['VALIDATING', 'WAITING_READY', 'DISPATCHING'])
        # Delayed publication must not rewrite the captured history.
        manager.core.status.state = 'FAILED'
        manager.apply_effects(transitions)
        expected = ['VALIDATING', 'WAITING_READY', 'DISPATCHING']
        self.assertEqual([message.state for message in manager.snapshots], expected)
        self.assertEqual([message.state for message in manager.legacy], expected)
        self.assertEqual([message.status.state for message in manager.feedbacks], expected)
        self.assertEqual([message.status.state for message in manager.events], expected)
        self.assertEqual([message.sequence for message in manager.events], [1, 2, 3])
        self.assertEqual([message.new_state for message in manager.events], expected)
        self.assertEqual([message.old_state for message in manager.events],
                         ['IDLE', 'VALIDATING', 'WAITING_READY'])
        self.assertTrue(all(message.kind == 'MISSION_STATE' for message in manager.events))
        self.assertTrue(all(message.source_instance == 'manager-instance-1'
                            for message in manager.events))
        self.assertTrue(all(message.status.task_id == 'new-task' for message in manager.events))
        self.assertEqual(manager.events[-1].status.execution_id, 'execution')
        self.assertEqual(manager.events[0].status.execution_id, '')

    def test_segment_feedback_events_preserve_stage_and_do_not_claim_task_success(self):
        from stair_nav.core.mission import MissionCore, Readiness
        manager = StatusEventHarness()
        manager.core = MissionCore(manager.route, manager.config, id_factory=lambda: 'execution')
        manager.core.start('inspection_demo', 'new-task', 10.)
        facts = Readiness(ready=True, released=True, stamp=10., target_segment_id='AB')
        manager.core.tick(10., facts, True)
        manager.core.post('ACCEPTED', 'new-task', 'execution', accepted=True)
        manager.core.post('FEEDBACK', 'new-task', 'execution', stamp=10.1,
                          segment_state='EXECUTING', phase='stair', progress=.4)
        manager.core.post('FEEDBACK', 'new-task', 'execution', stamp=10.2,
                          segment_state='ARRIVAL_CONFIRM', phase='geometric_candidate', progress=1.)
        effects = manager.core.tick(10.2, facts, True)
        manager.apply_effects([effect for effect in effects
                               if effect.kind in ('TRANSITION', 'SEGMENT_TRANSITION')])
        segment_events = [message for message in manager.events if message.kind == 'SEGMENT_STATE']
        self.assertEqual([message.new_state for message in segment_events],
                         ['EXECUTING', 'ARRIVAL_CONFIRM'])
        self.assertEqual([message.status.phase for message in segment_events],
                         ['stair', 'geometric_candidate'])
        self.assertTrue(all(message.status.state == 'RUNNING' for message in segment_events))
        self.assertTrue(all(message.status.completed_segments == 0 for message in segment_events))

    def test_periodic_snapshot_does_not_create_transition_or_advance_event_sequence(self):
        manager = StatusEventHarness()
        message = manager.publish()
        self.assertEqual(message.state, manager.core.status.state)
        self.assertEqual(len(manager.snapshots), 1)
        self.assertEqual(manager.events, [])
        self.assertEqual(manager.event_sequence, 0)

    def test_events_and_snapshots_work_without_an_action_client(self):
        from stair_nav.core.mission import Effect, MissionStatus
        manager = StatusEventHarness()
        manager.mission_handle = None
        snapshot = MissionStatus(task_id='service-task', state='VALIDATING')
        effect = Effect('TRANSITION', 'service-task', '', '',
                        dict(old='IDLE', new='VALIDATING', event='START',
                             reason='Mission accepted', code='', time=10.), snapshot)
        manager.apply_effects([effect])
        self.assertEqual(manager.events[0].status.task_id, 'service-task')
        self.assertEqual(manager.snapshots[0].state, 'VALIDATING')
        self.assertEqual(manager.feedbacks, [])


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

    def approach_manager(self, pose=None):
        from stair_nav.core.mission import MissionCore
        from stair_nav.core.models import Pose
        from stair_nav.ros_utils import pose_msg
        manager = self.manager()
        manager.config = dict(manager.config, approach_segment='APPROACH_A')
        manager.core = MissionCore(manager.route, manager.config,
                                   id_factory=lambda: 'approach-execution')
        manager.core.start('inspection_demo', 'approach-task', 10.)
        manager.pose = pose_msg(pose or Pose(-1., .4, 0., .6, 10., 'map'), 'map', 10.)
        manager.snapshot.status = SegmentStatus(state='IDLE', navigation_stopped=True)
        manager.motion.execution_id = ''
        manager.motion.segment_id = ''
        return manager

    def test_approach_preflight_uses_live_position_without_requiring_start_at_A(self):
        manager = self.approach_manager()
        # The approach needs flat travel and turning, not stair capabilities.
        manager.motion.capabilities = ['turn_in_place']
        manager.motion.supported_modes = ['flat']
        facts = manager.readiness(10.)
        self.assertTrue(facts.ready, facts.reason)
        self.assertTrue(facts.released)
        self.assertIs(facts.at_start, False)
        self.assertEqual(facts.target_segment_id, 'APPROACH_A')
        effects = manager.core.tick(10., facts, True)
        self.assertEqual([effect.segment_id for effect in effects if effect.kind == 'SEND'],
                         ['APPROACH_A'])

    def test_start_pose_check_includes_heading_and_does_not_guess_from_stale_pose(self):
        from stair_nav.core.models import Pose
        at_A = self.approach_manager(Pose(0., 0., 0., 0., 10., 'map'))
        self.assertIs(at_A.readiness(10.).at_start, True)
        turned = self.approach_manager(Pose(0., 0., 0., 1., 10., 'map'))
        self.assertIs(turned.readiness(10.).at_start, False)
        stale = self.approach_manager()
        stale.pose.header.stamp = stamp(8.)
        facts = stale.readiness(10.)
        self.assertIsNone(facts.at_start)
        self.assertFalse(facts.ready)
        effects = stale.core.tick(10., facts, True)
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))

    def test_already_at_A_skips_approach_even_without_approach_turning_capability(self):
        from stair_nav.core.models import Pose
        manager = self.approach_manager(Pose(0., 0., 0., 0., 10., 'map'))
        manager.motion.capabilities = ['stair_motion', 'stair_yaw_correction']
        facts = manager.readiness(10.)
        self.assertFalse(facts.ready, 'approach must require its own turning capability')
        self.assertIs(facts.at_start, True)
        effects = manager.core.tick(10., facts, True)
        self.assertEqual(manager.core.status.segment_id, 'AB')
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        facts = manager.readiness(10.)
        self.assertTrue(facts.ready, facts.reason)
        effects = manager.core.tick(10., facts, True)
        self.assertEqual([effect.segment_id for effect in effects if effect.kind == 'SEND'], ['AB'])

    def test_arriving_at_A_while_waiting_skips_unneeded_approach_capabilities(self):
        from stair_nav.core.models import Pose
        from stair_nav.ros_utils import pose_msg
        manager = self.approach_manager(Pose(.13, 0., 0., 0., 10., 'map'))
        manager.motion.capabilities = ['stair_motion', 'stair_yaw_correction']
        facts = manager.readiness(10.)
        self.assertIs(facts.at_start, False)
        self.assertFalse(facts.ready)
        self.assertFalse(any(effect.kind == 'SEND' for effect in manager.core.tick(10., facts, True)))
        # A new localization estimate settles inside A before any goal was sent.
        manager.pose = pose_msg(Pose(.11, 0., 0., 0., 10., 'map'), 'map', 10.)
        facts = manager.readiness(10.)
        self.assertIs(facts.at_start, True)
        effects = manager.core.tick(10., facts, True)
        self.assertEqual(manager.core.status.segment_id, 'AB')
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        facts = manager.readiness(10.)
        self.assertTrue(facts.ready, facts.reason)
        effects = manager.core.tick(10., facts, True)
        self.assertEqual([effect.segment_id for effect in effects if effect.kind == 'SEND'], ['AB'])

    def test_leaving_A_while_waiting_rechecks_approach_before_dispatch(self):
        from stair_nav.core.models import Pose
        from stair_nav.ros_utils import pose_msg
        manager = self.approach_manager(Pose(.11, 0., 0., 0., 10., 'map'))
        manager.motion.stable = False
        facts = manager.readiness(10.)
        self.assertTrue(facts.at_start)
        self.assertFalse(facts.ready)
        manager.core.tick(10., facts, True)
        self.assertEqual(manager.core.status.segment_id, 'AB')
        manager.motion.stable = True
        manager.pose = pose_msg(Pose(.13, 0., 0., 0., 10., 'map'), 'map', 10.)
        facts = manager.readiness(10.)
        self.assertIs(facts.at_start, False)
        self.assertFalse(facts.ready, 'the AB start guard must reject the updated pose')
        effects = manager.core.tick(10., facts, True)
        self.assertEqual(manager.core.status.segment_id, 'APPROACH_A')
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))
        facts = manager.readiness(10.)
        self.assertTrue(facts.ready, facts.reason)
        effects = manager.core.tick(10., facts, True)
        self.assertEqual([effect.segment_id for effect in effects if effect.kind == 'SEND'], ['APPROACH_A'])

    def test_A_guard_is_restored_after_approach_success(self):
        manager = self.approach_manager()
        manager.core.tick(10., manager.readiness(10.), True)
        manager.core.post('ACCEPTED', 'approach-task', 'approach-execution', accepted=True)
        manager.core.tick(10., manager.readiness(10.), True)
        manager.core.post('RESULT', 'approach-task', 'approach-execution',
                          status=4, success=True, navigation_stopped=True, hold_confirmed=True)
        manager.core.tick(10., manager.readiness(10.), True)
        self.assertEqual(manager.core.status.segment_id, 'AB')
        # Even a claimed downstream success cannot authorize AB while live
        # localization still lies away from A.
        manager.snapshot.status = SegmentStatus(
            execution_id='approach-execution', segment_id='APPROACH_A',
            state='SUCCEEDED', navigation_stopped=True)
        manager.motion.execution_id = 'approach-execution'
        manager.motion.segment_id = 'APPROACH_A'
        facts = manager.readiness(10.)
        self.assertFalse(facts.ready)
        self.assertIs(facts.at_start, False)
        effects = manager.core.tick(10., facts, True)
        self.assertFalse(any(effect.kind == 'SEND' for effect in effects))

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
        initial_statuses, initial_feedbacks = [], []
        class Core:
            active = False
            status = Status(task_id='task', execution_id='current', segment_id='AB', state='CHECKING')
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
            def publish_feedback(self, message):
                initial_feedbacks.append(message)
            def canceled(self):
                self.canceled_called = True
        published = []
        harness = SimpleNamespace(
            lock=RLock(), reserved=True, reserved_execution_id='current',
            pending_cancel=False, goal_handle=None, completion=None,
            core=Core(), now=lambda: 10.0, period=.05,
            publish_output=published.append,
            status_message=lambda status, now: SegmentNavigation.status_message(
                SimpleNamespace(route={'frame': 'map'}), status, now),
            statuses=SimpleNamespace(publish=initial_statuses.append),
            get_logger=lambda: SimpleNamespace(error=lambda message: None))
        goal = Goal()
        response = SegmentNavigation.on_cancel(harness, goal)
        self.assertEqual(response, CancelResponse.ACCEPT)
        self.assertTrue(harness.pending_cancel)
        self.assertEqual(calls, [])
        coroutine = SegmentNavigation.execute(harness, goal)
        coroutine.send(None)  # Advance the real async callback until its Future await.
        self.assertEqual(calls, ['start', 'cancel', 'tick'])
        self.assertEqual(initial_statuses[0].state, 'CHECKING')
        self.assertEqual(initial_feedbacks[0].status.state, 'CHECKING')
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
