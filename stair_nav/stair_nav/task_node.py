"""Asynchronous ROS adapter for the deterministic mission state machine.

Only core.tick() adjudicates queued downstream events. No callback blocks on
transport, planner work, a condition variable, or a nested executor spin.
"""
from copy import deepcopy
import json
import math
from threading import RLock
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
from stair_nav_interfaces.action import ExecuteMission, NavigateSegment
from stair_nav_interfaces.msg import MissionEvent, MissionStatus, MotionStatus, SegmentStatus
from stair_nav_interfaces.srv import GetNavigationState
from stair_nav.core.config import load_route, segment_config
from stair_nav.core.mission import Effect, MissionCore, Readiness, load_mission
from stair_nav.ros_utils import durable_qos, live_qos, pose_from_msg, seconds, stamp


TERMINAL = {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED'}
STATUS_FIELDS = (
    'task_id', 'route_id', 'execution_id', 'segment_id', 'segment_index',
    'segment_count', 'completed_segments', 'state', 'phase', 'segment_state',
    'segment_progress', 'error_code', 'error_message', 'navigation_stopped',
    'hold_confirmed', 'blocked', 'cancel_requested', 'cancel_sent',
    'cancel_accepted', 'downstream_terminal',
)


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('route_file', '')
        self.declare_parameter('mission_file', '')
        # A restarted standalone node never resumes an old mission.
        self.declare_parameter('autostart', False)
        route_file = self.get_parameter('route_file').value
        mission_file = self.get_parameter('mission_file').value
        if not route_file or not mission_file:
            raise ValueError('route_file and mission_file are required; provide validated site YAML files')
        self.route = load_route(route_file)
        self.config = load_mission(mission_file, self.route)
        self.core = MissionCore(self.route, self.config)
        self.lock = RLock()
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(MissionStatus, '/mission/status', durable_qos())
        self.event_publisher = self.create_publisher(MissionEvent, '/mission/events', durable_qos(100))
        self.source_instance = str(uuid.uuid4())
        self.event_sequence = 0
        self.legacy_publisher = self.create_publisher(SegmentStatus, '/task/status', durable_qos())
        self.client = ActionClient(self, NavigateSegment, '/navigate_segment', callback_group=self.group)
        self.state_client = self.create_client(GetNavigationState, '/navigation/get_state',
                                               callback_group=self.group)
        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose,
                                 live_qos(5), callback_group=self.group)
        self.create_subscription(MotionStatus, '/motion/status', self.on_motion,
                                 live_qos(5), callback_group=self.group)
        self.create_subscription(SegmentStatus, '/navigation/status', self.on_navigation,
                                 durable_qos(), callback_group=self.group)
        self.create_service(Trigger, '~/start', self.start_service, callback_group=self.group)
        self.create_service(Trigger, '~/cancel', self.cancel_service, callback_group=self.group)
        self.create_service(Trigger, '~/reset', self.reset_service, callback_group=self.group)
        self.server = ActionServer(
            self, ExecuteMission, '/execute_mission', self.execute_mission,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.group, result_timeout=60,
        )
        self.pose = None
        self.motion = None
        self.navigation = None
        self.latest_segment_feedback = None
        self.snapshot = None
        self.snapshot_future = None
        self.snapshot_requested = -math.inf
        self.snapshot_last_poll = -math.inf
        self.handles = {}
        self.segment_goal_ids = {}
        self.pending_cancels = set()
        self.result_received = set()
        self.last_result_at = {}
        self.reserved = False
        self.pending_mission_cancel = False
        self.action_cancel_pending = False
        self.pending_terminal = None
        self.mission_handle = None
        self.mission_completion = None
        self.mission_task_id = ''
        self.mission_goal_id = ''
        self.autostart_pending = bool(self.get_parameter('autostart').value)
        self.create_timer(.05, self.timer, callback_group=self.group)
        self.publish_transition(Effect('TRANSITION', '', '', '', dict(
            old='', new='IDLE', event='INITIALIZED', reason='Task manager ready',
            code='', time=self.now()), deepcopy(self.core.status)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def busy(self):
        return (self.reserved or self.core.status.state not in TERMINAL or
                self.core.status.blocked)

    def on_pose(self, message):
        with self.lock:
            self.pose = message

    def on_motion(self, message):
        with self.lock:
            issued = seconds(message.header.stamp)
            if (message.header.frame_id != self.route['frame'] or
                    not math.isfinite(issued) or issued > self.now() + .1):
                return
            if self.motion is None or issued >= seconds(self.motion.header.stamp):
                self.motion = message

    def on_navigation(self, message):
        with self.lock:
            self.navigation = message

    def _fresh(self, header, maximum_age, now):
        age = now-seconds(header.stamp)
        return (header.frame_id == self.route['frame'] and
                math.isfinite(age) and -.1 <= age <= maximum_age)

    def readiness(self, now, check_start=True):
        """Translate fresh observations into preflight and release facts.

        The segment layer owns mode switching and final arrival. The task layer
        checks supported modes before dispatch and waits physical hold/stability
        independently after a canceled/failed Action result.
        """
        settings = self.config['readiness']
        m, snap = self.motion, self.snapshot
        ready = True
        reasons = []
        fresh_motion = m is not None and self._fresh(m.header, settings['motion_max_age'], now)
        fresh_snapshot = snap is not None and self._fresh(
            snap.header, settings['navigation_max_age'], now)
        released = False
        execution_id = ''
        observed = 0.0
        server_instance = ''
        if not fresh_snapshot:
            ready = False
            reasons.append('navigation synchronization missing/stale')
        else:
            server_instance = snap.server_instance
            execution_id = snap.status.execution_id
            observed = seconds(snap.header.stamp)
            after_result = self.last_result_at.get(self.core.status.task_id, -math.inf)
            snapshot_current = seconds(snap.header.stamp) >= after_result
            if snap.busy or not snap.control_released or not snapshot_current:
                ready = False
                reasons.append('downstream busy or control release unconfirmed')
            if fresh_motion:
                initial = not execution_id and snap.status.state == 'IDLE'
                terminal = snap.status.state in ('SUCCEEDED', 'FAILED', 'CANCELED')
                matching = (m.execution_id == execution_id and
                            (initial or m.segment_id == snap.status.segment_id))
                released = bool(snapshot_current and snap.control_released and not snap.busy and matching and
                                m.stable and not m.fault_code and
                                (initial or (terminal and snap.status.navigation_stopped and
                                             m.hold_confirmed)))
                observed = min(observed, seconds(m.header.stamp))
        if not released:
            ready = False
            if fresh_snapshot and fresh_motion:
                reasons.append('current motion does not confirm control release')
        if not fresh_motion:
            ready = False
            reasons.append('motion feedback missing/stale')
        elif m.fault_code or not m.ready:
            ready = False
            reasons.append(f'motion unavailable: {m.fault_code or "not ready"}')
        sid = self.core.status.segment_id or self.config['sequence'][0]
        seg = segment_config(self.route, sid)
        if fresh_motion:
            required = set(seg.get('required_capabilities', []))
            for flag, capability in (('allow_turn_in_place', 'turn_in_place'),
                                     ('allow_reverse', 'reverse'),
                                     ('allow_stair_correction', 'stair_yaw_correction')):
                if seg['tracking'].get(flag, False):
                    required.add(capability)
            if seg['kind'] == 'stair':
                required.add('stair_motion')
            if not required.issubset(set(m.capabilities)):
                ready = False
                reasons.append('required motion capabilities unavailable')
            if not set(seg['allowed_modes']).issubset(set(m.supported_modes)):
                ready = False
                reasons.append('required motion modes unsupported')
        at_start = None
        try:
            if self.pose is None or not self._fresh(self.pose.header, settings['pose_max_age'], now):
                raise ValueError('localization missing/stale')
            p = pose_from_msg(self.pose)
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.yaw)):
                raise ValueError('localization invalid')
            first = segment_config(self.route, self.config['sequence'][0])
            start = self.route['nodes'][first['start']]
            target, a = start['position'], start['arrival']
            yaw_error = abs(math.atan2(math.sin(p.yaw-start['yaw']), math.cos(p.yaw-start['yaw'])))
            at_start = (math.hypot(p.x-target[0], p.y-target[1]) <= a['xy'] and
                        abs(p.z-target[2]) <= a['z'] and yaw_error <= a['yaw'])
            if (check_start and settings.get('require_start_node', True) and
                    sid != self.config.get('approach_segment') and
                    self.core.status.completed_segments == 0 and
                    self.core.status.state in ('IDLE', 'VALIDATING', 'WAITING_READY',
                                               'SUCCEEDED', 'FAILED', 'CANCELED', 'FAULT_BLOCKED')):
                if not at_start:
                    raise ValueError(f'robot must first reach start node {first["start"]}')
        except (ValueError, TypeError, AttributeError) as exc:
            ready = False
            reasons.append(str(exc))
        return Readiness(ready=ready, released=released, execution_id=execution_id,
                         stamp=observed, reason='; '.join(reasons), server_instance=server_instance,
                         target_segment_id=sid, at_start=at_start)

    def poll_snapshot(self, now):
        if self.snapshot_future is not None:
            if now-self.snapshot_requested <= 1.0:
                return
            self.state_client.remove_pending_request(self.snapshot_future)
            self.snapshot_future = None
        if now-self.snapshot_last_poll < .2 or not self.state_client.service_is_ready():
            return
        self.snapshot_last_poll = now
        self.snapshot_requested = now
        future = self.state_client.call_async(GetNavigationState.Request())
        self.snapshot_future = future
        future.add_done_callback(self.snapshot_done)

    def snapshot_done(self, future):
        with self.lock:
            if future is not self.snapshot_future:
                return
            self.snapshot_future = None
            try:
                snapshot = future.result()
                if snapshot.header.frame_id == self.route['frame']:
                    self.snapshot = snapshot
            except Exception as exc:
                self.get_logger().warning(f'navigation synchronization failed: {exc}')

    def on_goal(self, request):
        with self.lock:
            if self.busy() or request.route_id not in ('', self.config['route_id']):
                return GoalResponse.REJECT
            self.autostart_pending = False
            self.reserved = True
            self.pending_mission_cancel = False
            self.action_cancel_pending = False
            self.pending_terminal = None
            return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        with self.lock:
            if not self.reserved or (self.mission_handle is not None and
                                     self.mission_handle is not goal_handle):
                return CancelResponse.REJECT
            if self.mission_handle is goal_handle and self.core.status.state in TERMINAL:
                return CancelResponse.REJECT
            if self.core.status.blocked:
                return CancelResponse.REJECT
            self.pending_mission_cancel = True
            self.action_cancel_pending = True
            if self.mission_handle is goal_handle:
                if not self.core.cancel(self.now()):
                    return CancelResponse.REJECT
                self.apply_effects(self.core.drain_effects())
            return CancelResponse.ACCEPT

    async def execute_mission(self, goal_handle):
        task_id = str(uuid.uuid4())
        try:
            with self.lock:
                self.mission_handle = goal_handle
                self.mission_task_id = task_id
                self.mission_goal_id = bytes(goal_handle.goal_id.uuid).hex()
                self.mission_completion = Future()
                if not self.core.start(goal_handle.request.route_id or self.config['route_id'],
                                       task_id, self.now()):
                    goal_handle.abort()
                    return ExecuteMission.Result(success=False, task_id=task_id,
                        route_id=self.config['route_id'], state='FAILED', error_code='BUSY',
                        error_message='mission is busy or fault blocked')
                if self.pending_mission_cancel or goal_handle.is_cancel_requested:
                    self.core.cancel(self.now())
                self.apply_effects(self.core.drain_effects())
                completion = self.mission_completion
            terminal = await completion
            with self.lock:
                if terminal.state == 'SUCCEEDED':
                    goal_handle.succeed()
                elif terminal.state == 'CANCELED' and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return ExecuteMission.Result(
                    success=terminal.state == 'SUCCEEDED', task_id=terminal.task_id,
                    route_id=terminal.route_id, state=terminal.state,
                    error_code=terminal.error_code, error_message=terminal.error_message,
                    navigation_stopped=terminal.navigation_stopped,
                    hold_confirmed=terminal.hold_confirmed)
        except Exception as exc:
            with self.lock:
                self.get_logger().error(f'mission Action wrapper failed: {type(exc).__name__}: {exc}')
                self.core.cancel(self.now())
                self.apply_effects(self.core.drain_effects())
                if goal_handle.is_active:
                    goal_handle.abort()
                return ExecuteMission.Result(success=False, task_id=task_id,
                    route_id=self.config['route_id'], state='FAILED',
                    error_code='MISSION_WRAPPER_ERROR', error_message=str(exc),
                    navigation_stopped=False, hold_confirmed=False)
        finally:
            with self.lock:
                if self.mission_task_id == task_id:
                    self.mission_handle = None
                    self.mission_completion = None
                    self.reserved = False
                    self.pending_mission_cancel = False
                    self.action_cancel_pending = False
                    self.pending_terminal = None

    def start_service(self, request, response):
        with self.lock:
            self.autostart_pending = False
            if self.busy():
                response.success = False
                response.message = 'BUSY: mission active or fault blocked; inspect /mission/status'
            else:
                task_id = str(uuid.uuid4())
                response.success = self.core.start(self.config['route_id'], task_id, self.now())
                self.mission_goal_id = ''
                self.apply_effects(self.core.drain_effects())
                response.message = f'task_id={task_id}' if response.success else 'mission start rejected'
            self.publish()
        return response

    def cancel_service(self, request, response):
        with self.lock:
            self.autostart_pending = False
            if self.reserved and self.mission_handle is None:
                self.pending_mission_cancel = True
                response.success = True
            else:
                response.success = self.core.cancel(self.now())
            self.apply_effects(self.core.drain_effects())
            response.message = ('cancel intent accepted; await result and fresh hold/stability'
                                if response.success else 'no cancellable active mission')
            self.publish()
        return response

    def reset_service(self, request, response):
        with self.lock:
            self.autostart_pending = False
            if self.reserved:
                response.success, response.message = False, 'mission Action has not finished'
            else:
                response.success, response.message = self.core.reset(self.now(), self.readiness(self.now(), check_start=False))
                self.apply_effects(self.core.drain_effects())
            self.publish()
        return response

    def timer(self):
        with self.lock:
            now = self.now()
            self.poll_snapshot(now)
            if self.autostart_pending and self.client.server_is_ready() and self.state_client.service_is_ready():
                self.autostart_pending = False
                if not self.busy():
                    self.core.start(self.config['route_id'], str(uuid.uuid4()), now)
            self.apply_effects(self.core.tick(now, self.readiness(now), self.client.server_is_ready()))
            self.deliver_terminal()
            self.publish()

    def apply_effects(self, effects):
        for effect in effects:
            key = (effect.task_id, effect.execution_id)
            if effect.kind == 'SEND':
                goal_uuid = uuid.uuid4()
                self.segment_goal_ids[key] = goal_uuid.hex
                goal = NavigateSegment.Goal(task_id=effect.task_id,
                    execution_id=effect.execution_id, segment_id=effect.segment_id)
                try:
                    future = self.client.send_goal_async(goal,
                        feedback_callback=lambda msg, k=key: self.feedback(msg, *k),
                        goal_uuid=UUID(uuid=list(goal_uuid.bytes)))
                    future.add_done_callback(lambda f, k=key: self.accepted(f, *k))
                except Exception as exc:
                    self.core.post('ERROR', *key, operation='send', message=str(exc))
            elif effect.kind == 'CANCEL':
                self.cancel_segment(key)
            elif effect.kind == 'TERMINAL':
                if (self.mission_task_id == effect.task_id and self.mission_completion is not None
                        and not self.mission_completion.done()):
                    terminal = deepcopy(self.core.status)
                    if (terminal.state == 'CANCELED' and self.action_cancel_pending and
                            self.mission_handle is not None and
                            not self.mission_handle.is_cancel_requested):
                        # on_cancel() precedes the rclpy CANCELING transition.
                        # Let the timer deliver after the protocol state changes.
                        self.pending_terminal = terminal
                    else:
                        self.mission_completion.set_result(terminal)
            elif effect.kind in ('TRANSITION', 'SEGMENT_TRANSITION'):
                self.get_logger().info(json.dumps(dict(
                    task_id=effect.task_id, segment_id=effect.segment_id,
                    execution_id=effect.execution_id,
                    goal_id=self.mission_goal_id,
                    segment_goal_id=self.segment_goal_ids.get(key, ''), **effect.data),
                    ensure_ascii=False, sort_keys=True))
                self.publish_transition(effect)

    def deliver_terminal(self):
        if (self.pending_terminal is not None and self.mission_handle is not None and
                self.mission_handle.is_cancel_requested and
                self.mission_completion is not None and not self.mission_completion.done()):
            self.mission_completion.set_result(self.pending_terminal)
            self.pending_terminal = None

    def feedback(self, message, task_id, execution_id):
        incoming = message.feedback.status
        if incoming.task_id != task_id or incoming.execution_id != execution_id:
            return
        with self.lock:
            if (task_id == self.core.status.task_id and
                    execution_id == self.core.status.execution_id and
                    incoming.segment_id != self.core.status.segment_id):
                return
            if (task_id == self.core.status.task_id and
                    execution_id == self.core.status.execution_id and self.core.active):
                self.latest_segment_feedback = deepcopy(incoming)
            self.core.post('FEEDBACK', task_id, execution_id,
                phase=incoming.phase, segment_state=incoming.state, progress=incoming.progress,
                error_code=incoming.error_code, error_message=incoming.error_message,
                stamp=seconds(incoming.header.stamp))

    def accepted(self, future, task_id, execution_id):
        key = (task_id, execution_id)
        with self.lock:
            try:
                handle = future.result()
            except Exception as exc:
                self.core.post('ERROR', task_id, execution_id, operation='send', message=str(exc))
                return
            # Record known acceptance before asking for the result. A local
            # get_result_async failure cannot turn a known execution into an
            # unknown send and strand its cancelable handle.
            self.core.post('ACCEPTED', task_id, execution_id, accepted=handle.accepted)
            if not handle.accepted:
                return
            self.handles[key] = handle
            try:
                handle.get_result_async().add_done_callback(lambda f, k=key: self.result(f, *k))
            except Exception as exc:
                self.core.post('ERROR', task_id, execution_id, operation='result', message=str(exc))
            # Do not abandon a handle returned after a cancellation/deadline.
            if key in self.pending_cancels:
                self.pending_cancels.discard(key)
                self.cancel_segment(key)

    def cancel_segment(self, key):
        if key in self.result_received:
            return
        handle = self.handles.get(key)
        if handle is None:
            self.pending_cancels.add(key)
            return
        try:
            handle.cancel_goal_async().add_done_callback(lambda f, k=key: self.cancel_response(f, *k))
        except Exception as exc:
            self.core.post('ERROR', *key, operation='cancel', message=str(exc))

    def cancel_response(self, future, task_id, execution_id):
        with self.lock:
            try:
                response = future.result()
                expected = self.segment_goal_ids.get((task_id, execution_id), '')
                accepted = any(bytes(item.goal_id.uuid).hex() == expected for item in response.goals_canceling)
                self.core.post('CANCEL_RESPONSE', task_id, execution_id, accepted=accepted)
            except Exception as exc:
                self.core.post('ERROR', task_id, execution_id, operation='cancel', message=str(exc))

    def result(self, future, task_id, execution_id):
        with self.lock:
            key = (task_id, execution_id)
            try:
                wrapped = future.result()
                result = wrapped.result
                if (key == (self.core.status.task_id, self.core.status.execution_id) and
                        key not in self.result_received):
                    self.last_result_at[task_id] = self.now()
                self.result_received.add(key)
                self.handles.pop(key, None)
                self.pending_cancels.discard(key)
                self.core.post('RESULT', task_id, execution_id, success=result.success,
                    navigation_stopped=result.navigation_stopped, hold_confirmed=result.hold_confirmed,
                    error_code=result.error_code, error_message=result.error_message, status=wrapped.status)
            except Exception as exc:
                self.core.post('ERROR', task_id, execution_id, operation='result', message=str(exc))

    def publish_transition(self, effect):
        """Publish the captured state, including states crossed within one tick."""
        status = effect.snapshot
        if status is None:
            return
        msg = self.publish(status=status)
        self.event_sequence += 1
        data = effect.data
        event = MissionEvent(
            source_instance=self.source_instance, sequence=self.event_sequence,
            kind='MISSION_STATE' if effect.kind == 'TRANSITION' else 'SEGMENT_STATE',
            old_state=data['old'], new_state=data['new'], event=data['event'],
            reason=data['reason'], error_code=data['code'], status=msg)
        event.header.stamp = stamp(data['time'])
        event.header.frame_id = self.route['frame']
        self.event_publisher.publish(event)

    def publish(self, status=None):
        status = self.core.status if status is None else status
        now = self.now()
        msg = MissionStatus()
        for name in STATUS_FIELDS:
            setattr(msg, name, getattr(status, name))
        msg.header.stamp = stamp(now)
        msg.header.frame_id = self.route['frame']
        msg.valid_until = stamp(now + self.config['readiness']['navigation_max_age'])
        msg.goal_id = self.mission_goal_id
        msg.segment_goal_id = self.segment_goal_ids.get((status.task_id, status.execution_id), '')
        self.publisher.publish(msg)
        # Existing monitoring clients keep their SegmentStatus subscription.
        legacy = SegmentStatus(task_id=status.task_id, execution_id=status.execution_id,
            segment_id=status.segment_id, phase=status.phase, state=status.state,
            progress=status.segment_progress, error_code=status.error_code,
            error_message=status.error_message, navigation_stopped=status.navigation_stopped,
            hold_confirmed=status.hold_confirmed)
        legacy.header = msg.header
        observed = self.latest_segment_feedback
        if (observed is not None and observed.task_id == status.task_id and
                observed.execution_id == status.execution_id):
            legacy.lateral_error = observed.lateral_error
            legacy.heading_error = observed.heading_error
            legacy.geometry_reached = observed.geometry_reached
        self.legacy_publisher.publish(legacy)
        if self.mission_handle is not None and self.mission_handle.is_active:
            self.mission_handle.publish_feedback(ExecuteMission.Feedback(status=msg))
        return msg

    def destroy_node(self):
        with self.lock:
            self.core.cancel(self.now())
            self.apply_effects(self.core.drain_effects())
        self.server.destroy()
        self.client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
