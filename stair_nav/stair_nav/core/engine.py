"""Clock-driven segment executor. All methods are serialized by its caller.

Planning runs in a worker; only tick() consumes its result. A canceled/timed-out
generation can therefore never publish a path into a subsequent execution.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from enum import Enum
import math
from threading import Event
from .config import segment_config
from .models import Command, ExecutionOutput, NavError, Pose, Status, wrap_angle
from .planning import plan
from .tracking import PurePursuit


class State(str, Enum):
    IDLE = 'IDLE'
    CHECKING = 'CHECKING'
    PLANNING = 'PLANNING'
    WAIT_READY = 'WAIT_READY'
    EXECUTING = 'EXECUTING'
    ARRIVAL_CONFIRM = 'ARRIVAL_CONFIRM'
    PAUSED_OBSTACLE = 'PAUSED_OBSTACLE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


TERMINAL = {State.IDLE, State.SUCCEEDED, State.FAILED, State.CANCELED}


class SegmentExecutor:
    def __init__(self, route, planning_executor=None):
        self.route = route
        self._pool = planning_executor or ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='segment_planner')
        self._owns_pool = planning_executor is None
        self.status = Status()
        self.path = None
        self._path_stamp = None
        self.pose = None
        self.motion = None
        self._tracker = None
        self._target = None
        self._future = None
        self._cancel_event = Event()
        self._seen = set()
        self._blocked = {}
        self._last_tick = None
        self._arrival_since = None
        self._arrival_started = None
        self._execution_started = None
        self._obstacle = None
        self._pause_started = None
        self._pause_state = None
        self._pause_phase = ''
        self._pause_state_since = None

    @property
    def active(self):
        return self.status.state not in TERMINAL

    def start(self, task_id, execution_id, segment_id, now):
        if self.active:
            return False
        if not task_id or not execution_id or execution_id in self._seen:
            raise NavError('INVALID_ID', 'Task/execution IDs must be nonempty; execution ID must be new')
        self.seg = segment_config(self.route, segment_id)
        self.goal = self.route['nodes'][self.seg['end']]
        self._seen.add(execution_id)
        self.status = Status(task_id=task_id, execution_id=execution_id,
                             segment_id=segment_id, phase='checking', state=State.CHECKING)
        self.path = None
        self._path_stamp = None
        self.motion = None
        self._tracker = None
        self._target = None
        self._future = None
        self._cancel_event = Event()
        self._started = now
        self._state_since = now
        self._last_tick = now
        self._arrival_since = None
        self._arrival_started = None
        self._execution_started = None
        self._obstacle = None
        self._pause_started = None
        self._pause_state = None
        self._pause_phase = ''
        self._pause_state_since = None
        return True

    def update_pose(self, pose):
        # Invalid newest input must be visible to tick rather than silently
        # retaining an earlier valid localization sample.
        if (self.pose is None or not math.isfinite(self.pose.stamp) or
                not math.isfinite(pose.stamp) or pose.stamp >= self.pose.stamp):
            self.pose = pose

    def update_motion(self, feedback):
        if (feedback.execution_id != self.status.execution_id or
                feedback.segment_id != self.status.segment_id or
                not feedback.execution_id or not math.isfinite(feedback.stamp)):
            return False
        if self.motion is not None and feedback.stamp <= self.motion.stamp:
            return False
        self.motion = feedback
        return True

    def set_blocked(self, segment_id, blocked, reason='', stamp=0):
        segment_id = segment_id or '*'
        old = self._blocked.get(segment_id)
        if old is None or stamp >= old[2]:
            self._blocked[segment_id] = (bool(blocked), reason, stamp)

    def update_obstacle(self, observation):
        """Accept only this execution's observations; invalid input stays visible."""
        if (not observation.execution_id or
                observation.execution_id != self.status.execution_id):
            return False
        previous = self._obstacle
        if (previous is not None and math.isfinite(previous.stamp) and
                math.isfinite(observation.stamp) and observation.stamp < previous.stamp):
            return False
        self._obstacle = observation
        return True

    def _pause_obstacle(self, now, phase):
        if self.status.state != State.PAUSED_OBSTACLE:
            self._pause_state = self.status.state
            self._pause_phase = self.status.phase
            self._pause_state_since = self._state_since
            self._pause_started = now
            self._transition(State.PAUSED_OBSTACLE, now)
            if self._tracker is not None:
                self._tracker.reset_motion()
        self.status.phase = phase
        self.status.geometry_reached = False
        self._arrival_since = None

    def _obstacle_gate(self, now, has_pose):
        """Pause without ending the Action; sensor and motion leases still run."""
        if not self.seg['obstacle_stop']['enabled']:
            return True
        # Planning never authorizes motion. Consume its result normally, then
        # gate WAIT_READY, so a blocked corridor cannot hide a failed worker.
        if self.status.state == State.PLANNING:
            return True
        paused = self.status.state == State.PAUSED_OBSTACLE
        original = self._pause_state if paused else self.status.state
        if not has_pose and (original != State.CHECKING or
                             now - self._started > self.seg['timeouts']['pose']):
            raise NavError('POSE_MISSING', 'Localization unavailable while waiting for obstacle clearance')
        if paused:
            if self.motion is not None:
                self._check_motion(now, False)
            elif now - self._pause_started > self.seg['timeouts']['ready']:
                raise NavError('READY_TIMEOUT', 'No current-execution motion feedback during obstacle hold')
            maximum = self.seg['obstacle_stop']['max_pause']
            if maximum and now - self._pause_started >= maximum:
                raise NavError('OBSTACLE_PAUSE_TIMEOUT', 'Obstacle pause exceeded its configured deadline')
        elif original in (State.EXECUTING, State.ARRIVAL_CONFIRM):
            self._check_motion(now, True)
        observation = self._obstacle
        usable = bool(observation is not None and observation.valid is True and
                      observation.execution_id == self.status.execution_id and
                      math.isfinite(observation.stamp) and math.isfinite(observation.valid_until) and
                      observation.stamp <= now + .1 and
                      observation.stamp <= observation.valid_until and now < observation.valid_until)
        if usable and self.path is not None:
            usable = (math.isfinite(observation.path_stamp) and
                      observation.path_stamp + 1e-6 >= self._path_stamp)
        if (observation is not None and math.isfinite(observation.stamp) and
                observation.stamp > now + .1):
            # Fail closed on this sample without letting its invalid future
            # stamp suppress later, correctly timed recovery observations.
            self._obstacle = None
        if not usable or observation.blocked:
            self._pause_obstacle(now, 'obstacle_blocked' if usable else 'obstacle_invalid')
            return False
        if not paused:
            return True
        if (not has_pose or self.motion is None or self.motion.stamp <= self._pause_started or
                not self.motion.ready or
                not self.motion.stable or not self.motion.hold_confirmed or
                self.motion.mode != self.seg['mode']):
            self.status.phase = 'waiting_hold'
            return False
        duration = now - self._pause_started
        for name in ('_execution_started', '_arrival_started'):
            value = getattr(self, name)
            if value is not None:
                setattr(self, name, value + duration)
        self._transition(self._pause_state, now)
        self._state_since = self._pause_state_since + duration
        self.status.phase = self._pause_phase
        self._pause_started = None
        self._pause_state = None
        self._pause_state_since = None
        if self._tracker is not None:
            self._tracker.reset_motion()
        # Keep this output a fresh HOLD. The next tick revalidates localization,
        # the clear lease, readiness and path matching before issuing TRACK.
        return False

    def _transition(self, state, now):
        self.status.state = state
        self._state_since = now

    def _finish(self, state, code, message, now):
        self._cancel_event.set()
        if self._future is not None:
            self._future.cancel()
        self._transition(state, now)
        self.status.error_code = code
        self.status.error_message = message
        self.status.navigation_stopped = True

    def cancel(self, now):
        if self.active:
            self._finish(State.CANCELED, 'CANCELED', 'Navigation revoked; adapter HOLD requested', now)

    def _valid_pose(self, now):
        if self.pose is None:
            return False
        p = self.pose
        if p.frame != self.route['frame'] or not all(math.isfinite(v) for v in
                (p.x, p.y, p.z, p.yaw, p.stamp)):
            self.pose = None
            raise NavError('INVALID_POSE', 'Localization must be finite in the configured route frame')
        if p.stamp > now + 0.1:
            self.pose = None
            raise NavError('INVALID_POSE', 'Localization timestamp is in the future')
        if now - p.stamp > self.seg['timeouts']['pose']:
            raise NavError('POSE_STALE', 'Localization timestamp exceeded its freshness limit')
        return True

    def _motion_fresh(self, now):
        return (self.motion is not None and -0.1 <= now - self.motion.stamp
                <= self.seg['timeouts']['status'])

    def _check_motion(self, now, require_ready):
        if not self._motion_fresh(now):
            raise NavError('MOTION_STALE', 'No fresh feedback for the current execution')
        m = self.motion
        if m.fault_code:
            raise NavError('MOTION_FAULT', f'{m.fault_code}: {m.fault_message}')
        required = set(self.seg.get('required_capabilities', []))
        for flag, capability in (('allow_turn_in_place', 'turn_in_place'),
                                 ('allow_reverse', 'reverse'),
                                 ('allow_stair_correction', 'stair_yaw_correction')):
            if self.seg['tracking'].get(flag, False):
                required.add(capability)
        if self.seg['kind'] == 'stair':
            required.add('stair_motion')
        missing = required - set(m.capabilities)
        if missing:
            raise NavError('CAPABILITY_MISSING', f'Missing motion capabilities: {sorted(missing)}')
        if not set(self.seg['allowed_modes']).issubset(set(m.supported_modes)):
            raise NavError('MODE_UNSUPPORTED', 'Adapter does not support all required segment modes')
        if require_ready and not m.ready:
            raise NavError('MOTION_NOT_READY', 'Motion readiness was lost during execution')

    def _apply_tracking(self, result):
        self.status.progress = result.progress
        self.status.lateral_error = result.lateral_error
        self.status.heading_error = result.heading_error
        self.status.geometry_reached = result.geometry_reached
        self.status.phase = result.phase
        # Capture the tracking heading in the map frame with the reference
        # position; a bare Waypoint would be displayed with a fixed yaw of zero.
        self._target = Pose(
            result.target.x, result.target.y, result.target.z,
            wrap_angle(self.pose.yaw + result.heading_error),
            self.pose.stamp, self.pose.frame)

    def tick(self, now, dt):
        vx = wz = 0.0
        valid = False
        if self.active:
            try:
                if not math.isfinite(now) or (self._last_tick is not None and now < self._last_tick):
                    raise NavError('CLOCK_JUMP', 'Navigation clock moved backwards or became invalid')
                if not math.isfinite(dt) or dt <= 0:
                    raise NavError('CLOCK_JUMP', 'Tracking dt must be positive and finite')
                self._last_tick = now
                for key in ('*', self.status.segment_id):
                    blocked = self._blocked.get(key)
                    if blocked and blocked[0]:
                        raise NavError('CHANNEL_BLOCKED', blocked[1] or 'External channel blocked')
                has_pose = self._valid_pose(now)
                if not self._obstacle_gate(now, has_pose):
                    pass
                elif self.status.state == State.CHECKING:
                    if not has_pose:
                        if now - self._started > self.seg['timeouts']['pose']:
                            raise NavError('POSE_MISSING', 'Localization never became available')
                    else:
                        self._plan_start = self.pose
                        self._future = self._pool.submit(
                            plan, self.route, self.seg, self.pose, self._cancel_event)
                        self.status.phase = 'planning'
                        self._transition(State.PLANNING, now)
                elif self.status.state == State.PLANNING:
                    if not has_pose:
                        raise NavError('POSE_MISSING', 'Localization unavailable')
                    if now - self._state_since > self.seg['planning']['timeout']:
                        raise NavError('PLANNING_TIMEOUT', 'Planner exceeded its deadline')
                    if self._future.done():
                        path = self._future.result()
                        tolerance = self.seg['planning'].get('start_tolerance_xy', 0.2)
                        if (math.hypot(self.pose.x - self._plan_start.x,
                                       self.pose.y - self._plan_start.y) > tolerance or
                                abs(self.pose.z - self._plan_start.z) >
                                self.seg['planning'].get('join_z', 0.15)):
                            raise NavError('START_MOVED', 'Robot moved outside the validated planning start')
                        self.path = path
                        self._path_stamp = now
                        self.status.path_stamp = now
                        self._tracker = PurePursuit(path, self.seg['tracking'], self.goal,
                                                    self.route.get('command_ttl', .2))
                        self.status.phase = 'waiting_ready'
                        self._transition(State.WAIT_READY, now)
                elif self.status.state == State.WAIT_READY:
                    if self._motion_fresh(now):
                        self._check_motion(now, False)
                        if self.motion.ready and self.motion.mode == self.seg['mode']:
                            self._execution_started = now
                            self._transition(State.EXECUTING, now)
                    if self.status.state == State.WAIT_READY and now - self._state_since > self.seg['timeouts']['ready']:
                        raise NavError('READY_TIMEOUT', 'Motion adapter did not confirm readiness and mode')
                elif self.status.state in (State.EXECUTING, State.ARRIVAL_CONFIRM):
                    self._check_motion(now, True)
                    if now - self._execution_started > self.seg['timeouts']['execution']:
                        raise NavError('EXECUTION_TIMEOUT', 'Segment execution deadline exceeded')
                    if self.status.state == State.EXECUTING and self.motion.mode != self.seg['mode']:
                        raise NavError('MODE_MISMATCH', 'Adapter changed movement mode during tracking')
                    result = self._tracker.update(self.pose, dt)
                    self._apply_tracking(result)
                    if result.geometry_reached:
                        if self.status.state == State.EXECUTING:
                            self._transition(State.ARRIVAL_CONFIRM, now)
                            self._arrival_started = now
                        self.status.phase = 'geometric_candidate'
                        a = self.goal['arrival']
                        confirmed = ((not a.get('require_exit', False) or self.motion.exit_confirmed)
                                     and (not a.get('require_stable', False) or self.motion.stable)
                                     and (not a.get('require_hold', True) or self.motion.hold_confirmed)
                                     and (not a.get('required_mode') or self.motion.mode == a['required_mode']))
                        if confirmed:
                            if self._arrival_since is None:
                                self._arrival_since = now
                            if now - self._arrival_since >= a['confirm_seconds']:
                                self._finish(State.SUCCEEDED, '', 'Geometry and configured adapter confirmations satisfied', now)
                        else:
                            self._arrival_since = None
                        if self.active and now - self._arrival_started > self.seg['timeouts']['arrival']:
                            raise NavError('ARRIVAL_CONFIRM_TIMEOUT', 'Geometric candidate lacks exit/stability/mode/hold confirmation')
                    else:
                        self._arrival_since = None
                        if self.status.state == State.ARRIVAL_CONFIRM:
                            self._transition(State.EXECUTING, now)
                        vx, wz = result.vx, result.wz
                        if (self.path.collision_grid is not None and
                                not self.path.collision_grid.motion_is_free(
                                    self.pose, vx, wz, self.route.get('command_ttl', .2))):
                            raise NavError('TRACKING_COLLISION',
                                           'Command sweep leaves inflated platform free space')
                        valid = True
            except NavError as exc:
                self._finish(State.FAILED, exc.code, exc.message, now)
                valid = False
            except Exception as exc:  # An adapter/planner bug also revokes all commands.
                self._finish(State.FAILED, 'INTERNAL_ERROR', f'{type(exc).__name__}: {exc}', now)
                valid = False
        if not self.active:
            valid = False
        self.status.navigation_stopped = not valid
        self.status.hold_confirmed = bool(self.status.execution_id and self._motion_fresh(now)
                                          and self.motion.hold_confirmed)
        mode = getattr(self, 'seg', {}).get('mode', '')
        if self.status.state in (State.ARRIVAL_CONFIRM, State.SUCCEEDED):
            mode = self.goal['arrival'].get('required_mode') or mode
        command = Command(stamp=now, valid_until=now + self.route.get('command_ttl', 0.2),
                          task_id=self.status.task_id, execution_id=self.status.execution_id,
                          segment_id=self.status.segment_id, phase=self.status.phase,
                          mode=mode, behavior='TRACK' if valid else 'HOLD',
                          vx=vx if valid else 0.0, wz=wz if valid else 0.0, valid=valid)
        return ExecutionOutput(replace(self.status), command, self.path, self._target)

    def close(self):
        self._cancel_event.set()
        if self._future is not None:
            self._future.cancel()
        if self._owns_pool:
            # The current queued job is canceled above; running planners observe
            # their execution's cancel event. Python 3.8 has no cancel_futures.
            self._pool.shutdown(wait=False)
