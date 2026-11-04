"""ROS-free M20 velocity lease, native readiness and gait-switch arbitration.

Outputs are requests for the ROS/SDK wrapper. No posture, power or autonomous
planner command is inferred here. Observations retain their acquisition time.
"""
from dataclasses import dataclass
import math
from typing import Optional

from .models import Command, MotionFeedback


@dataclass(frozen=True)
class M20Config:
    dry_run: bool = True
    navigation_mode_confirmed: bool = False
    planner_stopped: bool = False
    normal_location_status: int = -1
    require_location_status: bool = True
    flat_gait: int = 12290
    stair_gait: int = 12291
    supported_modes: tuple = ('flat',)
    capabilities: tuple = ()
    motion_timeout: float = .5
    imu_timeout: float = .5
    location_timeout: float = 1.5
    safety_timeout: float = 3.
    command_max_ttl: float = .5
    max_vx: float = .3
    max_wz: float = .6
    stable_seconds: float = .35
    linear_stable_threshold: float = .03
    yaw_stable_threshold: float = .05
    gyro_stable_threshold: float = .08
    confirmation_timeout: float = 1.
    gait_timeout: float = 3.
    flat_min_vx: float = .15
    stair_min_vx: float = .15
    flat_min_wz: float = .35
    stair_min_wz: float = .4

    def __post_init__(self):
        for name in ('dry_run', 'navigation_mode_confirmed', 'planner_stopped',
                     'require_location_status'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')
        for name in ('motion_timeout', 'imu_timeout', 'location_timeout', 'safety_timeout',
                     'command_max_ttl', 'max_vx', 'max_wz', 'stable_seconds',
                     'linear_stable_threshold', 'yaw_stable_threshold', 'gyro_stable_threshold',
                     'confirmation_timeout', 'gait_timeout'):
            if not _finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('flat_min_vx', 'stair_min_vx', 'flat_min_wz', 'stair_min_wz'):
            if not _finite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if (not self.supported_modes or set(self.supported_modes) - {'flat', 'stair'} or
                self.flat_gait == self.stair_gait):
            raise ValueError('Distinct gait IDs and supported flat/stair modes are required')
        if type(self.normal_location_status) is not int or self.normal_location_status not in (-1, 0, 1):
            raise ValueError('normal_location_status must be -1 (unconfirmed), 0 or 1')


@dataclass(frozen=True)
class MotionObservation:
    stamp: float
    state: int
    gait: int
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class M20Output:
    feedback: MotionFeedback
    vx: float
    wz: float
    publish_velocity: bool
    gait_request: Optional[int]
    diagnostics: dict


def _finite(*values):
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values)


class M20MotionAdapter:
    def __init__(self, config, frame='map'):
        self.config, self.frame = config, frame
        self.enabled = False
        self._motion = self._imu = self._location = None
        self._estop = self._charge = None
        self._command = self._confirmation = None
        self._execution_id = self._segment_id = self._task_id = ''
        self._retired = set()
        self._last_stamp = -math.inf
        self._last_now = self._last_steady = None
        self._lease_end = -math.inf
        self._stable_since = None
        self._stable = self._stair_seen = False
        self._hold_stamp = -math.inf
        self._pending_gait = None
        self._gait_ack_stamp = -math.inf
        self._fault_stop_pending = False
        self._owns_output = self._stopping = False
        self._enabled_at, self._last_nonzero = math.inf, -math.inf
        self._hold_authorized = False
        self._fault_code = self._fault_message = self._last_rejection = ''

    def observe_motion(self, observation):
        old = self._motion
        if (isinstance(observation, MotionObservation) and _finite(observation.stamp) and
                old is not None and observation.stamp <= old.stamp):
            return
        if not isinstance(observation, MotionObservation) or not _finite(
                observation.stamp, observation.state, observation.gait,
                observation.vx, observation.vy, observation.wz):
            self._motion, self._stable_since = None, None
            self._native_fault_edge('MOTION_STALE')
            return
        if (old is None or observation.stamp-old.stamp > self.config.motion_timeout or
                observation.state != 17 or
                math.hypot(observation.vx, observation.vy) > self.config.linear_stable_threshold or
                abs(observation.wz) > self.config.yaw_stable_threshold):
            self._stable_since = None
        self._motion = observation
        if observation.state != 17:
            self._native_fault_edge('MOTION_STATE')
        elif (self._pending_gait is None and observation.gait not in
              (self.config.flat_gait, self.config.stair_gait, 4097, 4099)):
            self._native_fault_edge('GAIT_UNSUPPORTED')

    def observe_imu(self, stamp, gyro_norm):
        if (_finite(stamp) and self._imu is not None and stamp <= self._imu[0]):
            return
        if not _finite(stamp, gyro_norm) or gyro_norm < 0:
            self._imu, self._stable_since = None, None
            self._native_fault_edge('IMU_STALE')
            return
        if (self._imu is None or stamp-self._imu[0] > self.config.imu_timeout or
                gyro_norm > self.config.gyro_stable_threshold):
            self._stable_since = None
        self._imu = (stamp, gyro_norm)

    def observe_location(self, stamp, status):
        if not self.config.require_location_status:
            return
        if (_finite(stamp) and self._location is not None and stamp <= self._location[0]):
            return
        if not _finite(stamp, status):
            self._location = None
            self._native_fault_edge('LOCATION_STALE')
        else:
            self._location = (stamp, status)
            if status != self.config.normal_location_status:
                self._native_fault_edge('LOCATION_ABNORMAL')

    def observe_estop(self, received_steady, active):
        if (_finite(received_steady) and self._estop is not None and
                received_steady < self._estop[0]):
            return
        if not _finite(received_steady) or type(active) is not bool:
            self._estop = None
            self._native_fault_edge('ESTOP_STALE')
        else:
            self._estop = (received_steady, active)
            if active:
                self._native_fault_edge('ESTOP_ACTIVE')

    def observe_charge(self, received_steady, state, error_code=0):
        if (_finite(received_steady) and self._charge is not None and
                received_steady < self._charge[0]):
            return
        if not _finite(received_steady, state, error_code):
            self._charge = None
            self._native_fault_edge('CHARGE_STALE')
        else:
            self._charge = (received_steady, state, error_code)
            if state != 0 or error_code != 0:
                self._native_fault_edge('CHARGING_OR_ERROR')

    def confirm(self, feedback, now=None):
        observed_now = self._last_now if now is None else now
        if (not isinstance(feedback, MotionFeedback) or not self._execution_id or
                feedback.execution_id != self._execution_id or
                feedback.segment_id != self._segment_id or observed_now is None or
                not self._fresh(feedback.stamp, observed_now, self.config.confirmation_timeout) or
                feedback.stamp < self._hold_stamp or
                (self._confirmation is not None and feedback.stamp <= self._confirmation.stamp)):
            return
        self._confirmation = feedback

    @staticmethod
    def _fresh(stamp, now, timeout):
        return _finite(stamp, now) and 0 <= now-stamp <= timeout

    def _fault(self, code, message):
        self._fault_code, self._fault_message = code, message

    def _native_fault_edge(self, code):
        """Retain an unsafe observation even if a healthy callback follows it.

        Observation callbacks do not have a current ROS/steady clock pair.
        Revoke immediately, then establish the stop-observation boundary at the
        next tick instead of mistaking the previous tick time for receipt time.
        """
        if not self.enabled:
            return
        self._fault(code, 'Native readiness lost; explicit enable and new HOLD required')
        self.enabled, self._stopping = False, self._owns_output
        self._lease_end, self._hold_authorized = -math.inf, False
        self._confirmation = None
        self._stable_since, self._stable = None, False
        self._fault_stop_pending = True

    def _clock(self, now, steady):
        if (not _finite(now, steady) or
                (self._last_now is not None and now < self._last_now) or
                (self._last_steady is not None and steady < self._last_steady)):
            self._fault('CLOCK_JUMP', 'Clock invalid or moved backwards; explicit re-enable required')
            self.enabled, self._stopping = False, self._owns_output
            self._hold_authorized = False
            self._lease_end, self._stable_since = -math.inf, None
            self._hold_stamp, self._confirmation = now, None
            if _finite(now, steady):
                self._last_now, self._last_steady = now, steady
            return False
        self._last_now, self._last_steady = now, steady
        return True

    def _mode(self):
        if self._motion is None:
            return ''
        return {self.config.flat_gait: 'flat', self.config.stair_gait: 'stair'}.get(
            self._motion.gait, '')

    def _on_stair(self):
        return bool(self._motion and self._motion.gait in (self.config.stair_gait, 4099))

    def _native_reason(self, now, steady):
        c, m = self.config, self._motion
        if m is None or not self._fresh(m.stamp, now, c.motion_timeout):
            return 'MOTION_STALE'
        if m.state != 17:
            return 'MOTION_STATE'
        if self._imu is None or not self._fresh(self._imu[0], now, c.imu_timeout):
            return 'IMU_STALE'
        if c.require_location_status:
            if c.normal_location_status == -1:
                return 'LOCATION_STATUS_UNCONFIRMED'
            if self._location is None or not self._fresh(self._location[0], now, c.location_timeout):
                return 'LOCATION_STALE'
            if self._location[1] != c.normal_location_status:
                return 'LOCATION_ABNORMAL'
        if self._estop is None or not self._fresh(self._estop[0], steady, c.safety_timeout):
            return 'ESTOP_STALE'
        if self._estop[1]:
            return 'ESTOP_ACTIVE'
        if self._charge is None or not self._fresh(self._charge[0], steady, c.safety_timeout):
            return 'CHARGE_STALE'
        if self._charge[1] != 0 or self._charge[2] != 0:
            return 'CHARGING_OR_ERROR'
        if (self._mode() not in ('flat', 'stair') and m.gait not in (4097, 4099) and
                self._pending_gait is None):
            return 'GAIT_UNSUPPORTED'
        return ''

    def _update_stable(self, now, steady):
        c, m = self.config, self._motion
        if self._pending_gait is not None or self._fault_stop_pending:
            self._stable_since, self._stable = None, False
            return
        quiet = (m is not None and self._fresh(m.stamp, now, c.motion_timeout) and
                 m.state == 17 and math.hypot(m.vx, m.vy) <= c.linear_stable_threshold and
                 abs(m.wz) <= c.yaw_stable_threshold and self._imu is not None and
                 self._fresh(self._imu[0], now, c.imu_timeout) and
                 self._imu[1] <= c.gyro_stable_threshold)
        if not quiet:
            self._stable_since, self._stable = None, False
            return
        sample = min(m.stamp, self._imu[0])
        boundary = max(self._last_nonzero, self._gait_ack_stamp,
                       self._hold_stamp if self._hold_stamp != math.inf else -math.inf)
        if sample <= boundary:
            self._stable_since, self._stable = None, False
            return
        if self._stable_since is None:
            self._stable_since = (sample, steady)
        self._stable = (sample-self._stable_since[0] >= c.stable_seconds and
                        steady-self._stable_since[1] >= c.stable_seconds)

    def _external_hold(self, now, require_exit=False):
        f = self._confirmation
        return bool(f and self._fresh(f.stamp, now, self.config.confirmation_timeout) and
                    f.stamp >= self._hold_stamp and f.stable and f.hold_confirmed and
                    (not require_exit or f.exit_confirmed) and not f.fault_code)

    def _live(self, now, steady):
        return bool(self._command and self._command.stamp <= now < self._command.valid_until and
                    steady < self._lease_end)

    def _holding(self, now, steady):
        return self._command is not None and (self._command.behavior == 'HOLD' or
                                              not self._live(now, steady))

    def enable(self, enabled, now, steady):
        if not self._clock(now, steady):
            return False, self._fault_message
        self._update_stable(now, steady)
        if not enabled:
            self.enabled = False
            self._hold_authorized = False
            self._stopping = self._owns_output
            self._lease_end = -math.inf
            if self._owns_output:
                self._hold_stamp, self._confirmation = now, None
                self._stable_since, self._stable = None, False
            return True, 'Stopping until native stability is confirmed' if self._stopping else 'Disabled'
        c = self.config
        reason = ('DRY_RUN' if c.dry_run else
                  'NAVIGATION_MODE_UNCONFIRMED' if not c.navigation_mode_confirmed else
                  'PLANNER_NOT_STOPPED' if not c.planner_stopped else
                  self._native_reason(now, steady))
        if reason or not self._stable or self._pending_gait is not None:
            return False, reason or ('GAIT_SWITCH_PENDING' if self._pending_gait else 'NOT_STABLE')
        self.enabled, self._stopping = True, False
        self._enabled_at, self._hold_authorized = now, False
        # Enabling never revives a velocity accepted before operator authorization.
        self._lease_end = -math.inf
        self._confirmation = None
        self._fault_code = self._fault_message = ''
        return True, 'Enabled; fresh navigation commands required'

    def accept(self, command, now, steady):
        if not self._clock(now, steady):
            return False
        self._update_stable(now, steady)
        def reject(reason):
            self._last_rejection = reason
            return False
        if not isinstance(command, Command) or not _finite(
                command.stamp, command.valid_until, command.vx, command.wz):
            return reject('MALFORMED_COMMAND')
        if not all(isinstance(value, str) and value for value in
                   (command.task_id, command.execution_id, command.segment_id)):
            return reject('MISSING_ID')
        ttl = command.valid_until-command.stamp
        if (command.stamp > now or command.valid_until <= now or
                not 0 < ttl <= self.config.command_max_ttl or
                command.behavior not in ('TRACK', 'HOLD') or
                type(command.valid) is not bool or
                command.valid != (command.behavior == 'TRACK') or
                (command.behavior == 'HOLD' and (command.vx != 0 or command.wz != 0))):
            return reject('INVALID_LEASE_OR_BEHAVIOR')
        if command.mode not in self.config.supported_modes:
            return reject('MODE_UNSUPPORTED')
        if command.behavior == 'TRACK' and (command.stamp < self._enabled_at or
                                           not self._hold_authorized):
            return reject('PRE_ENABLE_COMMAND_OR_INITIAL_HOLD_REQUIRED')
        if command.execution_id in self._retired or command.stamp < self._last_stamp:
            return reject('RETIRED_OR_OUT_OF_ORDER')
        same = command.execution_id == self._execution_id
        if not same and command.behavior == 'TRACK':
            return reject('INITIAL_HOLD_REQUIRED')
        if same and (command.task_id != self._task_id or command.segment_id != self._segment_id):
            return reject('IDENTITY_MISMATCH')
        if command.stamp == self._last_stamp:
            if not same or self._command is None or self._command.behavior == 'HOLD':
                return reject('DUPLICATE_OR_REVOKED')
            if command.behavior != 'HOLD':
                return reject('DUPLICATE_STAMP')
        if abs(command.vx) > self.config.max_vx or abs(command.wz) > self.config.max_wz:
            self._fault('COMMAND_LIMIT', 'Requested velocity exceeds configured independent limits')
            return reject('COMMAND_LIMIT')
        if not same and self._execution_id:
            if not (self._pending_gait is None and self._holding(now, steady) and self._stable and
                    (not self._stair_seen or self._external_hold(now))):
                return reject('PREVIOUS_EXECUTION_NOT_RELEASED')
            self._retired.add(self._execution_id)
        if not same:
            self._hold_authorized = False
            self._execution_id, self._segment_id, self._task_id = (
                command.execution_id, command.segment_id, command.task_id)
            self._confirmation = None
            self._stair_seen = self._on_stair()
        if command.behavior == 'HOLD' and (self._command is None or not same or
                                           self._command.behavior != 'HOLD'):
            self._hold_stamp = command.stamp
            self._confirmation = None
            self._stable_since, self._stable = None, False
        elif command.behavior == 'TRACK':
            # Exit evidence is meaningful only after this execution requests hold.
            self._hold_stamp, self._confirmation = math.inf, None
        self._command = command
        self._last_stamp = command.stamp
        self._lease_end = steady + (command.valid_until-now)
        if command.behavior == 'HOLD' and self.enabled and command.stamp >= self._enabled_at:
            self._hold_authorized = True
        self._last_rejection = ''
        return True

    def _can_stop_physically(self, now, steady):
        m, c = self._motion, self.config
        return bool(not c.dry_run and m and self._fresh(m.stamp, now, c.motion_timeout) and
                    m.state == 17 and self._estop and
                    self._fresh(self._estop[0], steady, c.safety_timeout) and not self._estop[1] and
                    self._charge and self._fresh(self._charge[0], steady, c.safety_timeout) and
                    self._charge[1:] == (0, 0))

    def tick(self, now, steady):
        clock_ok = self._clock(now, steady)
        c, m = self.config, self._motion
        mode = self._mode()
        if self._execution_id and self._on_stair():
            self._stair_seen = True
        if self._pending_gait:
            gait, sent, observation_stamp = self._pending_gait
            if _finite(steady) and steady-sent > c.gait_timeout:
                self._fault('GAIT_TIMEOUT', 'Native motion did not confirm the requested gait')
            elif (self._fault_code != 'GAIT_TIMEOUT' and m and m.state == 17 and
                    self._fresh(m.stamp, now, c.motion_timeout) and
                    m.gait == gait and m.stamp >= observation_stamp):
                self._pending_gait = None
                self._gait_ack_stamp = now
                self._stable_since, self._stable = None, False
        reason = self._native_reason(now, steady)
        if self.enabled and reason:
            self._native_fault_edge(reason)
        if self._fault_stop_pending and _finite(now):
            self._hold_stamp, self._confirmation = now, None
            self._stable_since, self._stable = None, False
            self._fault_stop_pending = False
        if clock_ok:
            self._update_stable(now, steady)
        else:
            self._stable = False
        live = self._live(now, steady)
        holding = self._holding(now, steady)
        if holding and self._hold_stamp == math.inf:
            self._hold_stamp, self._confirmation = now, None
            self._stable_since, self._stable = None, False
        expired = bool(self._command and not live)
        blocked = self._fault_code or ('DRY_RUN' if c.dry_run else '')
        blocked = blocked or ('DISABLED' if not self.enabled else reason)
        blocked = blocked or ('COMMAND_EXPIRED' if expired else '')
        external_hold = self._external_hold(now)
        hold = bool(self._pending_gait is None and holding and self._stable and m and m.state == 17 and
                    (not self._stair_seen or external_hold))
        vx = wz = 0.
        gait_request = None
        command = self._command
        if not blocked and command and live:
            target_gait = c.flat_gait if command.mode == 'flat' else c.stair_gait
            if (command.behavior == 'HOLD' and self._stable and self._pending_gait is None and
                    m.gait != target_gait and (not self._stair_seen or
                    self._external_hold(now, require_exit=command.mode == 'flat'))):
                gait_request = target_gait
                self._pending_gait = (target_gait, steady, now)
                self._owns_output = True
                self._stable_since, self._stable = None, False
            if command.behavior == 'TRACK' and mode == command.mode and self._pending_gait is None:
                vx, wz = command.vx, command.wz
        if self.enabled and command and not blocked:
            self._owns_output = True
        publish = bool(self._owns_output and self._can_stop_physically(now, steady))
        if blocked or self._stopping or not live:
            vx = wz = 0.
        if publish and (vx != 0 or wz != 0):
            self._last_nonzero = now
            self._stable_since, self._stable = None, False
            self._confirmation = None
        if self._stopping and self._pending_gait is None and self._stable:
            self._stopping = self._owns_output = False
            publish = False
        hold = hold and self._stable
        minima = (c.stair_min_vx, c.stair_min_wz) if mode == 'stair' else (c.flat_min_vx, c.flat_min_wz)
        deadzone = {'vx': 0 < abs(vx) < minima[0], 'wz': 0 < abs(wz) < minima[1]}
        evidence_stamp = min(m.stamp, self._imu[0]) if m and self._imu else 0.
        if hold and self._stair_seen and self._confirmation:
            evidence_stamp = min(evidence_stamp, self._confirmation.stamp)
        feedback = MotionFeedback(
            stamp=evidence_stamp, execution_id=self._execution_id,
            segment_id=self._segment_id, mode=mode, ready=not bool(blocked),
            stable=self._stable, hold_confirmed=hold,
            exit_confirmed=bool(self._pending_gait is None and self._stair_seen and
                                self._external_hold(now, require_exit=True)
                                and self._stable),
            fault_code=blocked, fault_message=self._fault_message if self._fault_code else blocked,
            capabilities=tuple(c.capabilities), supported_modes=tuple(c.supported_modes))
        return M20Output(feedback, vx, wz, publish, gait_request, dict(
            enabled=self.enabled, stopping=self._stopping, dry_run=c.dry_run,
            require_location_status=c.require_location_status,
            command_active=bool(live), command_expired=bool(command and not live),
            native_gate=reason, gait_pending=self._pending_gait is not None,
            last_rejection=self._last_rejection, velocity_deadzone=deadzone))
