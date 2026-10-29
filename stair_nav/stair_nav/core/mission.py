"""Deterministic mission arbitration; no ROS, wall clock, threads, or motion code.

The adapter serializes start/cancel/tick/reset. Async completions are queued with
both IDs; tick is the only place that consumes them. Effects describe I/O without
performing it. Navigation success already includes the segment arrival contract.
"""
from collections import deque
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path
import math
import uuid

import yaml

from .config import segment_config
from .models import NavError


class MissionState(str, Enum):
    IDLE = 'IDLE'
    VALIDATING = 'VALIDATING'
    WAITING_READY = 'WAITING_READY'
    DISPATCHING = 'DISPATCHING'
    RUNNING = 'RUNNING'
    APPROACHING_START = 'APPROACHING_START'
    CANCELING = 'CANCELING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'
    FAULT_BLOCKED = 'FAULT_BLOCKED'


TERMINAL = {MissionState.IDLE, MissionState.SUCCEEDED, MissionState.FAILED,
            MissionState.CANCELED, MissionState.FAULT_BLOCKED}
DEFAULT_TIMEOUTS = dict(service=5., ready=5., acceptance=3., execution=60.,
                        confirmation=8., cancel_response=2., cancel_terminal=5.,
                        cleanup=5., feedback=5.)
DEFAULT_READINESS = dict(pose_max_age=.5, motion_max_age=.5,
                         navigation_max_age=.5, require_start_node=True)


def validate_mission(config, route):
    """Validate mission references; coordinates and motion modes stay in route YAML."""
    if not isinstance(config, dict):
        raise NavError('MISSION_CONFIG_INVALID', 'Mission must be a mapping')
    config = dict(config)
    if not isinstance(config.get('route_id'), str) or not config['route_id'].strip():
        raise NavError('MISSION_CONFIG_INVALID', 'Nonempty route_id required')
    if 'route_file' in config and (not isinstance(config['route_file'], str) or not config['route_file'].strip()):
        raise NavError('MISSION_CONFIG_INVALID', 'route_file must be a nonempty path string')
    sequence = config.get('sequence', route.get('sequence'))
    if not isinstance(sequence, list) or not sequence or not all(
            isinstance(sid, str) and sid in route['segments'] for sid in sequence):
        raise NavError('MISSION_CONFIG_INVALID', 'sequence must reference existing segments')
    previous = None
    for sid in sequence:
        seg = segment_config(route, sid)
        if previous is not None and previous != seg['start']:
            raise NavError('MISSION_CONFIG_INVALID', f'Disconnected mission at {sid}')
        previous = seg['end']
    config['sequence'] = list(sequence)
    approach = config.get('approach_segment')
    if 'approach_segment' in config:
        if (not isinstance(approach, str) or not approach or
                approach not in route['segments'] or approach in sequence):
            raise NavError('MISSION_CONFIG_INVALID',
                           'approach_segment must reference a separate existing segment')
        seg = segment_config(route, approach)
        first = segment_config(route, sequence[0])
        if (seg['kind'] != 'platform' or seg['backend'] != 'grid_astar' or
                seg['end'] != first['start']):
            raise NavError('MISSION_CONFIG_INVALID',
                           'approach_segment must use platform grid_astar and end at the mission start node')
    for key, defaults in (('timeouts', DEFAULT_TIMEOUTS), ('readiness', DEFAULT_READINESS)):
        values = config.get(key, {})
        if not isinstance(values, dict) or set(values) - set(defaults):
            raise NavError('MISSION_CONFIG_INVALID', f'Unknown or invalid {key}')
        config[key] = {**defaults, **values}
        for name, value in config[key].items():
            if name == 'require_start_node':
                if type(value) is not bool:
                    raise NavError('MISSION_CONFIG_INVALID', 'require_start_node must be boolean')
            elif (isinstance(value, bool) or not isinstance(value, (int, float)) or
                  not math.isfinite(value) or value <= 0):
                raise NavError('MISSION_CONFIG_INVALID', f'{key}.{name} must be finite and positive')
    per_segment = config.get('per_segment', {})
    mission_segments = set(sequence) | ({approach} if approach else set())
    if not isinstance(per_segment, dict) or set(per_segment) - mission_segments:
        raise NavError('MISSION_CONFIG_INVALID', 'per_segment references unknown mission segment')
    for sid, override in per_segment.items():
        if not isinstance(override, dict) or set(override) - {'timeouts'}:
            raise NavError('MISSION_CONFIG_INVALID', f'{sid}: only timeouts overrides supported')
        values = override.get('timeouts', {})
        if not isinstance(values, dict) or set(values) - set(DEFAULT_TIMEOUTS):
            raise NavError('MISSION_CONFIG_INVALID', f'{sid}: invalid timeout names')
        for name, value in values.items():
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or value <= 0):
                raise NavError('MISSION_CONFIG_INVALID', f'{sid}: invalid timeout {name}')
    return config


def load_mission(filename, route):
    try:
        config = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
        config = validate_mission(config, route)
        if config.get('route_file') and route.get('_source_file'):
            referenced = (Path(filename).resolve().parent / config['route_file']).resolve()
            if referenced != Path(route['_source_file']).resolve():
                raise NavError('MISSION_CONFIG_INVALID',
                               f'Mission route_file {referenced} differs from loaded route {route["_source_file"]}')
        return config
    except NavError:
        raise
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise NavError('MISSION_CONFIG_INVALID', str(exc)) from exc


@dataclass
class Readiness:
    ready: bool = False
    released: bool = False
    execution_id: str = ''
    stamp: float = 0.
    reason: str = ''
    server_instance: str = ''
    # Prerequisites were checked for this upcoming segment. execution_id above
    # identifies the previous execution whose control release was observed.
    target_segment_id: str = ''
    # None means fresh localization cannot determine arrival at the main start.
    at_start: bool | None = None


@dataclass
class MissionStatus:
    task_id: str = ''
    route_id: str = ''
    execution_id: str = ''
    segment_id: str = ''
    segment_index: int = 0  # main sequence is one based; approach/before start is zero
    segment_count: int = 0
    completed_segments: int = 0
    state: str = MissionState.IDLE
    phase: str = ''
    segment_state: str = ''
    segment_progress: float = 0.
    error_code: str = ''
    error_message: str = ''
    navigation_stopped: bool = True
    hold_confirmed: bool = False
    blocked: bool = False
    cancel_requested: bool = False
    cancel_sent: bool = False
    cancel_accepted: bool = False
    downstream_terminal: bool = False


@dataclass
class Effect:
    kind: str
    task_id: str
    execution_id: str
    segment_id: str
    data: dict = field(default_factory=dict)
    snapshot: MissionStatus | None = None


class MissionCore:
    def __init__(self, route, config, id_factory=None):
        self.route = route
        self.config = validate_mission(config, route)
        self.sequence = self.config['sequence']
        self._approach_segment = self.config.get('approach_segment')
        self._approach_pending = False
        self.status = MissionStatus(route_id=self.config['route_id'],
                                    segment_count=len(self.sequence))
        self._new_id = id_factory or (lambda: str(uuid.uuid4()))
        self._events = deque()
        self._effects = []
        self._terminal_emitted = False
        self._last_now = None
        self._accepted = None
        self._result_received = False
        self._cancel_at = None
        self._cancel_ack_at = None
        self._cleanup_at = None
        self._stop_at = None
        self._confirmation_at = None
        self._stop_target = None
        self._seen_tasks = set()

    @property
    def active(self):
        return self.status.state not in TERMINAL

    @property
    def busy(self):
        return self.active or self.status.blocked

    def _effect(self, kind, **data):
        s = self.status
        snapshot = replace(s) if kind in ('TRANSITION', 'SEGMENT_TRANSITION') else None
        self._effects.append(Effect(kind, s.task_id, s.execution_id, s.segment_id, data, snapshot))

    def drain_effects(self):
        effects, self._effects = self._effects, []
        return effects

    def _transition(self, state, now, event, reason='', code=''):
        old = self.status.state
        if old == state:
            return
        self.status.state = state
        self._state_at = now
        self._effect('TRANSITION', old=str(old.value if isinstance(old, Enum) else old),
                     new=str(state.value if isinstance(state, Enum) else state),
                     event=event, reason=reason, code=code, time=now)

    def _segment_transition(self, state, phase, now, event, code=None):
        s = self.status
        old_state, old_phase = s.segment_state, s.phase
        s.segment_state, s.phase = state, phase
        if (state, phase) != (old_state, old_phase):
            self._effect('SEGMENT_TRANSITION', old=old_state, new=state,
                         event=event,
                         reason=f'phase {old_phase} → {phase}' if old_phase != phase else '',
                         code=s.error_code if code is None else code, time=now)

    def _finish(self, state, now, event, code='', message=''):
        self.status.error_code = code
        self.status.error_message = message
        self.status.blocked = state == MissionState.FAULT_BLOCKED
        self._transition(state, now, event, message, code)
        if not self._terminal_emitted:
            self._terminal_emitted = True
            self._effect('TERMINAL', state=state.value)

    def start(self, route_id, task_id, now):
        if (self.busy or not task_id or task_id in self._seen_tasks or
                route_id != self.config['route_id'] or not math.isfinite(now)):
            return False
        self._seen_tasks.add(task_id)
        self.status = MissionStatus(task_id=task_id, route_id=route_id,
                                    segment_count=len(self.sequence))
        self._terminal_emitted = False
        self._last_now = now
        self._accepted = None
        self._result_received = False
        self._stop_target = None
        self._cancel_at = self._cancel_ack_at = self._cleanup_at = self._stop_at = None
        self._confirmation_at = None
        self._approach_pending = bool(self._approach_segment)
        self._select(-1 if self._approach_pending else 0, now)
        self._transition(MissionState.VALIDATING, now, 'START', 'Mission accepted')
        return True

    def _select(self, index, now, preserve_wait=False):
        s = self.status
        s.segment_index = index + 1
        # The optional approach uses live localization as its planning start;
        # it is outside the counted, static main route sequence.
        s.segment_id = self._approach_segment if index == -1 else self.sequence[index]
        s.execution_id = ''
        s.phase = s.segment_state = ''
        s.segment_progress = 0.
        s.cancel_sent = s.cancel_accepted = s.downstream_terminal = False
        self._accepted = None
        self._result_received = False
        self._confirmation_at = None
        if not preserve_wait:
            self._waiting_at = now
        self._timeouts = {**self.config['timeouts'], **self.config.get(
            'per_segment', {}).get(s.segment_id, {}).get('timeouts', {})}

    def post(self, kind, task_id, execution_id, **data):
        self._events.append((kind, task_id, execution_id, data))

    def cancel(self, now):
        if not self.active or self.status.cancel_requested:
            return False
        self.status.cancel_requested = True
        self._begin_stop('CANCELED', 'CANCELED', 'Task cancellation requested', now, 'CANCEL')
        return True

    def _begin_stop(self, target, code, message, now, event):
        if self._stop_target is None:
            self._stop_target = target
            self.status.error_code = code
            self.status.error_message = message
            self._stop_at = now
        if not self.status.execution_id:
            self.status.navigation_stopped = True
            self._finish(MissionState(self._stop_target), now, event,
                         self.status.error_code, self.status.error_message)
            return
        self._transition(MissionState.CANCELING, now, event, message, code)
        if self._accepted is True and not self._result_received:
            self._request_cancel(now)

    def _request_cancel(self, now):
        if self.status.cancel_sent:
            return
        self.status.cancel_sent = True
        self._cancel_at = now
        self._effect('CANCEL')

    def _fault(self, code, message, now, event='UNCERTAIN'):
        self.status.navigation_stopped = False if not self._result_received else self.status.navigation_stopped
        self.status.hold_confirmed = False
        self._finish(MissionState.FAULT_BLOCKED, now, event, code, message)

    def _event(self, kind, data, now):
        s = self.status
        if kind == 'ACCEPTED':
            if self._accepted is not None:
                return
            self._accepted = bool(data.get('accepted'))
            if not self._accepted:
                s.downstream_terminal = True
                self._result_received = True
                s.navigation_stopped = True
                if s.blocked:
                    return
                target = self._stop_target or 'FAILED'
                self._finish(MissionState(target), now, 'REJECTED',
                             s.error_code or 'GOAL_REJECTED',
                             s.error_message or 'Segment goal rejected')
            elif self._stop_target or s.blocked:
                self._request_cancel(now)
            else:
                self._run_at = self._last_feedback = now
                state = (MissionState.APPROACHING_START if s.segment_id == self._approach_segment
                         else MissionState.RUNNING)
                self._transition(state, now, 'ACCEPTED')
        elif kind == 'FEEDBACK':
            if s.state not in (MissionState.DISPATCHING, MissionState.RUNNING,
                               MissionState.APPROACHING_START, MissionState.CANCELING) or self._result_received:
                return
            sample = data.get('stamp', now)
            if (not isinstance(sample, (int, float)) or not math.isfinite(sample) or
                    not -.1 <= now - sample <= self._timeouts['feedback'] or
                    sample <= getattr(self, '_last_sample', -math.inf)):
                return
            self._last_sample = sample
            canceling = s.state == MissionState.CANCELING
            if not canceling:
                self._last_feedback = now
            progress = data.get('progress', 0.)
            if isinstance(progress, (int, float)) and math.isfinite(progress):
                s.segment_progress = min(1., max(0., progress))
            if data.get('error_code') and not canceling:
                s.error_code = data['error_code']
                s.error_message = data.get('error_message', '')
            self._segment_transition(data.get('segment_state', ''), data.get('phase', ''),
                                     now, 'SEGMENT_FEEDBACK', data.get('error_code') or s.error_code)
            confirming = s.segment_state == 'ARRIVAL_CONFIRM' or s.phase == 'geometric_candidate'
            if confirming and not canceling and self._confirmation_at is None:
                self._confirmation_at = now
            # Once arrival confirmation has started, leaving/re-entering cannot
            # reset its mission deadline by noisy geometry feedback.
        elif kind == 'RESULT':
            if self._result_received:
                return
            self._result_received = True
            s.downstream_terminal = True
            s.navigation_stopped = bool(data.get('navigation_stopped'))
            s.hold_confirmed = bool(data.get('hold_confirmed'))
            # Result and feedback travel independently. Record a terminal state
            # before selecting another segment even if its last feedback is late.
            segment_state = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'FAILED'}.get(data.get('status'))
            if segment_state is not None and s.segment_state != segment_state:
                self._segment_transition(segment_state, s.phase, now, 'SEGMENT_RESULT',
                                         data.get('error_code') or s.error_code)
            if s.blocked:
                return  # evidence is retained for explicit reset; fault is sticky
            if self._stop_target:
                self._cleanup_at = now
                return
            success = data.get('success') and data.get('status') == 4
            if success and s.navigation_stopped and s.hold_confirmed:
                s.segment_progress = 1.
                if s.segment_id == self._approach_segment:
                    self._select(0, now)
                    self._transition(MissionState.WAITING_READY, now, 'APPROACH_SUCCEEDED')
                elif s.completed_segments + 1 == len(self.sequence):
                    s.completed_segments += 1
                    self._finish(MissionState.SUCCEEDED, now, 'LAST_SEGMENT_SUCCEEDED')
                else:
                    s.completed_segments += 1
                    self._select(s.completed_segments, now)
                    self._transition(MissionState.WAITING_READY, now, 'SEGMENT_SUCCEEDED')
            else:
                code = data.get('error_code') or ('RESULT_CONTRACT' if success else 'SEGMENT_FAILED')
                message = data.get('error_message') or 'Segment result did not satisfy success/release contract'
                self._begin_stop('FAILED', code, message, now, 'SEGMENT_FAILED')
                self._cleanup_at = now
        elif kind == 'CANCEL_RESPONSE':
            if self._result_received:
                return  # result may win the cancel-response race
            if data.get('accepted'):
                s.cancel_accepted = True
                self._cancel_ack_at = now
            elif not s.blocked:
                self._fault('CANCEL_REJECTED', 'Downstream refused cancellation; execution unresolved', now)
        elif kind == 'ERROR':
            operation = data.get('operation', 'unknown')
            if s.blocked:
                return
            if operation == 'cancel' and not self._result_received:
                self._fault('CANCEL_TRANSPORT', data.get('message', ''), now)
            elif operation in ('send', 'result'):
                self._begin_stop('FAILED', 'ACTION_TRANSPORT', data.get('message', ''), now, 'TRANSPORT_ERROR')
        else:
            raise ValueError(f'Unknown mission event {kind}')

    def tick(self, now, readiness=None, server_ready=False):
        readiness = readiness or Readiness()
        if not math.isfinite(now):
            if self.active:
                self._begin_stop('FAILED', 'CLOCK_JUMP', 'Mission clock is invalid',
                                 self._last_now or 0., 'CLOCK_JUMP')
            return self.drain_effects()
        if self._last_now is not None and now < self._last_now:
            if self.active:
                self._begin_stop('FAILED', 'CLOCK_JUMP', 'Mission clock moved backwards',
                                 now, 'CLOCK_JUMP')
                # Rebase cleanup deadlines so a clock reset cannot defer cleanup
                # indefinitely. Fresh ROS observations still need new stamps.
                for name in ('_stop_at', '_cancel_at', '_cancel_ack_at', '_cleanup_at'):
                    if getattr(self, name) is not None:
                        setattr(self, name, now)
            self._last_sample = -math.inf
        self._last_now = now
        while self._events:
            kind, task_id, execution_id, data = self._events.popleft()
            current = (task_id == self.status.task_id and execution_id == self.status.execution_id
                       and bool(execution_id))
            if not current:
                if kind == 'ACCEPTED' and data.get('accepted'):
                    self._effects.append(Effect('CANCEL', task_id, execution_id, '', {}))
                continue
            if not self.active and not self.status.blocked:
                continue
            self._event(kind, data, now)
        s = self.status
        if s.state == MissionState.VALIDATING:
            # Constructor validated the route references before accepting any I/O.
            self._transition(MissionState.WAITING_READY, now, 'VALIDATED')
        if s.state == MissionState.WAITING_READY:
            target_changed = False
            start_known = readiness.at_start is True or readiness.at_start is False
            if self._approach_pending:
                # Localization may settle while capabilities, service or
                # release evidence are still missing. Keep the decision open
                # until the first goal is actually dispatched.
                if start_known:
                    index = 0 if readiness.at_start else -1
                    target = self.sequence[0] if index == 0 else self._approach_segment
                    if target != s.segment_id:
                        self._select(index, now, preserve_wait=True)
                        target_changed = True
            # A queued result may have selected another segment after the
            # adapter evaluated readiness. Only facts for this target can send.
            if not server_ready and now - self._waiting_at >= self._timeouts['service']:
                self._finish(MissionState.FAILED, now, 'TIMEOUT', 'SERVICE_TIMEOUT',
                             'Segment Action server unavailable')
            elif now - self._waiting_at >= self._timeouts['ready']:
                self._finish(MissionState.FAILED, now, 'TIMEOUT', 'READY_TIMEOUT',
                             readiness.reason or 'Readiness or downstream synchronization unavailable')
            elif (not target_changed and
                  (not self._approach_pending or start_known) and
                  server_ready and readiness.ready and readiness.released and
                  readiness.target_segment_id == s.segment_id):
                self._approach_pending = False
                s.execution_id = self._new_id()
                self._accepted = None
                self._result_received = False
                s.navigation_stopped = False
                s.hold_confirmed = False
                self._dispatch_at = now
                self._last_sample = -math.inf
                self._dispatch_server = readiness.server_instance
                self._transition(MissionState.DISPATCHING, now, 'READY')
                self._effect('SEND')
        elif s.state == MissionState.DISPATCHING:
            if now - self._dispatch_at >= self._timeouts['acceptance']:
                self._begin_stop('FAILED', 'GOAL_ACCEPT_TIMEOUT',
                                 'Acceptance unknown; cancel late accepted handle; no retry', now, 'TIMEOUT')
        elif s.state in (MissionState.RUNNING, MissionState.APPROACHING_START):
            if now - self._run_at >= self._timeouts['execution']:
                self._begin_stop('FAILED', 'EXECUTION_TIMEOUT', 'Segment execution exceeded mission deadline', now, 'TIMEOUT')
            elif (self._confirmation_at is not None and
                  now - self._confirmation_at >= self._timeouts['confirmation']):
                self._begin_stop('FAILED', 'CONFIRMATION_TIMEOUT', 'Segment arrival confirmation exceeded deadline', now, 'TIMEOUT')
            elif now - self._last_feedback >= self._timeouts['feedback']:
                self._begin_stop('FAILED', 'FEEDBACK_TIMEOUT', 'No fresh segment feedback', now, 'TIMEOUT')
        if s.state == MissionState.CANCELING:
            if self._result_received:
                if self._accepted is False:
                    self._finish(MissionState(self._stop_target), now, 'REJECTED', s.error_code, s.error_message)
                elif (s.navigation_stopped and readiness.released and
                      readiness.execution_id == s.execution_id and
                      math.isfinite(readiness.stamp) and
                      self._cleanup_at is not None and
                      self._cleanup_at <= readiness.stamp <= now + .1 and
                      now-readiness.stamp <= self.config['readiness']['motion_max_age']):
                    s.hold_confirmed = True
                    self._finish(MissionState(self._stop_target), now, 'RELEASE_CONFIRMED',
                                 s.error_code, s.error_message)
                elif self._cleanup_at is not None and now - self._cleanup_at >= self._timeouts['cleanup']:
                    self._fault('CLEANUP_TIMEOUT', 'Navigation terminal but fresh adapter hold/stability/release missing', now)
            elif self._cancel_at is not None and not s.cancel_accepted and now - self._cancel_at >= self._timeouts['cancel_response']:
                self._fault('CANCEL_RESPONSE_TIMEOUT', 'Cancellation response unknown', now)
            elif self._cancel_ack_at is not None and now - self._cancel_ack_at >= self._timeouts['cancel_terminal']:
                self._fault('CANCEL_TERMINAL_TIMEOUT', 'Cancellation acknowledged but terminal result missing', now)
            elif self._accepted is None and now - self._stop_at >= self._timeouts['cancel_terminal']:
                self._fault('ACCEPTANCE_UNRESOLVED', 'Goal acceptance still unknown; late handle will be canceled', now)
        return self.drain_effects()

    def reset(self, now, readiness):
        if self.active:
            return False, 'BUSY: mission is active'
        if not self.status.blocked:
            return False, 'No blocked fault to reset'
        if self._accepted is None or (self._accepted and not self._result_received):
            return False, 'Original goal acceptance/terminal unresolved; restore transport or reconcile/restart execution chain'
        if not readiness.ready or not readiness.released or not readiness.server_instance:
            return False, readiness.reason or 'Fresh idle server and adapter release required'
        if (not math.isfinite(readiness.stamp) or not 0 <= now-readiness.stamp <=
                self.config['readiness']['motion_max_age']):
            return False, 'Release evidence is stale'
        self.status.blocked = False
        self._transition(MissionState.IDLE, now, 'RESET', 'Explicit reset with resolved goal and fresh release')
        return True, 'Fault cleared; next mission starts from the configured initial node'
