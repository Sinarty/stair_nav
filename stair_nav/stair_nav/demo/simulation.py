"""Kinematic software-only adapter; no stair contact or safe-hold dynamics.

Only accepted vx/wz commands move XY/yaw. Height is looked up from the explicitly
selected synthetic segment profile using that integrated XY; it is never driven
towards a task target. Pose and path share the configured navigation reference.
"""
import math
from pathlib import Path

import yaml

from ..core.config import segment_config
from ..core.models import Command, MotionFeedback, NavError, Pose
from ..core.planning import OccupancyGrid


class Simulator:
    def __init__(self, route, initial_pose=None):
        self.route = route
        self.settings = route.get('simulation', {})
        initial_pose = self.settings.get('initial_pose') if initial_pose is None else initial_pose
        if initial_pose is not None:
            if (not isinstance(initial_pose, (list, tuple)) or len(initial_pose) != 4
                    or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                               and math.isfinite(value) for value in initial_pose)):
                raise NavError('INVALID_INITIAL_POSE', 'simulation initial_pose must be finite [x, y, z, yaw]')
            xyz_yaw = list(map(float, initial_pose))
        else:
            initial = route['nodes'][self.settings.get('initial_node', 'A')]
            xyz_yaw = [*initial['position'], initial['yaw']]
        self.pose = Pose(*xyz_yaw, 0.0, route['frame'])
        self.command = None
        self._segments = {key: segment_config(route, key) for key in route['segments']}
        self.faults = dict(not_ready=False, localization_dropout=False,
                           execution_fault=False, missing_exit=False, unstable=False)
        self._retired = set()
        self._last_stamp = -math.inf
        self._stationary_since = None
        self._hold_since = None
        self._speed = 0.0
        self._turn = 0.0
        self._profile_s = 0.0
        self._profiles = {}
        self._height_grids = {}
        base = Path(route.get('_base_dir', '.'))
        for segment_id, segment in self._segments.items():
            if segment['backend'] == 'height_grid_astar':
                data = yaml.safe_load((base / segment['grid_file']).read_text())
                options = dict(segment.get('planning', {}), inflation_radius=0.)
                self._height_grids[segment_id] = OccupancyGrid.from_mapping(data, options)
        for segment_id, filename in self.settings.get('profiles', {}).items():
            data = yaml.safe_load((base / filename).read_text())
            points = [tuple(p['position']) for p in data['points']]
            cumulative = [0.0]
            for a, b in zip(points, points[1:]):
                cumulative.append(cumulative[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
            self._profiles[segment_id] = (points, cumulative)

    def accept(self, command: Command):
        """Fail closed on malformed input; ties prefer revocation over tracking.

        The adapter independently checks its motion limits. Its command cache is
        volatile; an accepted command expires even if the publisher disappears.
        """
        if not isinstance(command, Command):
            return False
        if (not command.task_id or not command.execution_id
                or command.segment_id not in self._segments):
            return False
        numbers = (command.stamp, command.valid_until, command.vx, command.wz)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in numbers):
            return False
        lifetime = command.valid_until-command.stamp
        if not 0 < lifetime <= self.route.get('command_ttl', .2)+1e-6:
            return False
        if command.behavior not in ('TRACK', 'HOLD') or command.valid != (command.behavior == 'TRACK'):
            return False
        segment = self._segments[command.segment_id]
        supported = self.settings.get('supported_modes', ['flat', 'stair'])
        if command.mode not in supported or command.mode not in segment['allowed_modes']:
            return False
        if command.valid:
            tracking = segment['tracking']
            capabilities = set(self.settings.get('capabilities', []))
            if not set(segment.get('required_capabilities', [])).issubset(capabilities):
                return False
            if abs(command.vx) > tracking['max_vx']+1e-6 or abs(command.wz) > tracking['max_wz']+1e-6:
                return False
            if command.vx < 0 and not tracking.get('allow_reverse', False):
                return False
            if abs(command.vx) < 1e-8 and abs(command.wz) > 1e-8 and not tracking.get('allow_turn_in_place', False):
                return False
            if command.mode == 'stair' and abs(command.wz) > 1e-8:
                if not tracking.get('allow_stair_correction', False) or 'stair_yaw_correction' not in capabilities:
                    return False
        elif abs(command.vx) > 1e-8 or abs(command.wz) > 1e-8:
            return False
        if command.stamp < self._last_stamp or command.execution_id in self._retired:
            return False
        old = self.command
        if old and command.stamp == self._last_stamp:
            if command.execution_id != old.execution_id:
                return False
            if not old.valid and command.valid:
                return False
            if old.valid and command.valid and command != old:
                return False
        if old and old.execution_id != command.execution_id:
            self._retired.add(old.execution_id)
            self._profile_s = 0.0
            self._hold_since = None
            self._stationary_since = None
        self.command = command
        self._last_stamp = command.stamp
        return True

    def _profile_height(self, segment_id):
        surface = self._height_grids.get(segment_id)
        if surface is not None:
            # HOLD must not pull an initially misplaced robot onto a floor.
            height = surface.height_at(self.pose.x, self.pose.y)
            if self._speed == 0 or height is None or abs(height-self.pose.z) > surface.max_step_height+1e-9:
                return self.pose.z
            return height
        profile = self._profiles.get(segment_id)
        if profile is None:
            # Flat motion cannot change floors, including while holding after
            # rejecting a goal on another platform.
            return self.pose.z
        points, cumulative = profile
        candidates = []
        for i, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length2 = dx*dx + dy*dy
            if length2 <= 1e-12:
                continue
            t = max(0.0, min(1.0, ((self.pose.x-a[0])*dx+(self.pose.y-a[1])*dy)/length2))
            progress = cumulative[i] + t*math.sqrt(length2)
            # History prevents choosing a different overlapping profile branch.
            if progress < self._profile_s - .2 or progress > self._profile_s + 1.0:
                continue
            px, py = a[0]+t*dx, a[1]+t*dy
            distance = math.hypot(self.pose.x-px, self.pose.y-py)
            candidates.append((distance, progress, a[2]+t*(b[2]-a[2])))
        if not candidates:
            return self.pose.z
        distance, progress, height = min(candidates)
        # Outside the synthetic stair corridor there is no invented terrain.
        if distance > .7:
            return self.pose.z
        self._profile_s = max(self._profile_s, progress)
        return height

    def step(self, dt, now):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('simulation dt must be positive and finite')
        c = self.command
        valid = bool(c and c.valid and c.behavior == 'TRACK' and c.stamp <= now
                     and now < c.valid_until and not self.faults['execution_fault']
                     and not self.faults['not_ready'])
        self._speed = c.vx if valid else 0.0
        self._turn = c.wz if valid else 0.0
        # Midpoint integration: actual accepted command, never pose-to-target interpolation.
        yaw_mid = self.pose.yaw + .5*self._turn*dt
        x = self.pose.x + self._speed*math.cos(yaw_mid)*dt
        y = self.pose.y + self._speed*math.sin(yaw_mid)*dt
        yaw = math.atan2(math.sin(self.pose.yaw+self._turn*dt), math.cos(self.pose.yaw+self._turn*dt))
        self.pose = Pose(x, y, self.pose.z, yaw, now, self.route['frame'])
        if c:
            self.pose = Pose(x, y, self._profile_height(c.segment_id), yaw, now, self.route['frame'])
        stationary = abs(self._speed) < .025 and abs(self._turn) < .04
        if stationary:
            if self._stationary_since is None:
                self._stationary_since = now
        else:
            self._stationary_since = None
        if c and not valid:
            if self._hold_since is None:
                self._hold_since = now
        else:
            self._hold_since = None
        return self.pose

    def motion_status(self, now):
        c = self.command
        execution_id, segment_id, mode = ('', '', 'flat') if c is None else (c.execution_id, c.segment_id, c.mode)
        stable = bool(self._stationary_since is not None
                      and now-self._stationary_since >= self.settings.get('stable_delay', .25)
                      and not self.faults['unstable'])
        hold = bool(self._hold_since is not None
                    and now-self._hold_since >= self.settings.get('hold_delay', .25))
        exit_confirmed = False
        profile = self._profiles.get(segment_id)
        if profile:
            points, cumulative = profile
            exit_confirmed = bool(cumulative[-1]-self._profile_s <= self.settings.get('exit_margin', .35)
                                  and abs(self.pose.z-points[-1][2]) <= .12)
        if segment_id in self._height_grids:
            segment = self._segments[segment_id]
            goal = self.route['nodes'][segment['end']]['position']
            exit_confirmed = bool(math.hypot(self.pose.x-goal[0], self.pose.y-goal[1]) <=
                                  self.settings.get('exit_margin', .35) and abs(self.pose.z-goal[2]) <= .12)
        exit_confirmed = exit_confirmed and not self.faults['missing_exit']
        fault_code = 'SIM_EXECUTION_FAULT' if self.faults['execution_fault'] else ''
        return MotionFeedback(
            stamp=now, execution_id=execution_id, segment_id=segment_id, mode=mode,
            ready=not self.faults['not_ready'] and not bool(fault_code),
            exit_confirmed=exit_confirmed, stable=stable, hold_confirmed=hold,
            fault_code=fault_code, fault_message='injected execution fault' if fault_code else '',
            capabilities=tuple(self.settings.get('capabilities', [])),
            supported_modes=tuple(self.settings.get('supported_modes', ['flat', 'stair'])))
