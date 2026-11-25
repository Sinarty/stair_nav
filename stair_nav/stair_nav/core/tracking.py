"""Pure Pursuit on an annotated 3-D path, producing horizontal vx and yaw rate."""
from collections import deque
from dataclasses import dataclass
import math

from .models import NavError, Waypoint, wrap_angle


@dataclass(frozen=True)
class TrackingOutput:
    vx: float
    wz: float
    progress: float
    lateral_error: float
    heading_error: float
    target: Waypoint
    phase: str
    geometry_reached: bool


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


class PurePursuit:
    def __init__(self, path, tracking, goal_node, command_ttl=.2):
        self.path, self.config, self.goal = path, tracking, goal_node
        self.distance, self.progress = 0.0, 0.0
        self.vx, self.wz = 0.0, 0.0
        self.last_pose = None
        self.command_ttl = command_ttl
        self.cornering = tracking.get('cornering')
        self._corners = deque()
        self._corner_phase = ''
        self._zero_observed_at = None
        if self.cornering:
            for i, (a, b, c) in enumerate(zip(path.points, path.points[1:], path.points[2:]), 1):
                incoming = math.atan2(b.y-a.y, b.x-a.x)
                outgoing = math.atan2(c.y-b.y, c.x-b.x)
                if abs(wrap_angle(outgoing-incoming)) >= self.cornering['min_turn_angle']:
                    self._corners.append((path.cumulative[i], outgoing))

    def _confirmed_zero(self, motion):
        if self.vx or self.wz or motion is None:
            return False
        if self._zero_observed_at is None:
            self._zero_observed_at = motion.stamp
            return False
        return motion.stable and motion.stamp > self._zero_observed_at

    def _corner_control(self, pose, motion):
        """Brake, turn toward the outgoing edge, then wait for measured stability."""
        _, outgoing = self._corners[0]
        heading = wrap_angle(outgoing-pose.yaw)
        aligned = abs(heading) <= self.cornering['resume_angle']
        if not self._corner_phase:
            self._corner_phase = 'corner_braking'
        if self._corner_phase == 'corner_braking' and self._confirmed_zero(motion):
            self._corner_phase = 'corner_turning'
        if self._corner_phase == 'corner_turning' and aligned:
            self._corner_phase = 'corner_settling'
            self._zero_observed_at = None
        if self._corner_phase == 'corner_settling' and self._confirmed_zero(motion):
            if aligned:
                self._corners.popleft()
                self._corner_phase = ''
                self._zero_observed_at = None
                return None
            self._corner_phase = 'corner_turning'
        return heading, 1.5*heading if self._corner_phase == 'corner_turning' else 0.

    def _visible_target(self, pose, distance):
        grid = self.path.collision_grid
        target = self.path.sample(distance)
        if grid is None:
            return target
        if not grid.line_is_free((pose.x, pose.y), (pose.x, pose.y)):
            raise NavError('TRACKING_COLLISION', 'Live position left the inflated platform free space')
        # A lookahead beyond a bend can lie across an obstacle even when the
        # reference polyline is valid. Retreat along that polyline until visible.
        while distance > self.distance:
            if grid.line_is_free((pose.x, pose.y), (target.x, target.y)):
                return target
            distance = max(self.distance, distance-grid.resolution*.25)
            target = self.path.sample(distance)
        if not grid.line_is_free((pose.x, pose.y), (target.x, target.y)):
            raise NavError('TRACKING_COLLISION', 'No visible reference point in platform free space')
        return target

    def _match(self, pose):
        if len(self.path.points) == 1:
            point = self.path.points[0]
            return 0.0, point, (1.0, 0.0)
        back = self.config.get('match_backtrack', .2)
        forward = self.config.get('match_forward', 1.5)
        if self.last_pose is not None:
            motion = math.dist((pose.x, pose.y, pose.z),
                               (self.last_pose.x, self.last_pose.y, self.last_pose.z))
            forward = min(forward, 2*motion+.25)
        lower, upper = max(0, self.distance-back), min(self.path.length, self.distance+forward)
        best = None
        for i, (a, b) in enumerate(zip(self.path.points, self.path.points[1:])):
            s0, s1 = self.path.cumulative[i:i+2]
            if s1 < lower or s0 > upper or s1-s0 < 1e-9:
                continue
            dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
            t = ((pose.x-a.x)*dx+(pose.y-a.y)*dy+(pose.z-a.z)*dz)/(dx*dx+dy*dy+dz*dz)
            t = _clip(t, max(0, (lower-s0)/(s1-s0)), min(1, (upper-s0)/(s1-s0)))
            s = s0+t*(s1-s0)
            point = Waypoint(a.x+t*dx, a.y+t*dy, a.z+t*dz, a.phase, a.mode)
            score = (pose.x-point.x)**2+(pose.y-point.y)**2+4*(pose.z-point.z)**2
            # Break equal-distance overlapping projections by history, then earlier progress.
            score += 1e-6*abs(s-self.distance)
            if best is None or score < best[0]:
                best = score, s, point, (dx, dy)
        if best is None:
            raise NavError('TRACKING_UNREACHABLE', 'No path edge within the active progress window')
        return best[1:]

    def update(self, pose, dt, motion=None):
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.yaw, dt)) or dt <= 0:
            raise NavError('INVALID_POSE', 'Tracking needs a finite pose and positive timestep')
        s, matched, tangent = self._match(pose)
        planar_error = math.hypot(pose.x-matched.x, pose.y-matched.y)
        height_error = abs(pose.z-matched.z)
        if height_error > self.config.get('max_height_error', .3):
            raise NavError('HEIGHT_DEVIATION', f'Height departure {height_error:.3f} m')
        if planar_error > self.config.get('max_cross_track', .6):
            raise NavError('PATH_DEVIATION', f'Path departure {planar_error:.3f} m')
        cross = tangent[0]*(pose.y-matched.y)-tangent[1]*(pose.x-matched.x)
        lateral = math.copysign(planar_error, cross) if cross else planar_error
        self.distance = max(self.distance, s)
        self.progress = self.distance/self.path.length if self.path.length > 1e-9 else 1.0
        self.last_pose = pose
        lookahead = min(self.path.length, self.distance+self.config.get('lookahead', .6))
        goal_position, arrival = self.goal['position'], self.goal.get('arrival', {})
        goal_xy = math.hypot(pose.x-goal_position[0], pose.y-goal_position[1])
        goal_z = abs(pose.z-goal_position[2])
        goal_yaw = wrap_angle(self.goal.get('yaw', 0)-pose.yaw)
        position_reached = (goal_xy <= arrival.get('xy', .15) and goal_z <= arrival.get('z', .15)
                            and self.progress >= arrival.get('progress', .97))
        geometry_reached = position_reached and abs(goal_yaw) <= arrival.get('yaw', .15)
        corner_command = None
        corner_speed = None
        if self._corners and not position_reached:
            corner_distance = self._corners[0][0]-self.distance
            if corner_distance <= self.cornering['stop_distance']:
                corner_command = self._corner_control(pose, motion)
            elif corner_distance < self.cornering['slowdown_distance']:
                lookahead = min(lookahead, self._corners[0][0])
                span = self.cornering['slowdown_distance']-self.cornering['stop_distance']
                ratio = (corner_distance-self.cornering['stop_distance'])/span
                approach_speed = self.cornering['approach_speed']
                corner_speed = approach_speed + ratio*(self.config['max_vx']-approach_speed)
        target = self._visible_target(pose, lookahead)
        if geometry_reached:
            self.vx, self.wz = 0.0, 0.0
            return TrackingOutput(0, 0, self.progress, lateral, goal_yaw, target, matched.phase, True)
        dx, dy = target.x-pose.x, target.y-pose.y
        target_distance = math.hypot(dx, dy)
        heading = wrap_angle(math.atan2(dy, dx)-pose.yaw) if target_distance > 1e-8 else goal_yaw
        stair = matched.mode == 'stair' or matched.phase == 'stair'
        turn_allowed = self.config.get('allow_turn_in_place', False) and (not stair or self.config.get('allow_stair_correction', False))
        phase = matched.phase
        if corner_command is not None:
            heading, desired_wz = corner_command
            desired_vx = 0.
            phase = self._corner_phase
        elif position_reached:
            heading = goal_yaw
            if not turn_allowed:
                raise NavError('FINAL_HEADING_UNREACHABLE', 'Target heading requires an unavailable in-place turn')
            desired_vx = 0.0
            desired_wz = 1.5*heading
        else:
            reverse = self.config.get('allow_reverse', False) and abs(heading) > math.pi/2
            if reverse:
                heading = wrap_angle(heading-math.copysign(math.pi, heading))
            if abs(heading) > self.config.get('heading_stop_threshold', 1.0):
                if not turn_allowed:
                    raise NavError('TRACKING_UNREACHABLE', 'Heading error exceeds available correction capability')
                desired_vx, desired_wz = 0.0, 1.5*heading
            else:
                remaining = max(0, self.path.length-self.distance)
                speed = self.config.get('max_vx', .4)*min(1.0, remaining/self.config.get('slowdown_distance', .8))
                speed = max(self.config.get('min_vx', .03), speed)
                if corner_speed is not None:
                    speed = min(speed, corner_speed)
                    phase = 'corner_approach'
                desired_vx = min(self.config.get('max_vx', .4), speed)*max(.1, math.cos(heading))
                desired_vx *= -1 if reverse else 1
                desired_wz = 2*abs(desired_vx)*math.sin(heading)/max(target_distance, .05)
        if stair and not self.config.get('allow_stair_correction', False):
            desired_wz = 0.0
        max_vx, max_wz = self.config.get('max_vx', .4), self.config.get('max_wz', .6)
        desired_vx, desired_wz = _clip(desired_vx, -max_vx, max_vx), _clip(desired_wz, -max_wz, max_wz)
        # A delayed cycle never authorises one large command jump.
        dt = min(dt, .25)
        dv, dw = self.config.get('max_accel', .3)*dt, self.config.get('max_angular_accel', .6)*dt
        old_vx, old_wz = self.vx, self.wz
        self.vx = _clip(desired_vx, old_vx-dv, old_vx+dv)
        self.wz = _clip(desired_wz, old_wz-dw, old_wz+dw)
        grid = self.path.collision_grid
        if grid is not None:
            # Include a braking reserve so slowing begins before the command
            # reaches the edge of the inflated free space.
            horizon = self.command_ttl + 2*abs(self.vx)/self.config.get('max_accel', .3) + dt
            if (not grid.motion_is_free(pose, self.vx, self.wz, horizon) or
                    not grid.motion_is_free(pose, self.vx, self.wz, self.command_ttl)):
                # Retain the acceleration bounds while reducing forward motion
                # and steering toward a visible point on the validated path.
                slow_vx = _clip(0., old_vx-dv, old_vx+dv)
                turn = _clip(desired_wz if corner_command is not None else 1.5*heading, -max_wz, max_wz)
                turn = _clip(turn, old_wz-dw, old_wz+dw)
                if not turn_allowed and abs(slow_vx) < 1e-8 and abs(turn) > 1e-8:
                    raise NavError('TRACKING_COLLISION', 'Collision avoidance requires an unavailable in-place turn')
                horizon = self.command_ttl + 2*abs(slow_vx)/self.config.get('max_accel', .3) + dt
                if (not grid.motion_is_free(pose, slow_vx, turn, horizon) or
                        not grid.motion_is_free(pose, slow_vx, turn, self.command_ttl)):
                    raise NavError('TRACKING_COLLISION', 'No collision-free command within motion limits')
                self.vx, self.wz = slow_vx, turn
        return TrackingOutput(self.vx, self.wz, self.progress, lateral, heading, target, phase, False)
