"""Pure Pursuit on an annotated 3-D path, producing horizontal vx and yaw rate."""
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
    def __init__(self, path, tracking, goal_node):
        self.path, self.config, self.goal = path, tracking, goal_node
        self.distance, self.progress = 0.0, 0.0
        self.vx, self.wz = 0.0, 0.0
        self.last_pose = None

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

    def update(self, pose, dt):
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
        target = self.path.sample(min(self.path.length, self.distance+self.config.get('lookahead', .6)))
        goal_position, arrival = self.goal['position'], self.goal.get('arrival', {})
        goal_xy = math.hypot(pose.x-goal_position[0], pose.y-goal_position[1])
        goal_z = abs(pose.z-goal_position[2])
        goal_yaw = wrap_angle(self.goal.get('yaw', 0)-pose.yaw)
        position_reached = (goal_xy <= arrival.get('xy', .15) and goal_z <= arrival.get('z', .15)
                            and self.progress >= arrival.get('progress', .97))
        geometry_reached = position_reached and abs(goal_yaw) <= arrival.get('yaw', .15)
        if geometry_reached:
            self.vx, self.wz = 0.0, 0.0
            return TrackingOutput(0, 0, self.progress, lateral, goal_yaw, target, matched.phase, True)
        dx, dy = target.x-pose.x, target.y-pose.y
        target_distance = math.hypot(dx, dy)
        heading = wrap_angle(math.atan2(dy, dx)-pose.yaw) if target_distance > 1e-8 else goal_yaw
        stair = matched.mode == 'stair' or matched.phase == 'stair'
        turn_allowed = self.config.get('allow_turn_in_place', False) and (not stair or self.config.get('allow_stair_correction', False))
        if position_reached:
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
        self.vx = _clip(desired_vx, self.vx-dv, self.vx+dv)
        self.wz = _clip(desired_wz, self.wz-dw, self.wz+dw)
        return TrackingOutput(self.vx, self.wz, self.progress, lateral, heading, target, matched.phase, False)
