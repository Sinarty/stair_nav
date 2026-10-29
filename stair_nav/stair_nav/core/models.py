"""Shared values. Distances are metres, angles radians, stamps clock seconds."""
from dataclasses import dataclass, field
from bisect import bisect_right
import math


class NavError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    stamp: float = 0.0
    frame: str = 'map'


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    phase: str = 'platform'
    mode: str = 'flat'


@dataclass
class PlannedPath:
    points: list
    # The exact inflated platform map used by planning; absent for stair presets.
    collision_grid: object = field(default=None, repr=False, compare=False)
    cumulative: list = field(init=False)
    length: float = field(init=False)

    def __post_init__(self):
        if not self.points:
            raise NavError('EMPTY_PATH', 'Path contains no points')
        self.cumulative = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.dist(
                (a.x, a.y, a.z), (b.x, b.y, b.z)))
        self.length = self.cumulative[-1]

    def sample(self, distance):
        if len(self.points) == 1 or distance >= self.length:
            return self.points[-1]
        distance = max(0.0, distance)
        i = min(bisect_right(self.cumulative, distance) - 1, len(self.points) - 2)
        a, b = self.points[i:i + 2]
        span = self.cumulative[i + 1] - self.cumulative[i]
        t = (distance - self.cumulative[i]) / span if span > 0 else 0
        return Waypoint(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                        a.z + t * (b.z - a.z), a.phase, a.mode)


@dataclass(frozen=True)
class MotionFeedback:
    stamp: float
    execution_id: str
    segment_id: str
    mode: str
    ready: bool = True
    exit_confirmed: bool = False
    stable: bool = False
    hold_confirmed: bool = False
    fault_code: str = ''
    fault_message: str = ''
    capabilities: tuple = ()
    supported_modes: tuple = ()


@dataclass(frozen=True)
class Command:
    stamp: float
    valid_until: float
    task_id: str
    execution_id: str
    segment_id: str
    phase: str
    mode: str
    behavior: str = 'HOLD'
    vx: float = 0.0
    wz: float = 0.0
    valid: bool = False


@dataclass
class Status:
    task_id: str = ''
    execution_id: str = ''
    segment_id: str = ''
    phase: str = ''
    state: str = 'IDLE'
    progress: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    error_code: str = ''
    error_message: str = ''
    geometry_reached: bool = False
    navigation_stopped: bool = True
    hold_confirmed: bool = False


@dataclass
class ExecutionOutput:
    status: Status
    command: Command
    path: object = None
    target: object = None
