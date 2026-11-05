"""Flat-ground point-cloud stop corridor and conservative clear debouncing.

Points and optional path coordinates use a ground-reference frame with X forward,
Y left and Z up.  This module does not transform clouds or infer ground height.
The ROS caller must check source/receipt freshness and transform availability.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObstacleConfig:
    forward_distance: float = 1.2
    half_width: float = .45
    min_height: float = .12
    max_height: float = 1.8
    self_front: float = .3
    self_rear: float = .4
    self_half_width: float = .25
    voxel_size: float = .08
    min_points: int = 3
    clear_seconds: float = 1.
    cloud_timeout: float = .5

    def __post_init__(self):
        for name in ('forward_distance', 'half_width', 'max_height', 'voxel_size',
                     'cloud_timeout'):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or value <= 0):
                raise ValueError('{} must be positive and finite'.format(name))
        for name in ('min_height', 'self_front', 'self_rear', 'self_half_width',
                     'clear_seconds'):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or value < 0):
                raise ValueError('{} must be nonnegative and finite'.format(name))
        if self.max_height <= self.min_height:
            raise ValueError('max_height must exceed min_height')
        if (isinstance(self.min_points, bool) or not isinstance(self.min_points, int) or
                self.min_points < 1):
            raise ValueError('min_points must be a positive integer')


@dataclass(frozen=True)
class ObstacleResult:
    valid: bool
    blocked: bool
    reason: str
    nearest_distance: float = math.inf
    points: int = 0


def _invalid(reason):
    return ObstacleResult(False, True, reason)


def _corridor_segments(path_xy, distance):
    """Build a length-limited local polyline, including the robot-to-path join."""
    path = [(0., 0.)]
    for point in path_xy:
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('path coordinates must be finite')
        if (x, y) != path[-1]:
            path.append((x, y))
    if len(path) == 1:
        # Missing/zero-length routes retain the full forward stop corridor.
        return None
    segments, offset = [], 0.
    for start, end in zip(path, path[1:]):
        length = math.hypot(end[0]-start[0], end[1]-start[1])
        retained = min(length, distance-offset)
        ratio = retained/length
        clipped = (start[0]+ratio*(end[0]-start[0]), start[1]+ratio*(end[1]-start[1]))
        segments.append((start, clipped, offset, retained))
        offset += retained
        if offset >= distance:
            break
    return segments


def _path_distance(x, y, segments, half_width):
    """Return nearest along-path projection inside the swept circular footprint."""
    best = math.inf
    for start, end, offset, length in segments:
        dx, dy = end[0]-start[0], end[1]-start[1]
        fraction = (max(0., min(1., ((x-start[0])*dx+(y-start[1])*dy)/(length*length)))
                    if length else 0.)
        px, py = start[0]+fraction*dx, start[1]+fraction*dy
        if math.hypot(x-px, y-py) <= half_width:
            best = min(best, offset+fraction*length)
    return best


def corridor_obstacles(points, config, path_xy=None):
    """Inspect one cloud; ``points`` counts unique occupied corridor voxels.

    The default corridor is 0 <= X <= forward_distance, |Y| <= half_width.
    An optional local path replaces it with a disk swept along its first
    forward_distance metres, including its corner and endpoint footprints.
    This corridor depends on the retained path, never the current velocity.

    Nonfinite points are discarded. Empty/all-nonfinite or malformed clouds are
    invalid rather than clear. Finite floor/background points establish a valid
    observation even when no points survive height/corridor/self filtering.
    """
    try:
        segments = None if path_xy is None else _corridor_segments(path_xy, config.forward_distance)
    except (TypeError, ValueError, IndexError, OverflowError):
        return _invalid('INVALID_PATH')
    voxels, nearest, finite_points = set(), math.inf, 0
    try:
        for point in points:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            finite_points += 1
            if not config.min_height <= z <= config.max_height:
                continue
            if -config.self_rear <= x <= config.self_front and abs(y) <= config.self_half_width:
                continue
            if segments is None:
                if not (0. <= x <= config.forward_distance and abs(y) <= config.half_width):
                    continue
                distance = x
            else:
                distance = _path_distance(x, y, segments, config.half_width)
                if not math.isfinite(distance):
                    continue
            voxels.add((math.floor(x/config.voxel_size), math.floor(y/config.voxel_size),
                        math.floor(z/config.voxel_size)))
            nearest = min(nearest, distance)
    except (TypeError, ValueError, IndexError, OverflowError):
        return _invalid('INVALID_CLOUD')
    if not finite_points:
        return _invalid('EMPTY_CLOUD')
    blocked = len(voxels) >= config.min_points
    return ObstacleResult(True, blocked, 'OBSTACLE' if blocked else 'CLEAR', nearest, len(voxels))


class ObstacleDetector:
    """Stop on the first occupied/invalid frame; clear only on continuous frames.

    ``stamp`` is the source timestamp in seconds, not a periodically refreshed
    publication time.  Equal/reversed stamps and gaps beyond cloud_timeout reset
    clearance. The caller must also invalidate on steady-clock receipt timeout;
    source stamps alone cannot detect a stopped stream while observe is idle.
    """
    def __init__(self, config=None):
        self.config = config or ObstacleConfig()
        self._last_stamp = None
        self._clear_since = None
        self.result = _invalid('NO_CLOUD')

    def observe(self, points, stamp, valid=True, path_xy=None):
        try:
            source_stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            source_stamp = math.nan
        if isinstance(stamp, bool) or not math.isfinite(source_stamp) or source_stamp < 0:
            return self._invalidate('INVALID_STAMP')
        previous = self._last_stamp
        if previous is not None and source_stamp <= previous:
            return self._invalidate('NON_MONOTONIC_STAMP')
        self._last_stamp = source_stamp
        if not valid:
            return self._invalidate('INVALID_OBSERVATION')
        if previous is not None and source_stamp-previous > self.config.cloud_timeout:
            self._clear_since = None
            self.result = _invalid('CLOUD_GAP')
        geometry = corridor_obstacles(points, self.config, path_xy)
        if not geometry.valid or geometry.blocked:
            self._clear_since = None
            self.result = geometry
            return self.result
        if self._clear_since is None:
            self._clear_since = source_stamp
        clearing = source_stamp-self._clear_since+1e-9 < self.config.clear_seconds
        self.result = ObstacleResult(True, clearing, 'CLEARING' if clearing else 'CLEAR',
                                     geometry.nearest_distance, geometry.points)
        return self.result

    def _invalidate(self, reason):
        self._clear_since = None
        self.result = _invalid(reason)
        return self.result
