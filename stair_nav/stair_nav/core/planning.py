"""Conservative annotated-path loading and single-platform occupancy-grid A*.

This module has no ROS dependency. Neither backend validates a real stairway:
preset entry corridors must be surveyed/certified by the integrator, and grid
collision checks cover only the explicitly selected platform raster.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from heapq import heappop, heappush
from pathlib import Path
import math

import yaml

from .models import NavError, PlannedPath, Waypoint


def _cancelled(cancel_event):
    if cancel_event is not None and cancel_event.is_set():
        raise NavError('PLANNING_CANCELLED', 'Planning was cancelled')


def _load(route, filename):
    path = Path(route.get('_base_dir', '.')) / filename
    try:
        with path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except (OSError, yaml.YAMLError, TypeError) as exc:
        raise NavError('MAP_IO', f'Cannot load {filename}: {exc}') from exc


def _finite(values):
    return all(isinstance(v, (float, int)) and math.isfinite(v) for v in values)


class PlanningBackend(ABC):
    @abstractmethod
    def plan(self, route, segment, pose, cancel_event=None):
        """Return a path in the same frame/reference point as pose."""


class PresetPathBackend(PlanningBackend):
    def plan(self, route, segment, pose, cancel_event=None):
        _cancelled(cancel_event)
        document = _load(route, segment['path_file']) if 'path_file' in segment else {
            'frame': route.get('frame', 'map'), 'points': segment.get('path', [])}
        if not isinstance(document, dict):
            raise NavError('INVALID_PATH', 'Preset YAML must be a mapping with points')
        if document.get('frame') != route.get('frame', 'map') or pose.frame != document['frame']:
            raise NavError('FRAME_MISMATCH', 'Preset, route, and localization frames must match')
        reference = route.get('reference_point')
        if document.get('reference_point', reference) != reference:
            raise NavError('REFERENCE_MISMATCH', 'Preset and localization reference points differ')
        points, seen_coordinates = [], set()
        maximum_jump = float(segment.get('planning', {}).get('max_jump', 2.0))
        allowed = segment.get('allowed_modes', [segment.get('mode', 'flat')])
        rows = document.get('points')
        if not isinstance(rows, list) or len(rows) < 2:
            raise NavError('EMPTY_PATH', 'Preset requires at least two annotated points')
        for row in rows:
            _cancelled(cancel_event)
            if not isinstance(row, dict) or not isinstance(row.get('position'), list):
                raise NavError('INVALID_PATH', 'Each point needs position, phase and mode')
            xyz = row['position']
            if len(xyz) != 3 or not _finite(xyz):
                raise NavError('INVALID_PATH', 'Preset coordinates must be finite XYZ triples')
            if not row.get('phase') or row.get('mode') not in allowed:
                raise NavError('INVALID_PATH', 'Missing phase or disallowed point motion mode')
            point = Waypoint(*map(float, xyz), str(row['phase']), str(row['mode']))
            coordinate_key = tuple(map(float, xyz))
            if coordinate_key in seen_coordinates:
                raise NavError('DUPLICATE_POINT', 'Repeated preset XYZ point; loops require separate explicit segments')
            seen_coordinates.add(coordinate_key)
            if points:
                a = points[-1]
                distance = math.dist((a.x, a.y, a.z), xyz)
                if distance < 1e-6:
                    raise NavError('DUPLICATE_POINT', 'Adjacent duplicate preset points')
                if distance > maximum_jump:
                    raise NavError('PATH_JUMP', f'Preset jump {distance:.3f} exceeds {maximum_jump:.3f} m')
                if math.hypot(point.x - a.x, point.y - a.y) < 1e-6:
                    raise NavError('VERTICAL_PATH', 'Pure vertical edges cannot be tracked using horizontal vx')
            points.append(point)
        start, end = points[0], points[-1]
        goal = route['nodes'][segment['end']]
        tolerance = goal.get('arrival', {})
        goal_xyz = goal['position']
        if (math.hypot(end.x - goal_xyz[0], end.y - goal_xyz[1]) > tolerance.get('xy', .15)
                or abs(end.z - goal_xyz[2]) > tolerance.get('z', .15)):
            raise NavError('PATH_GOAL_MISMATCH', 'Preset endpoint is outside the configured target region')
        options = segment.get('planning', {})
        # Acceptance means the surveyed entry corridor is explicitly authorised
        # for this small neighbourhood. Do not fabricate an unchecked join edge.
        if (math.hypot(pose.x - start.x, pose.y - start.y) > options.get('join_xy', .25)
                or abs(pose.z - start.z) > options.get('join_z', .15)):
            raise NavError('JOIN_OUT_OF_RANGE', 'Live pose is outside the certified preset entry neighbourhood')
        offset = math.dist((pose.x, pose.y, pose.z), (start.x, start.y, start.z))
        if offset > 1e-6 and not options.get('entry_corridor_confirmed', False):
            raise NavError('JOIN_UNVERIFIED', 'Nonzero entry offset needs an explicitly confirmed local corridor')
        if not _finite((pose.x, pose.y, pose.z, pose.yaw)):
            raise NavError('INVALID_POSE', 'Localization contains non-finite values')
        return PlannedPath(points)


def _intersects_rectangle(a, b, xmin, ymin, xmax, ymax):
    """Closed line/closed rectangle intersection; boundary grazing is blocked."""
    t0, t1 = 0.0, 1.0
    for origin, delta, lo, hi in ((a[0], b[0]-a[0], xmin, xmax),
                                  (a[1], b[1]-a[1], ymin, ymax)):
        if abs(delta) < 1e-12:
            if origin < lo or origin > hi:
                return False
        else:
            enter, leave = (lo-origin)/delta, (hi-origin)/delta
            if enter > leave:
                enter, leave = leave, enter
            t0, t1 = max(t0, enter), min(t1, leave)
            if t0 > t1 + 1e-12:
                return False
    return True


@dataclass
class OccupancyGrid:
    width: int
    height: int
    resolution: float
    origin: tuple
    height_z: float
    platform_id: str
    frame: str
    blocked: set

    @classmethod
    def from_mapping(cls, data, options=None, cancel_event=None):
        options = options or {}
        try:
            width, height = int(data['width']), int(data['height'])
            resolution = float(data['resolution'])
            origin = tuple(data['origin'])
            height_z = float(data['height_z'])
            platform_id, frame = str(data['platform_id']), str(data['frame'])
            rows = data['data']
            if (width <= 0 or height <= 0 or resolution <= 0 or len(origin) != 2
                    or not _finite((*origin, resolution, height_z)) or len(rows) != height
                    or any(not isinstance(row, list) or len(row) != width for row in rows)
                    or not all(_finite(row) for row in rows)):
                raise ValueError('invalid grid dimensions, coordinates, or data rows')
        except (KeyError, TypeError, ValueError) as exc:
            raise NavError('INVALID_GRID', str(exc)) from exc
        threshold = options.get('occupied_threshold', 50)
        unknown_blocked = options.get('unknown_is_obstacle', True)
        blocked = {(x, y) for y, row in enumerate(rows) for x, value in enumerate(row)
                   if value >= threshold or (value < 0 and unknown_blocked)}
        radius = float(options.get('inflation_radius', 0.0))
        if not math.isfinite(radius) or radius < 0:
            raise NavError('INVALID_GRID', 'Inflation radius must be finite and nonnegative')
        if radius > 0:
            original = set(blocked)
            reach = math.ceil((radius + resolution * math.sqrt(2)) / resolution)
            # Inflate whole cells conservatively: all points of any retained free
            # cell clear occupied cell squares by at least the configured radius.
            limit = radius + resolution * math.sqrt(2) / 2
            for ox, oy in original:
                _cancelled(cancel_event)
                for dy in range(-reach, reach + 1):
                    for dx in range(-reach, reach + 1):
                        x, y = ox + dx, oy + dy
                        nx, ny = max(abs(dx)*resolution-resolution/2, 0), max(abs(dy)*resolution-resolution/2, 0)
                        if 0 <= x < width and 0 <= y < height and math.hypot(nx, ny) <= limit:
                            blocked.add((x, y))
            for y in range(height):
                _cancelled(cancel_event)
                for x in range(width):
                    edge_distance = min(x, y, width-1-x, height-1-y) * resolution
                    if edge_distance < radius:
                        blocked.add((x, y))
        return cls(width, height, resolution, origin, height_z, platform_id, frame, blocked)

    def free(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and cell not in self.blocked

    def cell(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def centre(self, cell):
        return (self.origin[0]+(cell[0]+.5)*self.resolution,
                self.origin[1]+(cell[1]+.5)*self.resolution)

    def line_is_free(self, a, b):
        if not self.free(self.cell(*a)) or not self.free(self.cell(*b)):
            return False
        lo = self.cell(min(a[0], b[0])-1e-10, min(a[1], b[1])-1e-10)
        hi = self.cell(max(a[0], b[0])+1e-10, max(a[1], b[1])+1e-10)
        for y in range(max(0, lo[1]), min(self.height-1, hi[1])+1):
            for x in range(max(0, lo[0]), min(self.width-1, hi[0])+1):
                if (x, y) in self.blocked:
                    xmin, ymin = self.origin[0]+x*self.resolution, self.origin[1]+y*self.resolution
                    if _intersects_rectangle(a, b, xmin, ymin, xmin+self.resolution, ymin+self.resolution):
                        return False
        return True


class GridAStarBackend(PlanningBackend):
    def plan(self, route, segment, pose, cancel_event=None):
        options = segment.get('planning', {})
        grid = OccupancyGrid.from_mapping(_load(route, segment['grid_file']), options, cancel_event)
        if grid.frame != route.get('frame', 'map') or pose.frame != grid.frame:
            raise NavError('FRAME_MISMATCH', 'Grid, route and localization frames must match')
        if grid.platform_id != segment.get('platform_id'):
            raise NavError('PLATFORM_MISMATCH', 'Explicit grid platform_id does not match segment')
        goal = route['nodes'][segment['end']]['position']
        z_tolerance = options.get('platform_z_tolerance', .15)
        if abs(pose.z-grid.height_z) > z_tolerance or abs(goal[2]-grid.height_z) > z_tolerance:
            raise NavError('WRONG_PLATFORM_HEIGHT', 'Grid is only valid at its declared platform height')
        if not _finite((pose.x, pose.y, pose.z, pose.yaw, *goal)):
            raise NavError('INVALID_POSE', 'Start and goal coordinates must be finite')
        start, target = grid.cell(pose.x, pose.y), grid.cell(goal[0], goal[1])
        if not grid.free(start):
            raise NavError('INVALID_START', 'Start is occupied, unknown, inflated or outside this platform grid')
        if not grid.free(target):
            raise NavError('INVALID_GOAL', 'Goal is occupied, unknown, inflated or outside this platform grid')
        frontier, came_from, costs = [(0.0, start)], {}, {start: 0.0}
        found = False
        while frontier:
            _cancelled(cancel_event)
            _, current = heappop(frontier)
            if current == target:
                found = True
                break
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                neighbour = current[0]+dx, current[1]+dy
                if not grid.free(neighbour):
                    continue
                if dx and dy and (not grid.free((current[0]+dx, current[1]))
                                  or not grid.free((current[0], current[1]+dy))):
                    continue
                candidate = costs[current] + math.hypot(dx, dy)
                if candidate < costs.get(neighbour, math.inf):
                    costs[neighbour], came_from[neighbour] = candidate, current
                    priority = candidate + math.hypot(neighbour[0]-target[0], neighbour[1]-target[1])
                    heappush(frontier, (priority, neighbour))
        if not found:
            raise NavError('NO_PATH', 'Goal is unreachable on the selected inflated platform grid')
        cells, cursor = [target], target
        while cursor != start:
            cursor = came_from[cursor]
            cells.append(cursor)
        cells.reverse()
        if start == target:
            coordinates = [(pose.x, pose.y), (goal[0], goal[1])]
        else:
            coordinates = [(pose.x, pose.y)] + [grid.centre(cell) for cell in cells] + [(goal[0], goal[1])]
        coordinates = [p for i, p in enumerate(coordinates) if not i or math.dist(p, coordinates[i-1]) > 1e-8]
        if options.get('simplify', False) and len(coordinates) > 2:
            simplified, i = [coordinates[0]], 0
            while i < len(coordinates)-1:
                _cancelled(cancel_event)
                j = len(coordinates)-1
                while j > i+1 and not grid.line_is_free(coordinates[i], coordinates[j]):
                    j -= 1
                simplified.append(coordinates[j])
                i = j
            coordinates = simplified
        # Recheck raw and simplified output, including exact start/goal-to-cell joins.
        if any(not grid.line_is_free(a, b) for a, b in zip(coordinates, coordinates[1:])):
            raise NavError('PATH_COLLISION', 'Output path failed conservative post-planning collision validation')
        return PlannedPath([Waypoint(x, y, grid.height_z, 'platform', segment.get('mode', 'flat'))
                            for x, y in coordinates])


BACKENDS = {'preset_path': PresetPathBackend(), 'grid_astar': GridAStarBackend()}


def plan(route, segment, pose, cancel_event=None):
    try:
        backend = BACKENDS[segment['backend']]
    except KeyError as exc:
        raise NavError('UNKNOWN_BACKEND', f"Unsupported planning backend {segment.get('backend')}") from exc
    return backend.plan(route, segment, pose, cancel_event)
