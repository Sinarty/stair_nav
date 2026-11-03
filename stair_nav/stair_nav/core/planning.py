"""Conservative annotated-path loading and single-platform occupancy-grid A*.

This module has no ROS dependency. Neither backend validates a real stairway:
preset entry corridors must be surveyed/certified by the integrator, and grid
collision checks cover only the explicitly selected platform raster.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from heapq import heappop, heappush
from pathlib import Path
from typing import Optional
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
    heights: Optional[tuple] = None
    phases: Optional[tuple] = None
    max_step_height: float = .2

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
        heights, phases = data.get('heights'), data.get('phases')
        if heights is not None:
            if (not isinstance(heights, list) or len(heights) != height or
                    any(not isinstance(row, list) or len(row) != width or not _finite(row) or
                        any(isinstance(value, bool) for value in row)
                        for row in heights)):
                raise NavError('INVALID_GRID', 'heights must be a finite grid-sized matrix')
            heights = tuple(tuple(map(float, row)) for row in heights)
        if phases is not None:
            if (not isinstance(phases, list) or len(phases) != height or
                    any(not isinstance(row, list) or len(row) != width or
                        any(value not in ('entry', 'stair', 'exit', 'platform') for value in row)
                        for row in phases)):
                raise NavError('INVALID_GRID', 'phases must be a grid-sized matrix of navigation phases')
            phases = tuple(tuple(row) for row in phases)
        maximum_step = options.get('max_step_height', .2)
        if (not isinstance(maximum_step, (int, float)) or isinstance(maximum_step, bool) or
                not math.isfinite(maximum_step) or maximum_step <= 0):
            raise NavError('INVALID_GRID', 'max_step_height must be positive and finite')
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
        return cls(width, height, resolution, origin, height_z, platform_id, frame, blocked,
                   heights, phases, float(maximum_step))

    def free(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and cell not in self.blocked

    def cell(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def centre(self, cell):
        return (self.origin[0]+(cell[0]+.5)*self.resolution,
                self.origin[1]+(cell[1]+.5)*self.resolution)

    def height_at(self, x, y):
        cell = self.cell(x, y)
        if not self.free(cell):
            return None
        return self.heights[cell[1]][cell[0]] if self.heights is not None else self.height_z

    def can_step(self, current, target):
        """A ground robot may follow adjacent surfaces, never fly over a riser."""
        if not self.free(current) or not self.free(target):
            return False
        if self.heights is None:
            return True
        dx, dy = target[0]-current[0], target[1]-current[1]
        if abs(dx) > 1 or abs(dy) > 1:
            return False
        cells = [current, target]
        if dx and dy:
            cells += [(current[0]+dx, current[1]), (current[0], current[1]+dy)]
        if not all(self.free(cell) for cell in cells):
            return False
        levels = [self.heights[y][x] for x, y in cells]
        return max(levels)-min(levels) <= self.max_step_height+1e-9

    def _height_line_is_free(self, a, b):
        if self.heights is None:
            return True
        fractions = {0., 1.}
        for axis in (0, 1):
            delta = b[axis]-a[axis]
            if abs(delta) < 1e-12:
                continue
            lower = math.floor((min(a[axis], b[axis])-self.origin[axis])/self.resolution)
            upper = math.ceil((max(a[axis], b[axis])-self.origin[axis])/self.resolution)
            for index in range(lower, upper+1):
                fraction = (self.origin[axis]+index*self.resolution-a[axis])/delta
                if 0 < fraction < 1:
                    fractions.add(fraction)
        fractions = sorted(fractions)
        samples = [0.] + [(left+right)/2 for left, right in zip(fractions, fractions[1:])] + [1.]
        cells = [self.cell(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])) for t in samples]
        return all(self.can_step(current, target) for current, target in zip(cells, cells[1:]))

    def line_is_free(self, a, b, margin=0.):
        if not self.free(self.cell(*a)) or not self.free(self.cell(*b)):
            return False
        if not self._height_line_is_free(a, b):
            return False
        if margin and (min(a[0], b[0])-margin < self.origin[0] or
                       min(a[1], b[1])-margin < self.origin[1] or
                       max(a[0], b[0])+margin >= self.origin[0]+self.width*self.resolution or
                       max(a[1], b[1])+margin >= self.origin[1]+self.height*self.resolution):
            return False
        lo = self.cell(min(a[0], b[0])-margin-1e-10, min(a[1], b[1])-margin-1e-10)
        hi = self.cell(max(a[0], b[0])+margin+1e-10, max(a[1], b[1])+margin+1e-10)
        for y in range(max(0, lo[1]), min(self.height-1, hi[1])+1):
            for x in range(max(0, lo[0]), min(self.width-1, hi[0])+1):
                if (x, y) in self.blocked:
                    xmin, ymin = self.origin[0]+x*self.resolution, self.origin[1]+y*self.resolution
                    if _intersects_rectangle(a, b, xmin-margin, ymin-margin,
                                             xmin+self.resolution+margin, ymin+self.resolution+margin):
                        return False
        return True

    def motion_is_free(self, pose, vx, wz, duration):
        """Check the whole constant-twist command, including its curved sweep.

        Each short arc is enclosed by its chord expanded by an upper bound on
        the arc/chord deviation and midpoint position integration error. The
        map already includes the configured radius.
        """
        if (not _finite((pose.x, pose.y, pose.yaw, vx, wz, duration)) or duration < 0):
            return False
        if not self.line_is_free((pose.x, pose.y), (pose.x, pose.y)):
            return False
        if vx == 0 or duration == 0:
            return True
        if not _finite((vx*duration, wz*duration)):
            return False
        steps = max(1, math.ceil(abs(vx)*duration/(self.resolution*.25)),
                    math.ceil(abs(wz)*duration/.05))
        # Keep validation bounded inside the periodic control callback.
        if steps > 4096:
            return False
        step = duration/steps
        # Also enclose midpoint integration for any partition of this lifetime,
        # including one delayed simulator/control step spanning the whole TTL.
        integration_margin = (abs(vx)*duration)*(wz*duration)**2/24
        x, y, yaw = pose.x, pose.y, pose.yaw
        for _ in range(steps):
            angle = wz*step
            half = angle*.5
            scale = math.sin(half)/half if abs(half) > 1e-12 else 1.
            nx = x+vx*step*scale*math.cos(yaw+half)
            ny = y+vx*step*scale*math.sin(yaw+half)
            # |acceleration| = |vx*wz| bounds deviation from the chord.
            margin = abs(vx*wz)*step*step/8 + integration_margin
            if not self.line_is_free((x, y), (nx, ny), margin):
                return False
            x, y, yaw = nx, ny, yaw+angle
        return True


class GridAStarBackend(PlanningBackend):
    def plan(self, route, segment, pose, cancel_event=None):
        options = segment.get('planning', {})
        grid = OccupancyGrid.from_mapping(_load(route, segment['grid_file']), options, cancel_event)
        if grid.heights is not None:
            raise NavError('INVALID_GRID', 'Variable-height surfaces require height_grid_astar')
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
        # The target-cell centre can lie beyond the exact goal, making the
        # terminal polyline pass through the goal and double back. Remove that
        # detour only when the direct endpoint join is collision-free.
        if len(coordinates) >= 3 and grid.line_is_free(coordinates[-3], coordinates[-1]):
            coordinates.pop(-2)
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
                            for x, y in coordinates], collision_grid=grid)


class HeightGridAStarBackend(PlanningBackend):
    """A* over occupied/free cells with an explicit supporting surface height."""
    def plan(self, route, segment, pose, cancel_event=None):
        options = segment.get('planning', {})
        grid = OccupancyGrid.from_mapping(_load(route, segment['grid_file']), options, cancel_event)
        if grid.heights is None:
            raise NavError('INVALID_GRID', 'Stair A* needs an explicit heights matrix')
        if grid.frame != route.get('frame', 'map') or pose.frame != grid.frame:
            raise NavError('FRAME_MISMATCH', 'Height grid, route and localization frames must match')
        if grid.platform_id != segment.get('platform_id'):
            raise NavError('PLATFORM_MISMATCH', 'Height grid surface ID does not match segment')
        goal = route['nodes'][segment['end']]['position']
        if not _finite((pose.x, pose.y, pose.z, pose.yaw, *goal)):
            raise NavError('INVALID_POSE', 'Start and goal coordinates must be finite')
        start, target = grid.cell(pose.x, pose.y), grid.cell(goal[0], goal[1])
        if not grid.free(start):
            raise NavError('INVALID_START', 'Start is outside free stair surface cells')
        if not grid.free(target):
            raise NavError('INVALID_GOAL', 'Goal is outside free stair surface cells')
        tolerance = options.get('platform_z_tolerance', .15)
        if abs(pose.z-grid.height_at(pose.x, pose.y)) > tolerance:
            raise NavError('WRONG_PLATFORM_HEIGHT', 'Live pose is not on the declared stair surface')
        if abs(goal[2]-grid.height_at(goal[0], goal[1])) > tolerance:
            raise NavError('WRONG_PLATFORM_HEIGHT', 'Goal is not on the declared stair surface')
        def xyz(cell):
            return (*grid.centre(cell), grid.heights[cell[1]][cell[0]])
        frontier, costs, parents = [(0., start)], {start: 0.}, {}
        found = False
        while frontier:
            _cancelled(cancel_event)
            _, current = heappop(frontier)
            if current == target:
                found = True
                break
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                neighbour = current[0]+dx, current[1]+dy
                if not grid.can_step(current, neighbour):
                    continue
                # Finish lateral alignment on a tread/landing; never optimize
                # length by traversing a riser diagonally.
                if dx and dy and abs(xyz(current)[2]-xyz(neighbour)[2]) > 1e-9:
                    continue
                candidate = costs[current]+math.dist(xyz(current), xyz(neighbour))
                if candidate < costs.get(neighbour, math.inf):
                    costs[neighbour], parents[neighbour] = candidate, current
                    heappush(frontier, (candidate+math.dist(xyz(neighbour), xyz(target)), neighbour))
        if not found:
            raise NavError('NO_PATH', 'No supported stair route within the configured step-height limit')
        cells, cursor = [target], target
        while cursor != start:
            cursor = parents[cursor]
            cells.append(cursor)
        cells.reverse()
        rows = [(pose.x, pose.y, grid.height_at(pose.x, pose.y))]
        rows += [xyz(cell) for cell in cells]
        rows += [(goal[0], goal[1], grid.height_at(goal[0], goal[1]))]
        rows = [row for i, row in enumerate(rows) if not i or math.dist(row, rows[i-1]) > 1e-8]
        if (len(rows) >= 3 and max(row[2] for row in rows[-3:])-min(row[2] for row in rows[-3:]) < 1e-9
                and grid.line_is_free(rows[-3][:2], rows[-1][:2])):
            rows.pop(-2)
        if any(not grid.line_is_free(a[:2], b[:2]) for a, b in zip(rows, rows[1:])):
            raise NavError('PATH_COLLISION', 'Stair path failed surface collision validation')
        def phase(row):
            x, y = grid.cell(row[0], row[1])
            return grid.phases[y][x] if grid.phases is not None else 'stair'
        return PlannedPath([Waypoint(*row, phase(row), segment.get('mode', 'stair')) for row in rows],
                           collision_grid=grid)


BACKENDS = {'preset_path': PresetPathBackend(), 'grid_astar': GridAStarBackend(),
            'height_grid_astar': HeightGridAStarBackend()}


def plan(route, segment, pose, cancel_event=None):
    try:
        backend = BACKENDS[segment['backend']]
    except KeyError as exc:
        raise NavError('UNKNOWN_BACKEND', f"Unsupported planning backend {segment.get('backend')}") from exc
    return backend.plan(route, segment, pose, cancel_event)
