"""Explicit per-platform/per-stair route configuration; no ROS imports."""
import copy
import math
from pathlib import Path
import yaml
from .models import NavError


def _merge(a, b):
    result = copy.deepcopy(a)
    for key, value in b.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def segment_config(route, segment_id):
    if segment_id not in route['segments']:
        raise NavError('UNKNOWN_SEGMENT', f'Unknown segment: {segment_id}')
    result = _merge(route.get('defaults', {}), route['segments'][segment_id])
    result['id'] = segment_id
    return result


def load_route(filename):
    try:
        with open(filename, encoding='utf-8') as stream:
            route = yaml.safe_load(stream)
        route['_source_file'] = str(Path(filename).resolve())
        route['_base_dir'] = str(Path(filename).resolve().parent)
        validate_route(route)
        return route
    except NavError:
        raise
    except (OSError, TypeError, ValueError, KeyError, yaml.YAMLError) as exc:
        raise NavError('CONFIG_INVALID', str(exc)) from exc


def validate_route(route):
    def check(condition, message):
        if not condition:
            raise NavError('CONFIG_INVALID', message)

    check(isinstance(route, dict), 'Route must be a mapping')
    check(bool(route.get('frame')), 'frame is required')
    check(route.get('reference_point') == 'ground_projection',
          'First version requires the shared ground_projection reference')
    for key, default in (('command_ttl', .2), ('control_period', .05)):
        value = route.get(key, default)
        check(isinstance(value, (int, float)) and math.isfinite(value) and value > 0,
              f'{key} must be finite and positive')
    nodes = route.get('nodes', {})
    check(bool(nodes), 'nodes are required')
    for name, node in nodes.items():
        position = node.get('position', [])
        check(len(position) == 3 and all(math.isfinite(float(x)) for x in position),
              f'{name}: finite [x,y,z] position required')
        check(math.isfinite(float(node['yaw'])), f'{name}: finite yaw required')
        arrival = node['arrival']
        for key in ('xy', 'z', 'yaw', 'confirm_seconds'):
            check(math.isfinite(float(arrival[key])) and arrival[key] > 0,
                  f'{name}: arrival.{key} must be positive')
        check(0 < arrival['progress'] <= 1, f'{name}: invalid progress threshold')
    sequence = route.get('sequence', [])
    check(bool(sequence), 'Explicit sequence is required')
    previous = None
    for sid in sequence:
        seg = segment_config(route, sid)
        check(seg['start'] in nodes and seg['end'] in nodes, f'{sid}: unknown task node')
        check(previous is None or previous == seg['start'], f'{sid}: disconnected sequence')
        previous = seg['end']
    for sid in route['segments']:
        seg = segment_config(route, sid)
        check(seg['backend'] in ('preset_path', 'grid_astar'), f'{sid}: unknown backend')
        check(seg['kind'] in ('stair', 'platform'), f'{sid}: unknown kind')
        check(seg['mode'] in seg['allowed_modes'], f'{sid}: mode must be allowed')
        arrival_mode = nodes[seg['end']]['arrival'].get('required_mode', '')
        check(not arrival_mode or arrival_mode in seg['allowed_modes'],
              f'{sid}: arrival mode must be allowed')
        if seg['backend'] == 'grid_astar':
            check(seg['kind'] == 'platform' and bool(seg.get('platform_id')),
                  f'{sid}: A* requires an explicit single platform')
            check(abs(nodes[seg['start']]['position'][2] - nodes[seg['end']]['position'][2])
                  <= nodes[seg['end']]['arrival']['z'], f'{sid}: A* cannot span floors')
        for key in ('lookahead', 'max_vx', 'max_wz', 'max_accel', 'max_angular_accel',
                    'slowdown_distance', 'max_cross_track', 'max_height_error',
                    'match_forward', 'heading_stop_threshold'):
            value = seg['tracking'][key]
            check(math.isfinite(value) and value > 0, f'{sid}: invalid tracking.{key}')
        for key in ('ready', 'execution', 'pose', 'status', 'arrival'):
            check(math.isfinite(seg['timeouts'][key]) and seg['timeouts'][key] > 0,
                  f'{sid}: invalid timeouts.{key}')
        for key in ('min_vx', 'match_backtrack'):
            value = seg['tracking'][key]
            check(math.isfinite(value) and value >= 0, f'{sid}: invalid tracking.{key}')
        check(seg['tracking']['min_vx'] <= seg['tracking']['max_vx'],
              f'{sid}: min_vx exceeds max_vx')
        for key in ('timeout', 'max_jump', 'join_xy', 'join_z'):
            value = seg['planning'][key]
            check(math.isfinite(value) and value > 0, f'{sid}: invalid planning.{key}')
        radius = seg['planning'].get('inflation_radius', 0)
        check(math.isfinite(radius) and radius >= 0, f'{sid}: invalid inflation radius')
    return route
