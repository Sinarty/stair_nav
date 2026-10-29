#!/usr/bin/env python3
"""Generate the synthetic O→A→B→C→D demo's occupancy and height grids.

AB and CD ascend; BC is a flat corridor. Dimensions are demonstration values,
not measurements of a real site. Regenerate the YAML after changing arguments.
"""
import argparse
import math
from pathlib import Path

import yaml


class RowDumper(yaml.SafeDumper):
    pass


def represent_list(dumper, values):
    # Keep one matrix row per line while ordinary nested records remain readable.
    return dumper.represent_sequence('tag:yaml.org,2002:seq', values,
                                    flow_style=all(not isinstance(v, (list, dict)) for v in values))


RowDumper.add_representer(list, represent_list)


def write_yaml(folder, name, document):
    title = '# SYNTHETIC DEMO ONLY. Editable dimensions; not a surveyed robot map.\n'
    (folder / name).write_text(title + yaml.dump(document, Dumper=RowDumper,
                                               sort_keys=False, allow_unicode=True), encoding='utf-8')


def raster(platform, xmin, ymin, width, height, resolution, z, thresholds=(), rise=0., end_y=None):
    rows, heights, phases = [], [], []
    for y in range(height):
        wy = ymin + (y + .5) * resolution
        level = sum(wy >= threshold - 1e-8 for threshold in thresholds)
        value = round(z + level * rise, 6)
        phase = 'platform'
        if thresholds:
            phase = ('entry' if wy < thresholds[0] - 1e-8 else
                     'stair' if wy < thresholds[-1] + resolution - 1e-8 else 'exit')
            if end_y is not None and wy >= end_y - 1e-8:
                phase = 'platform'
        rows.append([100 if x in (0, width - 1) or y in (0, height - 1) else 0
                     for x in range(width)])
        heights.append([value] * width)
        phases.append([phase] * width)
    data = dict(frame='map', reference_point='ground_projection', platform_id=platform,
                height_z=float(z), width=width, height=height, resolution=resolution,
                origin=[round(xmin, 6), round(ymin, 6)], data=rows)
    if thresholds:
        data.update(heights=heights, phases=phases)
    return data


def generate(folder, ab_length=4., bc_length=6., cd_length=3., approach_length=4.,
             rise=1.2, resolution=.1, steps=6):
    if min(ab_length, bc_length, cd_length, approach_length, rise, resolution) <= 0 or steps < 1:
        raise ValueError('All dimensions and step count must be positive')
    # Node coordinates and origins align all target nodes with cell centres.
    for value in (ab_length, bc_length, cd_length, approach_length, .5, .6):
        if abs(value / resolution - round(value / resolution)) > 1e-6:
            raise ValueError('Lengths and 0.5/0.6 m landings must be multiples of resolution')
    if ab_length < 3.2 or cd_length < 2.8:
        raise ValueError('Demo flights need enough length for steps and entry/exit landings')
    folder.mkdir(parents=True, exist_ok=True)
    half_cell = resolution / 2
    ax, ay = -approach_length, -.5
    by, cx, dy = ay + ab_length, ax + bc_length, ay + ab_length + cd_length
    north = math.pi / 2
    arrival = dict(xy=.12, z=.12, yaw=.15, progress=.97,
                   confirm_seconds=.35, require_exit=False, require_stable=True,
                   required_mode='flat')
    nodes = {}
    for name, position, yaw, exit_required in (
        ('O', [0., 0., 0.], math.atan2(ay, ax), False),
        ('A', [ax, ay, 0.], north, False),
        ('B', [ax, by, rise], north, True),
        ('C', [cx, by, rise], north, False),
        ('D', [cx, dy, 2 * rise], north, True),
    ):
        nodes[name] = dict(position=position, yaw=yaw,
                           arrival={**arrival, 'require_exit': exit_required})
        if name in ('A', 'C'):
            # Align on the flat entry landing before requesting stair mode;
            # the stair controller intentionally cannot turn in place.
            nodes[name]['arrival'].update(xy=.04, yaw=.05)
    tracking = dict(lookahead=.25, max_vx=.45, max_wz=.8, max_accel=.6,
                    max_angular_accel=1.6, slowdown_distance=.6, min_vx=.025,
                    max_cross_track=.3, max_height_error=.3, match_backtrack=.15,
                    match_forward=.7, allow_turn_in_place=False, allow_reverse=False,
                    allow_stair_correction=True, heading_stop_threshold=1.0)
    planning = dict(max_jump=.5, join_xy=.22, join_z=.15, inflation_radius=.12,
                    unknown_is_obstacle=True, timeout=3., max_step_height=round(rise / steps, 6))
    flat = dict(kind='platform', backend='grid_astar', mode='flat', allowed_modes=['flat'],
                required_capabilities=['turn_in_place'],
                tracking=dict(max_vx=.45, allow_turn_in_place=True, allow_stair_correction=False),
                planning=dict(simplify=True))
    stair = dict(kind='stair', backend='height_grid_astar', mode='stair',
                 allowed_modes=['stair', 'flat'],
                 required_capabilities=['stair_motion', 'stair_yaw_correction'])
    route = dict(frame='map', reference_point='ground_projection', control_period=.05,
                 command_ttl=.2, task_transport_timeout=5., sequence=['AB', 'BC', 'CD'],
                 approach_segment='APPROACH_A',
                 defaults=dict(allowed_modes=['stair', 'flat'],
                               required_capabilities=['stair_motion', 'stair_yaw_correction'],
                               tracking=tracking, planning=planning,
                               timeouts=dict(ready=3., execution=50., pose=.5, status=.5, arrival=6.)),
                 nodes=nodes,
                 segments={
                     'APPROACH_A': dict(**flat, start='A', end='A', platform_id='origin_platform',
                                       grid_file='origin_platform.yaml'),
                     'AB': dict(**stair, start='A', end='B', platform_id='stair_ab',
                                grid_file='stair_ab_grid.yaml'),
                     'BC': dict(**flat, start='B', end='C', platform_id='corridor_bc',
                                grid_file='corridor_bc.yaml'),
                     'CD': dict(**stair, start='C', end='D', platform_id='stair_cd',
                                grid_file='stair_cd_grid.yaml'),
                 },
                 simulation=dict(initial_node='O', hold_delay=.25, stable_delay=.25,
                                 exit_margin=.35,
                                 capabilities=['stair_motion', 'stair_yaw_correction', 'turn_in_place'],
                                 supported_modes=['stair', 'flat'], profiles={}))
    # Flat maps retain a margin for turning at A/B/C; stair grids are 1.6 m wide.
    origin = raster('origin_platform', ax - .7 - half_cell, ay - .7 - half_cell,
                    round((approach_length + 1.6) / resolution), round(2.2 / resolution),
                    resolution, 0.)
    corridor = raster('corridor_bc', ax - .7 - half_cell, by - .5 - half_cell,
                      round((bc_length + 1.6) / resolution), round(1.2 / resolution),
                      resolution, rise)
    ab_thresholds = [ay + .6 + index * .4 for index in range(steps)]
    cd_thresholds = [by + .6 + index * .35 for index in range(steps)]
    if ab_thresholds[-1] > by - .5 or cd_thresholds[-1] > dy - .5:
        raise ValueError('Step count/run leaves less than 0.5 m exit landing')
    ab = raster('stair_ab', ax - .7 - half_cell, ay - .5 - half_cell,
                round(1.6 / resolution), round((ab_length + 1.2) / resolution),
                resolution, 0., ab_thresholds, rise / steps, by)
    cd = raster('stair_cd', cx - .7 - half_cell, by - .5 - half_cell,
                round(1.6 / resolution), round((cd_length + 1.2) / resolution),
                resolution, rise, cd_thresholds, rise / steps, dy)
    mission = dict(route_id='inspection_demo', route_file='staircase_demo_route.yaml',
                   sequence=['AB', 'BC', 'CD'], approach_segment='APPROACH_A',
                   timeouts=dict(service=5., ready=5., acceptance=3., execution=55.,
                                 confirmation=8., feedback=2., cancel_response=3.,
                                 cancel_terminal=5., cleanup=5.),
                   readiness=dict(pose_max_age=.5, motion_max_age=.5,
                                  navigation_max_age=1., require_start_node=True), per_segment={})
    for name, data in [('staircase_demo_route.yaml', route), ('staircase_demo_mission.yaml', mission),
                       ('origin_platform.yaml', origin), ('corridor_bc.yaml', corridor),
                       ('stair_ab_grid.yaml', ab), ('stair_cd_grid.yaml', cd)]:
        write_yaml(folder, name, data)
    return route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[1] / 'stair_nav/config')
    parser.add_argument('--ab-length', type=float, default=4.)
    parser.add_argument('--bc-length', type=float, default=6.)
    parser.add_argument('--cd-length', type=float, default=3.)
    parser.add_argument('--approach-length', type=float, default=4.)
    parser.add_argument('--rise', type=float, default=1.2, help='Height gained by each flight (m)')
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--steps', type=int, default=6)
    args = parser.parse_args()
    route = generate(args.out, args.ab_length, args.bc_length, args.cd_length,
                     args.approach_length, args.rise, args.resolution, args.steps)
    print('Generated O→A→B→C→D maps:', {key: value['position'] for key, value in route['nodes'].items()})


if __name__ == '__main__':
    main()
