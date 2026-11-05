#!/usr/bin/env python3
"""Prove the default flat obstacle demo starts, pauses and resumes by itself.

No mission Action goal or obstacle command is sent by this observer. It starts
the normal launch defaults, observes the live cloud/person/state/command chain,
and reads control release after successful completion. Use a sourced build.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import traceback

from check_obstacle_stop import finite, stop_process_group


def run(options):
    os.environ['ROS_DOMAIN_ID'] = str(options.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(options.output_dir/'ros_logs')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMsg
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray
    from stair_nav_interfaces.msg import (
        ControlCommand, MissionStatus, MotionStatus, ObstacleStatus, SegmentStatus,
    )
    from stair_nav_interfaces.srv import GetNavigationState
    from stair_nav.ros_utils import durable_qos, live_qos, seconds

    started = time.monotonic()
    deadline = started+options.timeout
    samples = {key: [] for key in ('navigation', 'mission', 'command', 'obstacle',
                                   'pose', 'path', 'motion', 'person', 'cloud')}
    latest, transitions = {}, []
    context, executor, node, process = None, None, None, None
    report = dict(passed=False, domain=options.domain, sends_mission_goal=False,
                  sends_obstacle_command=False, rviz=options.rviz,
                  launch_log=str(options.output_dir/'launch.log'))
    log = (options.output_dir/'launch.log').open('w', encoding='utf-8')

    def now():
        return time.monotonic()-started

    def record(key, **data):
        value = dict(t=round(now(), 6), **data)
        samples[key].append(value)
        latest[key] = value
        return value

    def on_navigation(msg):
        previous = latest.get('navigation')
        value = record('navigation', stamp=seconds(msg.header.stamp),
                       execution_id=msg.execution_id, task_id=msg.task_id,
                       segment_id=msg.segment_id, state=msg.state, phase=msg.phase,
                       progress=msg.progress, error_code=msg.error_code,
                       error_message=msg.error_message,
                       navigation_stopped=msg.navigation_stopped, hold_confirmed=msg.hold_confirmed)
        if previous is None or (previous['state'], previous['phase']) != (msg.state, msg.phase):
            transitions.append(value)

    def on_mission(msg):
        record('mission', stamp=seconds(msg.header.stamp), route_id=msg.route_id,
               task_id=msg.task_id, execution_id=msg.execution_id, state=msg.state,
               segment_state=msg.segment_state, completed_segments=msg.completed_segments,
               navigation_stopped=msg.navigation_stopped, hold_confirmed=msg.hold_confirmed,
               blocked=msg.blocked, error_code=msg.error_code, error_message=msg.error_message)

    def on_command(msg):
        record('command', stamp=seconds(msg.header.stamp), valid_until=seconds(msg.valid_until),
               execution_id=msg.execution_id, task_id=msg.task_id, behavior=msg.behavior,
               valid=msg.valid, vx=msg.vx, wz=msg.wz)

    def on_obstacle(msg):
        record('obstacle', stamp=seconds(msg.header.stamp), valid_until=seconds(msg.valid_until),
               path_stamp=seconds(msg.path_stamp), execution_id=msg.execution_id,
               valid=msg.valid, blocked=msg.blocked, reason=msg.reason,
               nearest_distance=finite(msg.nearest_distance), occupied_voxels=msg.occupied_voxels)

    def on_pose(msg):
        p = msg.pose.position
        record('pose', stamp=seconds(msg.header.stamp), x=p.x, y=p.y, z=p.z)

    def on_path(msg):
        points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        signature = hashlib.sha256(json.dumps(points, separators=(',', ':')).encode()).hexdigest()
        record('path', stamp=seconds(msg.header.stamp), signature=signature, points=points)

    def on_motion(msg):
        record('motion', stamp=seconds(msg.header.stamp), execution_id=msg.execution_id,
               ready=msg.ready, stable=msg.stable, hold_confirmed=msg.hold_confirmed,
               fault_code=msg.fault_code)

    def on_person(msg):
        record('person', active=any(m.action == Marker.ADD for m in msg.markers),
               markers=[dict(ns=m.ns, id=m.id, action=m.action, type=m.type,
                             position=[m.pose.position.x, m.pose.position.y, m.pose.position.z])
                        for m in msg.markers])

    def on_cloud(msg):
        record('cloud', stamp=seconds(msg.header.stamp), frame=msg.header.frame_id,
               count=msg.width*msg.height, fields=[field.name for field in msg.fields])

    def wait(predicate, timeout, description):
        until = min(deadline, time.monotonic()+timeout)
        while True:
            if process.poll() is not None:
                raise AssertionError(f'launch exited with {process.returncode}; see launch.log')
            mission = latest.get('mission')
            if mission and mission['state'] in ('FAILED', 'FAULT_BLOCKED', 'CANCELED'):
                raise AssertionError(f'automatic demo failed: {mission}')
            if predicate():
                return
            if time.monotonic() >= until:
                snapshot = {key: latest.get(key) for key in ('navigation', 'mission', 'obstacle', 'pose')}
                raise AssertionError(f'timeout waiting for {description}: {snapshot}')
            executor.spin_once(timeout_sec=.02)

    def moving():
        return (latest.get('navigation', {}).get('state') == 'EXECUTING' and
                latest.get('command', {}).get('valid') and latest['command']['vx'] > .005)

    try:
        context = Context()
        rclpy.init(context=context)
        node = Node('obstacle_demo_auto_check', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        for msg_type, topic, callback, qos in (
                (SegmentStatus, '/navigation/status', on_navigation, durable_qos()),
                (MissionStatus, '/mission/status', on_mission, durable_qos()),
                (ControlCommand, '/navigation/command', on_command, live_qos(30)),
                (ObstacleStatus, '/navigation/obstacle_status', on_obstacle, live_qos(30)),
                (PoseStamped, '/localization/pose', on_pose, live_qos(30)),
                (PathMsg, '/navigation/reference_path', on_path, durable_qos()),
                (MotionStatus, '/motion/status', on_motion, live_qos(30)),
                (MarkerArray, '/simulation/obstacles', on_person, durable_qos()),
                (PointCloud2, '/simulation/points', on_cloud, qos_profile_sensor_data)):
            node.create_subscription(msg_type, topic, callback, qos)
        command = ['ros2', 'launch', 'stair_nav', 'obstacle_demo.launch.py',
                   'rviz:='+('true' if options.rviz else 'false')]
        # Leave autostart and auto_obstacle at their normal launch defaults.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=dict(os.environ), start_new_session=True)
        report['launch_command'] = command
        wait(lambda: moving() and latest.get('pose', {}).get('x', 0.) >= .3 and 'path' in latest,
             20., 'default launch to start navigating without an Action request')
        graph = node.get_node_names()
        assert all(graph.count(name) == 1 for name in ('sim_executor', 'segment_navigation',
                   'task_manager', 'obstacle_monitor', 'demo_obstacles')), graph
        execution_id = latest['navigation']['execution_id']
        task_id = latest['navigation']['task_id']
        path_signature = latest['path']['signature']
        assert execution_id and task_id
        assert latest['mission']['route_id'] == 'flat_obstacle_demo'
        before_obstacle_x = latest['pose']['x']
        wait(lambda: latest.get('person', {}).get('active') and
             latest.get('obstacle', {}).get('valid') and latest['obstacle']['blocked'] and
             latest['obstacle']['occupied_voxels'] >= 3, 12., 'automatic pedestrian point-cloud obstruction')
        person_entered = next(value['t'] for value in samples['person'] if value['active'])
        wait(lambda: latest['navigation']['state'] == 'PAUSED_OBSTACLE' and
             latest.get('motion', {}).get('stable') and latest['motion']['hold_confirmed'],
             4., 'automatic stop and physical hold')
        hold_at = now()
        hold_position = dict(latest['pose'])
        hold_progress = latest['navigation']['progress']
        wait(moving, 12., 'automatic clearance and same-task resume')
        resumed_at = now()
        assert resumed_at-hold_at >= 4., f'pedestrian pause was too short: {resumed_at-hold_at:.3f}s'
        person_left = next(value['t'] for value in samples['person']
                           if value['t'] > person_entered and not value['active'])
        assert person_left-person_entered >= 4.7, 'default pedestrian did not remain for approximately 5 s'
        resumed_command = next(value for value in samples['command'] if value['t'] >= hold_at and
                               value['valid'] and value['vx'] > .005)
        assert resumed_command['t']-person_left >= .85, 'resumed before continuous 1 s clearance'
        assert resumed_command['vx'] <= .08, 'resume bypassed acceleration from zero'
        hold_commands = [value for value in samples['command']
                         if hold_at <= value['t'] < resumed_command['t']]
        assert len(hold_commands) >= 4.*8, 'missing periodic HOLD commands during the pause'
        assert all(value['behavior'] == 'HOLD' and not value['valid'] and
                   value['vx'] == value['wz'] == 0. and value['execution_id'] == execution_id and
                   value['valid_until'] > value['stamp'] for value in hold_commands)
        assert max(b['t']-a['t'] for a, b in zip(hold_commands, hold_commands[1:])) < .35
        held_poses = [value for value in samples['pose']
                      if hold_at <= value['t'] < resumed_command['t']]
        assert len(held_poses) >= 4.*8, 'localization stopped during the obstacle pause'
        drift = max(math.dist((value['x'], value['y'], value['z']),
                              (hold_position['x'], hold_position['y'], hold_position['z']))
                    for value in held_poses)
        assert drift <= .02, f'pose drifted {drift:.4f} m during HOLD'
        held_states = [value for value in samples['navigation']
                       if hold_at <= value['t'] < resumed_command['t']-.1]
        assert held_states and all(value['state'] == 'PAUSED_OBSTACLE' and
                                   abs(value['progress']-hold_progress) <= .005
                                   for value in held_states)
        assert latest['navigation']['execution_id'] == execution_id
        assert latest['navigation']['task_id'] == task_id
        assert latest['path']['signature'] == path_signature
        wait(lambda: latest['mission']['state'] == 'SUCCEEDED' and
             latest['navigation']['state'] == 'SUCCEEDED', 35., 'automatic mission completion')
        assert latest['mission']['completed_segments'] == 1
        assert latest['mission']['navigation_stopped'] and latest['mission']['hold_confirmed']
        assert math.hypot(latest['pose']['x']-5., latest['pose']['y']) <= .15
        assert {value['execution_id'] for value in samples['navigation']
                if value['execution_id']} == {execution_id}
        assert len({value['signature'] for value in samples['path'] if value['points']}) == 1
        client = node.create_client(GetNavigationState, '/navigation/get_state')
        wait(client.service_is_ready, 5., 'control-release service')
        future = client.call_async(GetNavigationState.Request())
        wait(future.done, 5., 'control-release snapshot')
        release = future.result()
        assert not release.busy and release.control_released, release
        assert all(samples.values()), 'one or more evidence streams were not received'
        report.update(passed=True, state='SUCCEEDED', execution_id=execution_id, task_id=task_id,
                      path_signature=path_signature, before_obstacle_x=before_obstacle_x,
                      person_visible_seconds=person_left-person_entered,
                      pause_seconds=resumed_at-hold_at,
                      clear_delay=resumed_command['t']-person_left,
                      hold_messages=len(hold_commands), max_pause_drift_m=drift,
                      final_pose=latest['pose'], release=dict(busy=release.busy,
                                                             control_released=release.control_released))
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        report.update(elapsed_seconds=round(now(), 3), transitions=transitions,
                      topic_counts={key: len(values) for key, values in samples.items()},
                      latest={key: latest.get(key) for key in ('navigation', 'mission', 'obstacle', 'pose')})
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            rclpy.shutdown(context=context)
        if process is not None:
            stop_process_group(process)
        log.close()
        (options.output_dir/'observations.json').write_text(
            json.dumps(samples, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path('/tmp')/time.strftime('stair_obstacle_auto_%Y%m%d_%H%M%S'))
    parser.add_argument('--domain', type=int, default=199)
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--rviz', action='store_true', help='Also launch RViz during automatic verification')
    options = parser.parse_args(args)
    if not 0 <= options.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    options.output_dir = options.output_dir.resolve()
    options.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        report = run(options)
    except Exception:
        report = dict(passed=False, error=traceback.format_exc())
    report_path = options.output_dir/'report.json'
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: report.get(key) for key in ('passed', 'state', 'elapsed_seconds', 'error')},
                     ensure_ascii=False), flush=True)
    print(f'Report: {report_path}', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
