#!/usr/bin/env python3
"""Exercise flat point-cloud stop/resume through fresh, isolated ROS processes.

Run from a sourced build containing obstacle_demo.launch.py. The script owns its
launch process groups, uses localhost-only domains, and never controls another
running graph. Two scenarios prove automatic resume, loss-of-cloud holding, and
cancellation while paused. JSON summaries and topic JSONL traces are retained.
"""
import argparse
import hashlib
from itertools import islice
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def stop_process_group(process):
    """Reap only the process group created by this check, including on failure."""
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        try:
            if sig == signal.SIGINT:
                process.send_signal(sig)  # launch forwards it to its children once
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
    if process.poll() is None:
        process.wait(timeout=2)


def run_case(name, output_dir, domain, deadline):
    case_dir = output_dir/name
    case_dir.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(case_dir/'ros_logs')

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMsg
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_srvs.srv import SetBool
    from visualization_msgs.msg import Marker, MarkerArray
    from stair_nav_interfaces.action import ExecuteMission
    from stair_nav_interfaces.msg import (
        ControlCommand, MissionStatus, MotionStatus, ObstacleStatus, SegmentStatus,
    )
    from stair_nav_interfaces.srv import GetNavigationState
    from stair_nav.ros_utils import durable_qos, live_qos, seconds

    started = time.monotonic()
    latest = {}
    history = {key: [] for key in ('navigation', 'mission', 'motion', 'command',
                                   'obstacle', 'pose', 'path', 'cloud', 'person', 'zone')}
    transitions, checkpoints = [], []
    result = dict(case=name, domain=domain, passed=False,
                  launch_log=str(case_dir/'launch.log'), trace=str(case_dir/'topics.jsonl'))
    context, executor, node, action, process = None, None, None, None, None
    log = (case_dir/'launch.log').open('w', encoding='utf-8')
    trace = (case_dir/'topics.jsonl').open('w', encoding='utf-8')

    def elapsed():
        return time.monotonic()-started

    def record(key, topic, data):
        entry = dict(t=round(elapsed(), 6), **data)
        history[key].append(entry)
        latest[key] = entry
        trace.write(json.dumps(dict(topic=topic, **entry), ensure_ascii=False,
                               allow_nan=False)+'\n')
        return entry

    def on_navigation(msg):
        previous = latest.get('navigation')
        value = record('navigation', '/navigation/status', dict(
            stamp=seconds(msg.header.stamp), state=msg.state, phase=msg.phase,
            path_stamp=seconds(msg.path_stamp),
            task_id=msg.task_id, execution_id=msg.execution_id, segment_id=msg.segment_id,
            progress=msg.progress, navigation_stopped=msg.navigation_stopped,
            hold_confirmed=msg.hold_confirmed, error_code=msg.error_code,
            error_message=msg.error_message))
        if previous is None or (previous['state'], previous['phase']) != (msg.state, msg.phase):
            transitions.append(value)

    def on_mission(msg):
        record('mission', '/mission/status', dict(
            stamp=seconds(msg.header.stamp), state=msg.state, phase=msg.phase,
            route_id=msg.route_id, task_id=msg.task_id, execution_id=msg.execution_id,
            segment_id=msg.segment_id, segment_state=msg.segment_state,
            completed_segments=msg.completed_segments, segment_progress=msg.segment_progress,
            error_code=msg.error_code, error_message=msg.error_message,
            blocked=msg.blocked))

    def on_motion(msg):
        record('motion', '/motion/status', dict(
            stamp=seconds(msg.header.stamp), execution_id=msg.execution_id,
            ready=msg.ready, stable=msg.stable, hold_confirmed=msg.hold_confirmed,
            fault_code=msg.fault_code))

    def on_command(msg):
        record('command', '/navigation/command', dict(
            stamp=seconds(msg.header.stamp), valid_until=seconds(msg.valid_until),
            execution_id=msg.execution_id, task_id=msg.task_id, segment_id=msg.segment_id,
            behavior=msg.behavior, valid=msg.valid, vx=msg.vx, wz=msg.wz))

    def on_obstacle(msg):
        record('obstacle', '/navigation/obstacle_status', dict(
            stamp=seconds(msg.header.stamp), valid_until=seconds(msg.valid_until),
            path_stamp=seconds(msg.path_stamp),
            execution_id=msg.execution_id, valid=msg.valid, blocked=msg.blocked,
            reason=msg.reason, nearest_distance=finite(msg.nearest_distance),
            occupied_voxels=msg.occupied_voxels))

    def on_pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        record('pose', '/localization/pose', dict(
            stamp=seconds(msg.header.stamp), frame=msg.header.frame_id,
            x=p.x, y=p.y, z=p.z, yaw=yaw))

    def on_path(msg):
        points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        signature = hashlib.sha256(json.dumps(points, separators=(',', ':')).encode()).hexdigest()
        record('path', '/navigation/reference_path', dict(
            stamp=seconds(msg.header.stamp), frame=msg.header.frame_id,
            count=len(points), signature=signature, points=points))

    def on_cloud(msg):
        # Retain actual wire metadata and a bounded XYZ sample, not binary data.
        sample = [[finite(v) for v in row] for row in islice(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=False), 8)]
        record('cloud', '/simulation/points', dict(
            stamp=seconds(msg.header.stamp), frame=msg.header.frame_id,
            count=msg.width*msg.height, point_step=msg.point_step,
            fields=[field.name for field in msg.fields], sample=sample))

    def on_markers(key, topic, msg):
        record(key, topic, dict(markers=[dict(
            stamp=seconds(m.header.stamp), frame=m.header.frame_id,
            ns=m.ns, id=m.id, type=m.type, action=m.action, text=m.text,
            position=[m.pose.position.x, m.pose.position.y, m.pose.position.z],
            scale=[m.scale.x, m.scale.y, m.scale.z],
            points=[[p.x, p.y, p.z] for p in m.points]) for m in msg.markers]))

    def check_live():
        if time.monotonic() >= deadline:
            raise AssertionError('overall check deadline exceeded')
        if process.poll() is not None:
            raise AssertionError(f'launch exited with {process.returncode}; see {case_dir / "launch.log"}')
        mission = latest.get('mission')
        if mission and mission['state'] in ('FAILED', 'FAULT_BLOCKED'):
            raise AssertionError(f'mission failed: {mission}')

    def wait(predicate, timeout=10., description='condition'):
        until = min(deadline, time.monotonic()+timeout)
        while True:
            check_live()
            if predicate():
                return
            if time.monotonic() >= until:
                snapshot = {key: latest.get(key) for key in
                            ('navigation', 'mission', 'motion', 'command', 'obstacle', 'pose')}
                raise AssertionError(f'timeout waiting for {description}: {snapshot}')
            executor.spin_once(timeout_sec=.02)

    def spin_for(duration):
        until = time.monotonic()+duration
        wait(lambda: time.monotonic() >= until, duration+1., 'observation interval')

    def future_value(future, timeout=10., description='ROS future'):
        wait(future.done, timeout, description)
        return future.result()

    def set_value(client, value):
        response = future_value(client.call_async(SetBool.Request(data=value)), description='SetBool response')
        assert response.success, response.message
        checkpoints.append(dict(event=f'{client.srv_name}={value}', t=round(elapsed(), 6)))
        return elapsed()

    def paused():
        return latest.get('navigation', {}).get('state') == 'PAUSED_OBSTACLE'

    def moving():
        nav, command = latest.get('navigation', {}), latest.get('command', {})
        return (nav.get('state') == 'EXECUTING' and command.get('valid') and
                command.get('behavior') == 'TRACK' and command.get('vx', 0.) > .005)

    def stationary_pause(execution_id):
        wait(lambda: paused() and latest.get('motion', {}).get('execution_id') == execution_id and
             latest['motion']['stable'] and latest['motion']['hold_confirmed'] and
             latest.get('command', {}).get('behavior') == 'HOLD',
             description='paused and physically held')
        assert latest['navigation']['navigation_stopped']

    def hold_for(duration, execution_id, task_id, path_signature, result_future):
        stationary_pause(execution_id)
        initial = dict(latest['pose'])
        progress = latest['navigation']['progress']
        since = elapsed()
        until = time.monotonic()+duration
        while time.monotonic() < until:
            check_live()
            assert not result_future.done(), 'paused goal returned a terminal result'
            assert paused(), f'navigation resumed unexpectedly: {latest.get("navigation")}'
            assert latest['navigation']['execution_id'] == execution_id
            assert latest['navigation']['task_id'] == task_id
            assert abs(latest['navigation']['progress']-progress) <= .005
            assert latest['path']['signature'] == path_signature
            assert latest['mission']['state'] == 'RUNNING' and not latest['mission']['blocked']
            assert latest['mission']['completed_segments'] == 0
            executor.spin_once(timeout_sec=.02)
        commands = [c for c in history['command'] if c['t'] >= since]
        positions = [p for p in history['pose'] if p['t'] >= since]
        assert len(commands) >= duration*8, 'HOLD heartbeat did not continue during pause'
        assert len(positions) >= duration*8, 'localization heartbeat did not continue during pause'
        assert all(c['behavior'] == 'HOLD' and not c['valid'] and c['vx'] == c['wz'] == 0.
                   and c['execution_id'] == execution_id and c['valid_until'] > c['stamp']
                   for c in commands), commands
        gaps = [b['t']-a['t'] for a, b in zip(commands, commands[1:])]
        assert max(gaps, default=0.) < .35, f'HOLD heartbeat gap: {max(gaps)}'
        drift = max((math.dist((p['x'], p['y'], p['z']),
                              (initial['x'], initial['y'], initial['z'])) for p in positions), default=0.)
        assert drift <= .02, f'robot moved {drift:.4f} m while paused'
        checkpoint = dict(event='hold_verified', t=round(elapsed(), 6),
                          duration=duration, hold_messages=len(commands),
                          max_drift_m=drift, progress=progress,
                          position=[initial['x'], initial['y'], initial['z']])
        checkpoints.append(checkpoint)
        return checkpoint

    def resume_after(clear_returned_at, execution_id, path_signature):
        wait(moving, timeout=8., description='same goal to resume after continuous clear')
        command = next(value for value in history['command'] if value['t'] >= clear_returned_at and
                       value['valid'] and value['behavior'] == 'TRACK' and value['vx'] > .005)
        delay = command['t']-clear_returned_at
        assert delay >= .85, f'resumed before continuous 1 s clear window: {delay:.3f}s'
        assert command['vx'] <= .08, f'resume skipped acceleration from zero: {command}'
        assert command['execution_id'] == execution_id
        assert latest['path']['signature'] == path_signature
        assert latest['obstacle']['valid'] and not latest['obstacle']['blocked']
        assert latest['obstacle']['execution_id'] == execution_id
        checkpoints.append(dict(event='same_goal_resumed', t=command['t'], clear_delay=delay,
                                execution_id=execution_id, position=[latest['pose']['x'], latest['pose']['y']]))
        return delay

    def assert_release():
        client = node.create_client(GetNavigationState, '/navigation/get_state')
        try:
            wait(client.service_is_ready, description='navigation state service')
            state = None
            until = time.monotonic()+6.
            while time.monotonic() < until:
                state = future_value(client.call_async(GetNavigationState.Request()))
                if not state.busy and state.control_released:
                    return dict(busy=state.busy, control_released=state.control_released)
                spin_for(.1)
            raise AssertionError(f'navigation failed to release: {state}')
        finally:
            node.destroy_client(client)

    try:
        process = subprocess.Popen(
            ['ros2', 'launch', 'stair_nav', 'obstacle_demo.launch.py',
             'rviz:=false', 'autostart:=false', 'auto_obstacle:=false'],
            stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), start_new_session=True)
        context = Context()
        rclpy.init(context=context)
        node = Node(f'obstacle_stop_check_{name}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        for key, msg_type, topic, callback, qos in (
                ('navigation', SegmentStatus, '/navigation/status', on_navigation, durable_qos()),
                ('mission', MissionStatus, '/mission/status', on_mission, durable_qos()),
                ('motion', MotionStatus, '/motion/status', on_motion, live_qos(20)),
                ('command', ControlCommand, '/navigation/command', on_command, live_qos(30)),
                ('obstacle', ObstacleStatus, '/navigation/obstacle_status', on_obstacle, live_qos(20)),
                ('pose', PoseStamped, '/localization/pose', on_pose, live_qos(30)),
                ('path', PathMsg, '/navigation/reference_path', on_path, durable_qos()),
                ('cloud', PointCloud2, '/simulation/points', on_cloud, qos_profile_sensor_data)):
            node.create_subscription(msg_type, topic, callback, qos)
        node.create_subscription(MarkerArray, '/simulation/obstacles',
                                 lambda msg: on_markers('person', '/simulation/obstacles', msg), durable_qos())
        node.create_subscription(MarkerArray, '/navigation/obstacle_zone',
                                 lambda msg: on_markers('zone', '/navigation/obstacle_zone', msg), durable_qos())
        action = ActionClient(node, ExecuteMission, '/execute_mission')
        obstacle_client = node.create_client(SetBool, '/demo_obstacles/set_obstacle')
        cloud_client = node.create_client(SetBool, '/demo_obstacles/set_cloud_enabled')
        expected_nodes = ('sim_executor', 'segment_navigation', 'task_manager',
                          'obstacle_monitor', 'demo_obstacles')
        wait(lambda: action.server_is_ready() and obstacle_client.service_is_ready() and
             cloud_client.service_is_ready() and all(key in latest for key in
             ('mission', 'navigation', 'motion', 'pose', 'cloud', 'person', 'zone')) and
             all(node_name in node.get_node_names() for node_name in expected_nodes),
             timeout=25., description='isolated demo graph and sensor streams')
        graph = node.get_node_names()
        assert all(graph.count(node_name) == 1 for node_name in expected_nodes), graph
        assert latest['mission']['state'] == 'IDLE'
        assert latest['mission']['route_id'] == 'flat_obstacle_demo'
        handle = future_value(action.send_goal_async(ExecuteMission.Goal(route_id='flat_obstacle_demo')),
                              description='mission acceptance')
        assert handle.accepted, 'demo mission goal rejected'
        result_future = handle.get_result_async()
        wait(lambda: moving() and 'path' in latest and latest['pose']['x'] >= .45,
             timeout=15., description='robot to travel before obstacle insertion')
        execution_id, task_id = latest['navigation']['execution_id'], latest['navigation']['task_id']
        path_signature = latest['path']['signature']
        assert execution_id and task_id and latest['path']['count'] >= 2
        result.update(execution_id=execution_id, task_id=task_id, path_signature=path_signature)

        inserted_at = set_value(obstacle_client, True)
        wait(lambda: latest.get('obstacle', {}).get('valid') and
             latest['obstacle']['blocked'] and latest['obstacle']['occupied_voxels'] > 0,
             description='person point cloud to trigger obstacle detector')
        stationary_pause(execution_id)
        detection_delay = latest['navigation']['t']-inserted_at
        assert detection_delay < 1., f'person stop detection delayed by {detection_delay:.3f}s'
        assert any(m['action'] == Marker.ADD for value in history['person'] if value['t'] >= inserted_at
                   for m in value['markers']), 'person visualization was not published'
        first_hold = hold_for(1.2, execution_id, task_id, path_signature, result_future)

        if name == 'cancel_paused':
            canceled = future_value(handle.cancel_goal_async(), description='paused mission cancellation')
            assert canceled.goals_canceling, 'cancellation while paused was rejected'
            response = future_value(result_future, timeout=15., description='canceled mission result')
            assert response.status == GoalStatus.STATUS_CANCELED, response
            assert response.result.state == 'CANCELED' and not response.result.success
            assert response.result.navigation_stopped and response.result.hold_confirmed
            result.update(state=response.result.state, first_hold=first_hold,
                          release=assert_release(), detection_delay=detection_delay)
        else:
            cleared_at = set_value(obstacle_client, False)
            first_delay = resume_after(cleared_at, execution_id, path_signature)
            # Move again before testing an independent interruption of the same goal.
            resumed_x = latest['pose']['x']
            wait(lambda: moving() and latest['pose']['x'] >= resumed_x+.2,
                 description='forward progress after first resume')
            set_value(obstacle_client, True)
            wait(lambda: latest.get('obstacle', {}).get('valid') and
                 latest['obstacle']['blocked'] and latest['obstacle']['occupied_voxels'] > 0,
                 description='second obstacle detection')
            stationary_pause(execution_id)
            set_value(cloud_client, False)
            set_value(obstacle_client, False)
            wait(lambda: not latest['obstacle']['valid'] and latest['obstacle']['blocked'],
                 timeout=5., description='cloud-loss invalid sensing')
            # Drain in-flight frames before asserting that no sensor data arrives.
            spin_for(.2)
            clouds_before = len(history['cloud'])
            lost_cloud_hold = hold_for(1.3, execution_id, task_id, path_signature, result_future)
            assert len(history['cloud']) == clouds_before, 'cloud disable service did not stop cloud output'
            assert not latest['obstacle']['valid'], 'missing sensor data was reported as valid clearance'
            cloud_restarted_at = set_value(cloud_client, True)
            second_delay = resume_after(cloud_restarted_at, execution_id, path_signature)
            response = future_value(result_future, timeout=45., description='resumed mission success')
            assert response.status == GoalStatus.STATUS_SUCCEEDED, response
            assert response.result.state == 'SUCCEEDED' and response.result.success
            assert response.result.navigation_stopped and response.result.hold_confirmed
            wait(lambda: latest['mission']['state'] == 'SUCCEEDED', description='terminal mission snapshot')
            assert latest['mission']['completed_segments'] == 1
            assert math.hypot(latest['pose']['x']-5., latest['pose']['y']) <= .15
            result.update(state=response.result.state, first_hold=first_hold,
                          lost_cloud_hold=lost_cloud_hold, clear_delays=[first_delay, second_delay],
                          release=assert_release(), detection_delay=detection_delay)

        executions = {value['execution_id'] for value in history['navigation'] if value['execution_id']}
        assert executions == {execution_id}, f'interruption created another execution: {executions}'
        assert len({value['signature'] for value in history['path'] if value['count']}) == 1, 'interruption changed the path'
        assert all(history[key] for key in history), 'missing topic evidence'
        result['passed'] = True
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        result.update(elapsed_seconds=round(elapsed(), 3),
                      topic_counts={key: len(values) for key, values in history.items()},
                      checkpoints=checkpoints, transitions=transitions,
                      final_pose=latest.get('pose'),
                      latest={key: latest.get(key) for key in
                              ('navigation', 'mission', 'motion', 'command', 'obstacle')})
        if action is not None:
            action.destroy()
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            rclpy.shutdown(context=context)
        if process is not None:
            stop_process_group(process)
        trace.close()
        log.close()
        (case_dir/'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False,
                                                      allow_nan=False)+'\n', encoding='utf-8')
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path('/tmp')/time.strftime('stair_obstacle_check_%Y%m%d_%H%M%S'))
    parser.add_argument('--domain', type=int, default=197,
                        help='First of two isolated ROS domains (default: 197, 198)')
    parser.add_argument('--timeout', type=float, default=120., help='Overall wall-clock deadline in seconds')
    options = parser.parse_args(args)
    if not 0 <= options.domain < 232:
        parser.error('--domain must be between 0 and 231 so both case domains fit')
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    output_dir = options.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic()+options.timeout
    cases = []
    for index, name in enumerate(('resume_cloud_loss', 'cancel_paused')):
        try:
            case = run_case(name, output_dir, options.domain+index, deadline)
        except Exception:
            # Import/setup failures should still leave a machine-readable top
            # level result rather than losing the report completely.
            case = dict(case=name, domain=options.domain+index, passed=False,
                        error=traceback.format_exc())
        cases.append(case)
        print(json.dumps({key: cases[-1].get(key) for key in
                          ('case', 'passed', 'state', 'elapsed_seconds', 'error')}, ensure_ascii=False), flush=True)
        if time.monotonic() >= deadline:
            break
    report = dict(passed=len(cases) == 2 and all(case['passed'] for case in cases),
                  output_dir=str(output_dir), cases=cases)
    (output_dir/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                                   allow_nan=False)+'\n', encoding='utf-8')
    print(f'Report: {output_dir / "report.json"}', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
