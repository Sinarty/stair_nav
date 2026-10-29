#!/usr/bin/env python3
"""Run real ROS Action/topic/service checks against fresh synthetic demo processes.

Run after sourcing the built install. Each case starts a new simulator at A; no
case teleports an active robot. Results/logs are written under --output-dir.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def run_case(name, route, output_dir, domain):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ.setdefault('ROS_LOG_DIR', str(output_dir/'ros_logs'))
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Trigger, SetBool
    from stair_nav_interfaces.action import NavigateSegment
    from stair_nav_interfaces.msg import SegmentStatus, ControlCommand, MotionStatus, ChannelEvent
    from stair_nav.ros_utils import durable_qos, live_qos

    log_file = output_dir/f'{name}.log'
    env = dict(os.environ)
    with log_file.open('w') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'stair_nav', 'demo.launch.py', 'rviz:=false',
             'autostart:=false', f'route_file:={route}'],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        rclpy.init()
        node = Node(f'ros_smoke_{name}')
        latest = {'task': None, 'navigation': None, 'command': None, 'pose': None}
        history = []
        def navigation(msg):
            latest['navigation'] = msg
            history.append((msg.segment_id, msg.state, msg.error_code, msg.geometry_reached))
        node.create_subscription(SegmentStatus, '/navigation/status', navigation, durable_qos())
        node.create_subscription(SegmentStatus, '/task/status',
                                 lambda msg: latest.update(task=msg), durable_qos())
        node.create_subscription(ControlCommand, '/navigation/command',
                                 lambda msg: latest.update(command=msg), live_qos())
        node.create_subscription(PoseStamped, '/localization/pose',
                                 lambda msg: latest.update(pose=msg), live_qos(5))
        action = ActionClient(node, NavigateSegment, '/navigate_segment')
        old_motion = node.create_publisher(MotionStatus, '/motion/status', live_qos(5))
        blocked = node.create_publisher(ChannelEvent, '/navigation/channel_event', live_qos(5))
        service_clients = []
        def wait(predicate, timeout=8.0):
            deadline = time.monotonic()+timeout
            while not predicate():
                if proc.poll() is not None:
                    raise AssertionError(f'demo exited: see {log_file}')
                if time.monotonic() > deadline:
                    state = latest['navigation']
                    raise AssertionError(f'timeout in {name}: {state.state if state else "no status"}; see {log_file}')
                rclpy.spin_once(node, timeout_sec=0.03)
        def future_result(future, timeout=8.0):
            wait(future.done, timeout)
            return future.result()
        def service(path, srv_type, request):
            client = node.create_client(srv_type, path)
            service_clients.append(client)
            wait(client.service_is_ready)
            response = future_result(client.call_async(request))
            assert response.success, response.message
            return response
        def fault(key, value):
            return service(f'/sim_executor/{key}', SetBool, SetBool.Request(data=value))
        def nav_is(state):
            return latest['navigation'] is not None and latest['navigation'].state == state
        started = time.monotonic()
        try:
            wait(lambda: action.server_is_ready() and latest['pose'] is not None and
                 latest['navigation'] is not None, timeout=15.0)
            if name in ('normal', 'transport_timeout'):
                service('/task_manager/start', Trigger, Trigger.Request())
                if name == 'transport_timeout':
                    wait(lambda: nav_is('EXECUTING'))
                    children = subprocess.check_output(
                        ['ps', '-o', 'pid=,args=', '--ppid', str(proc.pid)], text=True)
                    target_pids = [int(line.split()[0]) for line in children.splitlines()
                                   if 'segment_navigation' in line]
                    assert len(target_pids) == 1, children
                    os.kill(target_pids[0], signal.SIGKILL)
                    wait(lambda: latest['task'] is not None and
                         latest['task'].state == 'FAULT_BLOCKED', timeout=20.0)
                    assert not latest['task'].navigation_stopped
                    assert not latest['task'].hold_confirmed
                    client = node.create_client(Trigger, '/task_manager/start')
                    service_clients.append(client)
                    wait(client.service_is_ready)
                    retry = future_result(client.call_async(Trigger.Request()))
                    assert not retry.success and 'BUSY' in retry.message
                    return {'case': name, 'passed': True,
                            'elapsed_seconds': round(time.monotonic()-started, 3),
                            'expected_error': latest['task'].error_code,
                            'outcome_unknown_and_new_task_rejected': True}
                wait(lambda: latest['task'] is not None and latest['task'].task_id and
                     latest['task'].state in ('SUCCEEDED','FAILED','CANCELED'), timeout=80.0)
                assert latest['task'].state == 'SUCCEEDED', latest['task'].error_message
                executed = {sid for sid, state, _, _ in history if state == 'EXECUTING'}
                assert {'AB','BC','CD'} <= executed, executed
                assert latest['task'].segment_id == 'CD'
                details = {'segments_executed': sorted(executed),
                           'final_task_and_segment': 'CD/SUCCEEDED'}
            else:
                if name == 'ready_timeout':
                    fault('not_ready', True)
                if name == 'missing_exit':
                    fault('missing_exit', True)
                execution_id = str(uuid.uuid4())
                goal = NavigateSegment.Goal(task_id=f'smoke-{name}',
                                            execution_id=execution_id, segment_id='AB')
                handle = future_result(action.send_goal_async(goal))
                assert handle.accepted, 'valid goal rejected'
                result_future = handle.get_result_async()
                if name == 'immediate_cancel':
                    cancel = future_result(handle.cancel_goal_async())
                    assert cancel.goals_canceling, 'immediate cancel rejected'
                if name in ('localization_stale','cancel','channel_blocked'):
                    wait(lambda: nav_is('EXECUTING'))
                    if name == 'localization_stale':
                        fault('localization_dropout', True)
                    elif name == 'channel_blocked':
                        event = ChannelEvent(segment_id='AB', blocked=True, reason='smoke blockage')
                        event.header.stamp = node.get_clock().now().to_msg()
                        event.header.frame_id = 'map'
                        blocked.publish(event)
                    else:
                        # An old execution's fault and confirmations must be ignored.
                        old = MotionStatus(execution_id='retired-id', segment_id='AB',
                                           fault_code='LATE_FAULT', fault_message='must be ignored',
                                           ready=True, exit_confirmed=True, stable=True)
                        old.header.stamp = node.get_clock().now().to_msg()
                        old.header.frame_id = 'map'
                        old_motion.publish(old)
                        until = time.monotonic()+0.25
                        wait(lambda: time.monotonic() >= until)
                        assert nav_is('EXECUTING'), 'old motion packet affected current execution'
                        cancel = future_result(handle.cancel_goal_async())
                        assert cancel.goals_canceling, 'cancel rejected'
                response = future_result(result_future, timeout=25.0)
                result = response.result
                expected = dict(ready_timeout='READY_TIMEOUT', localization_stale='POSE_STALE',
                                cancel='CANCELED', missing_exit='ARRIVAL_CONFIRM_TIMEOUT',
                                channel_blocked='CHANNEL_BLOCKED', immediate_cancel='CANCELED')[name]
                assert not result.success and result.error_code == expected, (
                    result.success, result.error_code, result.error_message)
                assert result.navigation_stopped
                wait(lambda: latest['command'] is not None and not latest['command'].valid)
                assert latest['command'].behavior == 'HOLD'
                if name == 'missing_exit':
                    assert any(item[3] for item in history), 'never reached geometric candidate'
                    assert not any(state == 'SUCCEEDED' for _, state, _, _ in history)
                details = {'expected_error': expected, 'hold_confirmed_at_result': result.hold_confirmed}
            return {'case': name, 'passed': True,
                    'elapsed_seconds': round(time.monotonic()-started, 3), **details}
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path,
                        default=Path(__file__).resolve().parents[1]/'stair_nav/config/demo_route.yaml')
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/stair_nav_ros_smoke'))
    parser.add_argument('--cases', default='normal,ready_timeout,localization_stale,cancel,missing_exit,channel_blocked')
    parser.add_argument('--domain-base', type=int, default=91)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for i, name in enumerate(args.cases.split(',')):
        try:
            result = run_case(name, args.route.resolve(), args.output_dir.resolve(), args.domain_base+i)
        except Exception as exc:
            result = {'case': name, 'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        (args.output_dir/'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
        if not result['passed']:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
