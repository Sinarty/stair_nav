#!/usr/bin/env python3
"""Real ROS mission checks against fresh simulator/navigation/manager processes.

Run from a sourced install. Every scenario uses its own ROS_DOMAIN_ID and all
processes belong to this test; no robot, installed service, or external graph is
controlled. Deterministic acceptance/cancel races live in the core tests.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def run_case(name, route, mission, output_dir, domain):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOG_DIR'] = str(output_dir/'ros_logs')
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import SetBool, Trigger
    from stair_nav_interfaces.action import ExecuteMission, NavigateSegment
    from stair_nav_interfaces.msg import MissionEvent, MissionStatus, SegmentStatus, MotionStatus
    from stair_nav.ros_utils import durable_qos, live_qos
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    processes, logs = {}, []
    env = dict(os.environ)
    def launch(key, executable, extra=()):
        log = (output_dir/f'{name}_{key}.log').open('w')
        logs.append(log)
        cmd = ['ros2', 'run', 'stair_nav', executable, '--ros-args',
               '-p', f'route_file:={route}', *extra]
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)
        processes[key] = proc
        return proc
    def stop(key):
        proc = processes.pop(key, None)
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
    manager_args = ['-p', 'autostart:=false']
    if mission is not None:
        manager_args += ['-p', f'mission_file:={mission}']
    sim_args = ['-p', 'initial_pose:=[-1.0, 0.4, 0.0, 0.6]'] if name.startswith('approach') else []
    if name == 'approach_outside':
        sim_args = ['-p', 'initial_pose:=[-20.0, 0.4, 0.0, 0.6]']
    elif name == 'approach_obstacle':
        sim_args = ['-p', 'initial_pose:=[-1.9, 0.9, 0.0, -1.0]']
    elif name == 'approach_corner':
        sim_args = ['-p', 'initial_pose:=[-2.0, 0.5, 0.0, 0.0]']
    launch('simulator', 'sim_executor', sim_args)
    launch('navigation', 'segment_navigation')
    launch('manager', 'task_manager', manager_args)
    rclpy.init()
    node = Node(f'mission_smoke_{name}')
    latest = {'mission': None, 'navigation': None, 'motion': None}
    nav_history, mission_history, feedback, events = [], [], [], []
    approach_trajectory = []
    def on_navigation(msg):
        latest['navigation'] = msg
        nav_history.append((msg.segment_id, msg.execution_id, msg.state, msg.error_code,
                            msg.geometry_reached))
    def on_mission(msg):
        latest['mission'] = msg
        mission_history.append((msg.task_id, msg.segment_id, msg.state, msg.error_code))
    def on_pose(msg):
        navigation = latest['navigation']
        if navigation is None or navigation.segment_id in ('', 'APPROACH_A'):
            position = msg.pose.position
            approach_trajectory.append((position.x, position.y))
    node.create_subscription(PoseStamped, '/localization/pose', on_pose, live_qos(20))
    node.create_subscription(SegmentStatus, '/navigation/status', on_navigation, durable_qos())
    node.create_subscription(MissionStatus, '/mission/status', on_mission, durable_qos())
    node.create_subscription(MissionEvent, '/mission/events', events.append, QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(MotionStatus, '/motion/status',
                             lambda msg: latest.update(motion=msg), live_qos(5))
    action = ActionClient(node, ExecuteMission, '/execute_mission')
    segment = ActionClient(node, NavigateSegment, '/navigate_segment')
    service_clients = []
    def wait(predicate, timeout=12):
        deadline = time.monotonic()+timeout
        while not predicate():
            for key, proc in processes.items():
                if proc.poll() is not None:
                    raise AssertionError(f'{key} exited; see {name}_{key}.log')
            if time.monotonic() > deadline:
                status = latest['mission']
                nav = latest['navigation']
                raise AssertionError(f'{name} timeout; mission={status}, navigation={nav}')
            rclpy.spin_once(node, timeout_sec=.03)
    def result(future, timeout=12):
        wait(future.done, timeout)
        return future.result()
    def service(path, kind, request):
        client = node.create_client(kind, path)
        service_clients.append(client)
        wait(client.service_is_ready)
        return result(client.call_async(request))
    def fault(key, value=True):
        response = service(f'/sim_executor/{key}', SetBool, SetBool.Request(data=value))
        assert response.success, response.message
    def nav_is(state):
        msg = latest['navigation']
        return msg is not None and msg.state == state
    def goal():
        return result(action.send_goal_async(ExecuteMission.Goal(route_id=route_id),
                      feedback_callback=lambda msg: feedback.append(msg.feedback.status)))
    def completed(handle, timeout=45):
        response = result(handle.get_result_async(), timeout)
        wait(lambda: latest['mission'] is not None and
             latest['mission'].task_id == response.result.task_id and
             latest['mission'].state == response.result.state)
        return response
    def assert_no_later_segment():
        assert not any(sid in ('BC', 'CD') and execution for sid, execution, *_ in nav_history), nav_history
    def assert_events(task_id, final_state):
        wait(lambda: any(message.status.task_id == task_id and
                         message.kind == 'MISSION_STATE' and message.new_state == final_state
                         for message in events))
        current = [message for message in events if message.status.task_id == task_id]
        assert current and len({message.source_instance for message in current}) == 1
        numbers = [message.sequence for message in current]
        assert numbers == list(range(numbers[0], numbers[-1]+1)), numbers
        states = [message.new_state for message in current if message.kind == 'MISSION_STATE']
        assert states[0] == 'VALIDATING', states
        if any(state in ('DISPATCHING', 'RUNNING', 'APPROACHING_START') for state in states):
            assert 'WAITING_READY' in states, states
        assert states[-1] == final_state, states
        for message in current:
            if message.kind == 'MISSION_STATE':
                assert message.new_state == message.status.state, message
            elif message.kind == 'SEGMENT_STATE':
                assert message.new_state == message.status.segment_state, message
            else:
                raise AssertionError(f'unknown event kind {message.kind}')
        return current
    started = time.monotonic()
    try:
        wait(lambda: action.server_is_ready() and segment.server_is_ready() and
             latest['mission'] is not None and latest['navigation'] is not None and
             latest['motion'] is not None and all(
                 node_name in node.get_node_names() for node_name in
                 ('task_manager', 'segment_navigation', 'sim_executor')), timeout=20)
        route_id = latest['mission'].route_id
        assert route_id, 'mission status must expose configured route_id even while idle'
        assert latest['mission'].state == 'IDLE'
        # Exactly one scheduler and one current execution chain in this domain.
        graph = node.get_node_names()
        for node_name in ('task_manager', 'segment_navigation', 'sim_executor'):
            assert graph.count(node_name) == 1, graph
        if name == 'ready_timeout':
            fault('not_ready')
            wait(lambda: latest['motion'] is not None and not latest['motion'].ready)
        handle = goal()
        assert handle.accepted, 'initial mission goal rejected'
        if name == 'early_cancel':
            canceled = result(handle.cancel_goal_async())
            assert canceled.goals_canceling, 'mission early cancellation rejected'
        elif name in ('normal', 'approach', 'approach_obstacle', 'approach_corner'):
            second = goal()
            assert not second.accepted, 'busy manager accepted another mission'
            response = completed(handle, timeout=90)
            assert response.status == GoalStatus.STATUS_SUCCEEDED, response
            assert response.result.success and response.result.state == 'SUCCEEDED'
            assert response.result.navigation_stopped and response.result.hold_confirmed
            executions = []
            for sid, execution, state, *_ in nav_history:
                if execution and state not in ('IDLE',) and (sid, execution) not in executions:
                    executions.append((sid, execution))
            expected = ['AB', 'BC', 'CD'] if name == 'normal' else ['APPROACH_A', 'AB', 'BC', 'CD']
            assert [sid for sid, _ in executions] == expected, executions
            assert len({execution for _, execution in executions}) == len(expected)
            assert latest['mission'].completed_segments == 3
            assert latest['mission'].segment_index == 3 and latest['mission'].segment_count == 3
            assert feedback and all(msg.task_id == response.result.task_id for msg in feedback)
            current_events = assert_events(response.result.task_id, 'SUCCEEDED')
            approaching = [message for message in current_events
                           if message.kind == 'MISSION_STATE' and message.new_state == 'APPROACHING_START']
            if name != 'normal':
                assert approaching, 'missing approach transition'
                assert all(message.status.segment_id == 'APPROACH_A' and
                           message.status.segment_index == 0 and
                           message.status.segment_count == 3 and
                           message.status.completed_segments == 0 for message in approaching)
            else:
                assert not approaching, 'already at A but approach was executed'
            if name in ('approach_obstacle', 'approach_corner'):
                import yaml
                from stair_nav.core.config import load_route, segment_config
                from stair_nav.core.planning import OccupancyGrid
                route_config = load_route(route)
                segment_configured = segment_config(route_config, 'APPROACH_A')
                grid_file = Path(route_config['_base_dir'])/segment_configured['grid_file']
                grid = OccupancyGrid.from_mapping(yaml.safe_load(grid_file.read_text()),
                                                  segment_configured['planning'])
                assert len(approach_trajectory) > 50, 'missing approach localization samples'
                assert all(grid.line_is_free(a, b) for a, b in
                           zip(approach_trajectory, approach_trajectory[1:])), 'approach clearance violated'
            # A newly created durable subscriber receives the current terminal snapshot.
            late = []
            node.create_subscription(MissionStatus, '/mission/status', late.append, durable_qos())
            wait(lambda: bool(late))
            assert late[-1].state == 'SUCCEEDED' and late[-1].task_id == response.result.task_id
            return dict(case=name, passed=True, segments=executions,
                        busy_rejected=True, late_monitor_received=True,
                        ordered_events=len(current_events),
                        checked_approach_edges=len(approach_trajectory)-1,
                        elapsed_seconds=round(time.monotonic()-started, 3))
        elif name == 'approach_cancel':
            wait(lambda: latest['mission'] is not None and
                 latest['mission'].state == 'APPROACHING_START' and
                 latest['navigation'] is not None and
                 latest['navigation'].segment_id == 'APPROACH_A' and nav_is('EXECUTING'))
            canceled = result(handle.cancel_goal_async())
            assert canceled.goals_canceling, 'approach cancellation rejected'
        elif name in ('cancel', 'localization_stale', 'transport_timeout', 'restart_no_resume'):
            wait(lambda: nav_is('EXECUTING'))
            if name == 'cancel':
                canceled = result(handle.cancel_goal_async())
                assert canceled.goals_canceling, 'running mission cancellation rejected'
            elif name == 'localization_stale':
                fault('localization_dropout')
            elif name == 'transport_timeout':
                # Kill only the process group launched by this scenario.
                proc = processes.pop('navigation')
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            else:
                old_execution = latest['navigation'].execution_id
                manager = processes.pop('manager')
                os.killpg(manager.pid, signal.SIGKILL)
                manager.wait(timeout=5)
                latest['mission'] = None
                launch('manager', 'task_manager', manager_args)
                wait(lambda: latest['mission'] is not None and latest['mission'].state == 'IDLE')
                assert not latest['mission'].task_id
                # Keep spinning so a delayed autostart would be visible.
                until = time.monotonic()+1.0
                wait(lambda: time.monotonic() >= until)
                assert latest['mission'].state == 'IDLE' and not latest['mission'].task_id
                assert latest['navigation'].execution_id == old_execution
                assert_no_later_segment()
                return dict(case=name, passed=True, old_execution=old_execution,
                            restarted_state='IDLE', automatic_resume=False,
                            elapsed_seconds=round(time.monotonic()-started, 3))
        elif name == 'missing_exit':
            # Start fault before the AB geometric candidate can be reached.
            fault('missing_exit')
        elif name not in ('ready_timeout', 'approach_outside'):
            raise ValueError(f'unknown scenario {name}')
        response = completed(handle, timeout=45)
        final = response.result
        assert not final.success, final
        assert_no_later_segment()
        if name.startswith('approach'):
            assert not any(sid in ('AB', 'BC', 'CD') and execution
                           for sid, execution, *_ in nav_history), nav_history
            assert latest['mission'].completed_segments == 0
        current_events = assert_events(final.task_id, final.state)
        if name in ('cancel', 'early_cancel', 'approach_cancel'):
            assert response.status == GoalStatus.STATUS_CANCELED, response
            assert final.state == 'CANCELED', final
            assert final.navigation_stopped
            if any(execution for _, execution, *_ in nav_history):
                assert final.hold_confirmed, 'task cancel completed before fresh adapter cleanup'
        elif name == 'transport_timeout':
            assert response.status == GoalStatus.STATUS_ABORTED
            assert final.state == 'FAULT_BLOCKED' and latest['mission'].blocked, final
            assert not goal().accepted, 'unknown downstream state allowed a fresh mission'
            reset = service('/task_manager/reset', Trigger, Trigger.Request())
            assert not reset.success, 'unresolved downstream goal allowed reset'
        else:
            assert response.status == GoalStatus.STATUS_ABORTED, response
            assert final.state == 'FAILED', final
            if name == 'ready_timeout':
                assert final.error_code == 'READY_TIMEOUT', final
                assert not any(execution for _, execution, *_ in nav_history), nav_history
            elif name == 'localization_stale':
                assert final.error_code == 'POSE_STALE', final
            elif name == 'missing_exit':
                assert final.error_code in ('ARRIVAL_CONFIRM_TIMEOUT', 'CONFIRMATION_TIMEOUT'), final
                assert any(geometry for _, _, _, _, geometry in nav_history)
            elif name == 'approach_outside':
                assert final.error_code == 'INVALID_START', final
        return dict(case=name, passed=True, state=final.state, error_code=final.error_code,
                    navigation_stopped=final.navigation_stopped,
                    hold_confirmed=final.hold_confirmed,
                    ordered_events=len(current_events),
                    elapsed_seconds=round(time.monotonic()-started, 3))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for key in list(processes):
            stop(key)
        for log in logs:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path,
                        default=Path(__file__).resolve().parents[1]/'stair_nav/config/demo_route.yaml')
    parser.add_argument('--mission', type=Path)
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/stair_nav_mission_smoke'))
    parser.add_argument('--domain-base', type=int, default=151)
    parser.add_argument('--cases', default='normal,approach,approach_obstacle,approach_corner,approach_cancel,approach_outside,cancel,early_cancel,ready_timeout,localization_stale,missing_exit,transport_timeout,restart_no_resume')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    outcomes = []
    for i, name in enumerate(args.cases.split(',')):
        try:
            outcome = run_case(name, args.route.resolve(),
                               args.mission.resolve() if args.mission else None,
                               args.output_dir.resolve(), args.domain_base+i)
        except Exception as exc:
            outcome = dict(case=name, passed=False, error=f'{type(exc).__name__}: {exc}')
        outcomes.append(outcome)
        print(json.dumps(outcome, ensure_ascii=False), flush=True)
        (args.output_dir/'results.json').write_text(json.dumps(outcomes, ensure_ascii=False, indent=2))
        if not outcome['passed']:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
