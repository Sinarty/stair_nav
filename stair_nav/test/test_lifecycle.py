"""Deterministic asynchronous-lifecycle checks with a controllable planner worker."""
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path

import pytest

from stair_nav.core.config import load_route, validate_route
from stair_nav.core.engine import SegmentExecutor, State
from stair_nav.core.models import MotionFeedback, NavError, PlannedPath, Pose, Waypoint


class ManualPlannerPool:
    def __init__(self):
        self.jobs = []

    def submit(self, function, *args):
        future = Future()
        # A running worker may finish after cancellation, just like a slow I/O
        # backend. Its result must never be consumed by a new execution.
        future.set_running_or_notify_cancel()
        self.jobs.append((future, args[-1]))
        return future


@pytest.fixture
def setup():
    route = load_route(Path(__file__).resolve().parents[1]/'config'/'demo_route.yaml')
    pool = ManualPlannerPool()
    return route, pool, SegmentExecutor(route, planning_executor=pool)


def path():
    return PlannedPath([Waypoint(0, 0, 0, 'entry', 'stair'),
                        Waypoint(1.5, 0, .5, 'stair', 'stair'),
                        Waypoint(3, 0, 1, 'exit', 'flat')])


def planning(engine, task='task1', execution='exec1', now=1.):
    assert engine.start(task, execution, 'AB', now)
    engine.update_pose(Pose(0, 0, 0, 0, now))
    output = engine.tick(now, .05)
    assert output.status.state == State.PLANNING
    assert not output.command.valid
    return output


def motion(engine, now, **kwargs):
    default = dict(stamp=now, execution_id=engine.status.execution_id,
                   segment_id='AB', mode='stair', ready=True,
                   capabilities=('stair_motion', 'stair_yaw_correction', 'turn_in_place', 'reverse'),
                   supported_modes=('stair', 'flat'))
    default.update(kwargs)
    sample = MotionFeedback(**default)
    engine.update_motion(sample)
    return sample


def executing(engine, pool):
    planning(engine)
    pool.jobs[-1][0].set_result(path())
    engine.update_pose(Pose(0, 0, 0, 0, 1.05))
    assert engine.tick(1.05, .05).status.state == State.WAIT_READY
    motion(engine, 1.1)
    engine.update_pose(Pose(0, 0, 0, 0, 1.1))
    assert engine.tick(1.1, .05).status.state == State.EXECUTING


def test_planning_timeout_revokes_and_late_result_does_not_leak(setup):
    route, pool, engine = setup
    route['defaults']['planning']['timeout'] = .1
    planning(engine)
    old_future, old_cancel = pool.jobs[0]
    engine.update_pose(Pose(0, 0, 0, 0, 1.2))
    output = engine.tick(1.2, .05)
    assert output.status.error_code == 'PLANNING_TIMEOUT'
    assert not output.command.valid and old_cancel.is_set()
    planning(engine, 'task2', 'exec2', 1.25)
    old_future.set_result(path())
    engine.update_pose(Pose(0, 0, 0, 0, 1.3))
    output = engine.tick(1.3, .05)
    assert output.status.state == State.PLANNING
    assert output.path is None
    assert output.command.execution_id == 'exec2'
    assert not pool.jobs[1][1].is_set()


def test_cancel_during_planning_then_new_task_ignores_old_failure(setup):
    _, pool, engine = setup
    planning(engine)
    old_future, old_cancel = pool.jobs[0]
    engine.cancel(1.01)
    output = engine.tick(1.02, .02)
    assert output.status.state == State.CANCELED and output.command.behavior == 'HOLD'
    assert not output.command.valid and output.status.navigation_stopped
    assert old_cancel.is_set()
    planning(engine, 'task2', 'exec2', 1.1)
    old_future.set_exception(NavError('OLD_FAILURE', 'stale old worker exception'))
    fresh_path = path()
    pool.jobs[1][0].set_result(fresh_path)
    engine.update_pose(Pose(0, 0, 0, 0, 1.15))
    output = engine.tick(1.15, .05)
    assert output.status.state == State.WAIT_READY
    assert output.path is fresh_path and output.status.error_code == ''


def test_busy_rejects_without_replacing_active_task_and_ids_cannot_be_reused(setup):
    _, _, engine = setup
    planning(engine)
    assert engine.start('task2', 'exec2', 'BC', 1.01) is False
    assert engine.status.execution_id == 'exec1' and engine.status.segment_id == 'AB'
    engine.cancel(1.02)
    assert engine.start('task2', 'exec2', 'AB', 1.03)
    engine.cancel(1.04)
    with pytest.raises(NavError) as caught:
        engine.start('task3', 'exec1', 'AB', 1.05)
    assert caught.value.code == 'INVALID_ID'


def test_missing_motion_capability_rejects_before_tracking(setup):
    _, pool, engine = setup
    planning(engine)
    pool.jobs[0][0].set_result(path())
    engine.update_pose(Pose(0, 0, 0, 0, 1.05))
    engine.tick(1.05, .05)
    motion(engine, 1.1, capabilities=('stair_motion',))
    engine.update_pose(Pose(0, 0, 0, 0, 1.1))
    output = engine.tick(1.1, .05)
    assert output.status.error_code == 'CAPABILITY_MISSING'
    assert not output.command.valid


@pytest.mark.parametrize('pose', [Pose(0, 0, 0, 0, 1., 'odom'),
                                 Pose(0, 0, 0, 0, 20.),
                                 Pose(float('nan'), 0, 0, 0, 1.),
                                 Pose(0, 0, 0, 0, float('nan'))])
def test_invalid_pose_frame_future_or_nonfinite_fails_safely(setup, pose):
    _, _, engine = setup
    assert engine.start('task1', 'exec1', 'AB', 1.)
    engine.update_pose(pose)
    output = engine.tick(1., .05)
    assert output.status.error_code == 'INVALID_POSE'
    assert not output.command.valid and output.command.behavior == 'HOLD'


def test_missing_pose_has_bounded_input_wait(setup):
    _, _, engine = setup
    assert engine.start('task1', 'exec1', 'AB', 1.)
    assert engine.tick(1., .05).status.state == State.CHECKING
    output = engine.tick(2., .05)
    assert output.status.error_code == 'POSE_MISSING' and not output.command.valid


def test_execution_timeout_is_independent_of_fresh_feedback(setup):
    route, pool, engine = setup
    route['defaults']['timeouts']['execution'] = .2
    executing(engine, pool)
    motion(engine, 1.4)
    engine.update_pose(Pose(0, 0, 0, 0, 1.4))
    output = engine.tick(1.4, .05)
    assert output.status.error_code == 'EXECUTION_TIMEOUT' and not output.command.valid


def test_large_deviation_invalidates_immediately(setup):
    _, pool, engine = setup
    executing(engine, pool)
    motion(engine, 1.15)
    engine.update_pose(Pose(.1, 2., .05, 0, 1.15))
    output = engine.tick(1.15, .05)
    assert output.status.error_code == 'PATH_DEVIATION'
    assert not output.command.valid and output.status.navigation_stopped


def test_motion_movement_during_planning_requires_replan(setup):
    _, pool, engine = setup
    planning(engine)
    pool.jobs[0][0].set_result(path())
    engine.update_pose(Pose(.3, 0, 0, 0, 1.05))
    output = engine.tick(1.05, .05)
    assert output.status.error_code == 'START_MOVED' and output.path is None


def test_worker_exception_is_terminal_and_revokes_command(setup):
    _, pool, engine = setup
    planning(engine)
    pool.jobs[0][0].set_exception(RuntimeError('backend crashed'))
    engine.update_pose(Pose(0, 0, 0, 0, 1.05))
    output = engine.tick(1.05, .05)
    assert output.status.error_code == 'INTERNAL_ERROR'
    assert not output.command.valid


def test_out_of_order_same_task_motion_does_not_override_new_status(setup):
    _, _, engine = setup
    planning(engine)
    fresh = motion(engine, 1.1)
    assert not engine.update_motion(replace(fresh, stamp=1., ready=False, fault_code='STALE_FAULT'))
    assert engine.motion is fresh


@pytest.mark.parametrize('bad_stamp', [float('nan'), 20.])
def test_invalid_pose_timestamp_can_recover_on_a_new_task(setup, bad_stamp):
    _, _, engine = setup
    assert engine.start('task1', 'exec1', 'AB', 1.)
    engine.update_pose(Pose(0, 0, 0, 0, bad_stamp))
    assert engine.tick(1., .05).status.error_code == 'INVALID_POSE'
    assert engine.start('task2', 'exec2', 'AB', 1.1)
    engine.update_pose(Pose(0, 0, 0, 0, 1.1))
    output = engine.tick(1.1, .05)
    assert output.status.state == State.PLANNING
    assert output.status.error_code == ''


@pytest.mark.parametrize('flag,capability', [('allow_turn_in_place', 'turn_in_place'),
                                           ('allow_reverse', 'reverse'),
                                           ('allow_stair_correction', 'stair_yaw_correction')])
def test_enabled_tracker_capability_is_required_even_if_explicit_list_is_empty(setup, flag, capability):
    route, pool, engine = setup
    route['defaults']['required_capabilities'] = []
    route['defaults']['tracking'][flag] = True
    planning(engine)
    pool.jobs[0][0].set_result(path())
    engine.update_pose(Pose(0, 0, 0, 0, 1.05))
    engine.tick(1.05, .05)
    capabilities = tuple(c for c in ('stair_motion', 'stair_yaw_correction', 'turn_in_place', 'reverse')
                         if c != capability)
    motion(engine, 1.1, capabilities=capabilities)
    engine.update_pose(Pose(0, 0, 0, 0, 1.1))
    output = engine.tick(1.1, .05)
    assert output.status.error_code == 'CAPABILITY_MISSING'
    assert capability in output.status.error_message


@pytest.mark.parametrize('key,value', [('command_ttl', float('inf')), ('control_period', 0.)])
def test_invalid_top_level_timing_is_rejected(setup, key, value):
    route, _, _ = setup
    route[key] = value
    with pytest.raises(NavError) as caught:
        validate_route(route)
    assert caught.value.code == 'CONFIG_INVALID'


@pytest.mark.parametrize('section,key,value', [('planning', 'timeout', float('inf')),
                                              ('tracking', 'min_vx', -.1),
                                              ('tracking', 'match_backtrack', float('nan'))])
def test_invalid_motion_parameters_are_rejected(setup, section, key, value):
    route, _, _ = setup
    route['defaults'][section][key] = value
    with pytest.raises(NavError) as caught:
        validate_route(route)
    assert caught.value.code == 'CONFIG_INVALID'
