"""Dynamic obstacle holds preserve execution while all observation leases run."""
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path

import pytest

from stair_nav.core.config import load_route, segment_config, validate_route
from stair_nav.core.engine import SegmentExecutor, State
from stair_nav.core.models import (
    MotionFeedback, NavError, ObstacleObservation, PlannedPath, Pose, Waypoint,
)


class PlannerPool:
    def __init__(self):
        self.path = PlannedPath([Waypoint(-1., 0., 0.), Waypoint(0., 0., 0.)])
        self.jobs = []

    def submit(self, *args):
        future = Future()
        future.set_result(self.path)
        self.jobs.append(future)
        return future


@pytest.fixture
def setup():
    route = load_route(Path(__file__).parent/'fixtures/legacy/demo_route.yaml')
    route['segments']['APPROACH_A']['obstacle_stop'] = {'enabled': True}
    validate_route(route)
    pool = PlannerPool()
    engine = SegmentExecutor(route, planning_executor=pool)
    yield route, pool, engine
    engine.close()


def tick(engine, now, *, blocked=False, cloud=True, pose=True, motion=True,
         x=-.8, y=0., **feedback):
    if pose:
        engine.update_pose(Pose(x, y, 0., 0., now))
    if motion:
        values = dict(stamp=now, execution_id=engine.status.execution_id,
                      segment_id='APPROACH_A', mode='flat', ready=True,
                      stable=True, hold_confirmed=True,
                      capabilities=('turn_in_place',), supported_modes=('flat',))
        values.update(feedback)
        engine.update_motion(MotionFeedback(**values))
    if cloud:
        engine.update_obstacle(ObstacleObservation(
            now, now+.3, engine.status.execution_id, True, blocked,
            path_stamp=engine._path_stamp or 0.))
    return engine.tick(now, .05)


def executing(engine):
    assert engine.start('task', 'execution', 'APPROACH_A', 1.)
    assert tick(engine, 1.).status.state == State.PLANNING
    assert tick(engine, 1.05).status.state == State.WAIT_READY
    assert tick(engine, 1.1).status.state == State.EXECUTING
    output = tick(engine, 1.15)
    assert output.command.valid and output.command.vx > 0.
    return output


def test_pause_preserves_action_path_progress_and_fresh_hold(setup):
    _, pool, engine = setup
    previous = executing(engine)
    tracker = engine._tracker
    paused = tick(engine, 1.2, blocked=True)
    assert paused.status.state == State.PAUSED_OBSTACLE and engine.active
    assert paused.status.phase == 'obstacle_blocked'
    assert paused.status.error_code == ''
    assert paused.path is previous.path is pool.path and engine._tracker is tracker
    assert paused.status.progress == previous.status.progress
    assert paused.command.task_id == 'task' and paused.command.execution_id == 'execution'
    assert paused.command.behavior == 'HOLD' and not paused.command.valid
    assert paused.command.vx == paused.command.wz == 0.
    again = tick(engine, 1.3, blocked=True)
    assert again.command.stamp > paused.command.stamp
    assert again.command.valid_until > paused.command.valid_until
    assert not engine.start('other', 'new', 'APPROACH_A', 1.3)


def test_clear_resumes_same_path_from_zero_velocity(setup):
    route, pool, engine = setup
    executing(engine)
    engine._tracker.vx, engine._tracker.wz = .3, .2
    tick(engine, 1.2, blocked=True)
    assert engine._tracker.vx == engine._tracker.wz == 0.
    resumed = tick(engine, 1.4)
    assert resumed.status.state == State.EXECUTING
    assert resumed.command.behavior == 'HOLD'
    output = tick(engine, 1.45)
    assert output.command.valid and output.path is pool.path
    assert 0 < output.command.vx <= route['defaults']['tracking']['max_accel']*.05+1e-9
    assert output.command.execution_id == 'execution'
    assert len(pool.jobs) == 1


@pytest.mark.parametrize('condition', ['ready', 'stable', 'hold_confirmed'])
def test_clear_waits_for_true_stop_and_readiness(setup, condition):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    output = tick(engine, 1.3, **{condition: False})
    assert output.status.state == State.PAUSED_OBSTACLE
    assert output.status.phase == 'waiting_hold' and not output.command.valid
    assert tick(engine, 1.4).status.state == State.EXECUTING


def test_resume_requires_hold_evidence_after_this_pause_started(setup):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    output = tick(engine, 1.25, motion=False)
    assert output.status.state == State.PAUSED_OBSTACLE
    assert output.status.phase == 'waiting_hold'
    assert tick(engine, 1.3).status.state == State.EXECUTING


@pytest.mark.parametrize('change', [
    {'valid': False}, {'valid_until': 1.2}, {'stamp': float('nan')},
    {'stamp': 4.}, {'valid_until': float('inf')},
])
def test_invalid_clear_observation_holds_instead_of_resuming(setup, change):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    observation = ObstacleObservation(1.3, 1.6, 'execution', True, False)
    assert engine.update_obstacle(replace(observation, **change))
    output = tick(engine, 1.3, cloud=False)
    assert output.status.state == State.PAUSED_OBSTACLE
    assert output.status.phase == 'obstacle_invalid' and not output.command.valid


def test_clear_lease_expiry_and_wrong_execution_cannot_authorize_motion(setup):
    _, _, engine = setup
    executing(engine)
    output = tick(engine, 1.5, cloud=False)
    assert output.status.state == State.PAUSED_OBSTACLE
    assert output.status.phase == 'obstacle_invalid'
    assert not engine.update_obstacle(ObstacleObservation(1.6, 2., 'old-execution', True, False))
    assert tick(engine, 1.6, cloud=False).status.state == State.PAUSED_OBSTACLE
    assert tick(engine, 1.65).status.state == State.EXECUTING


@pytest.mark.parametrize('path_stamp', [0., 1., float('nan')])
def test_clear_for_missing_or_old_path_cannot_authorize_tracking(setup, path_stamp):
    _, _, engine = setup
    executing(engine)
    assert engine._path_stamp == pytest.approx(1.05)
    engine.update_obstacle(ObstacleObservation(
        1.2, 1.5, 'execution', True, False, path_stamp=path_stamp))
    output = tick(engine, 1.2, cloud=False)
    assert output.status.state == State.PAUSED_OBSTACLE
    assert output.status.phase == 'obstacle_invalid' and not output.command.valid
    assert tick(engine, 1.3).status.state == State.EXECUTING
    assert tick(engine, 1.35).command.valid


def test_preplanning_clear_does_not_allow_one_track_before_path_is_observed(setup):
    _, _, engine = setup
    assert engine.start('task', 'execution', 'APPROACH_A', 1.)
    assert tick(engine, 1.).status.state == State.PLANNING
    assert tick(engine, 1.05).status.state == State.WAIT_READY
    # A clear decision from the initial front rectangle has a current cloud
    # lease but has not examined the newly generated path.
    engine.update_obstacle(ObstacleObservation(1.1, 1.4, 'execution', True, False))
    output = tick(engine, 1.1, cloud=False)
    assert output.status.state == State.PAUSED_OBSTACLE
    assert not output.command.valid
    assert tick(engine, 1.2).status.state == State.WAIT_READY
    assert tick(engine, 1.25).status.state == State.EXECUTING
    assert tick(engine, 1.3).command.valid


def test_old_clear_cannot_overwrite_a_newer_block(setup):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    assert not engine.update_obstacle(ObstacleObservation(1.1, 2., 'execution', True, False))
    assert tick(engine, 1.25, cloud=False).status.phase == 'obstacle_blocked'


def test_bad_future_timestamp_does_not_poison_fresh_sensor_recovery(setup):
    _, _, engine = setup
    executing(engine)
    engine.update_obstacle(ObstacleObservation(40., 41., 'execution', True, False))
    assert tick(engine, 1.2, cloud=False).status.state == State.PAUSED_OBSTACLE
    assert tick(engine, 1.3).status.state == State.EXECUTING


def test_missing_cloud_stops_before_planning_and_allows_motion_handshake(setup):
    _, pool, engine = setup
    assert engine.start('task', 'execution', 'APPROACH_A', 1.)
    output = tick(engine, 1., cloud=False, motion=False)
    assert output.status.state == State.PAUSED_OBSTACLE and not pool.jobs
    assert tick(engine, 1.1, motion=False).status.phase == 'waiting_hold'
    assert tick(engine, 1.15).status.state == State.CHECKING
    assert tick(engine, 1.2).status.state == State.PLANNING


def test_missing_current_execution_motion_has_bounded_handshake(setup):
    _, _, engine = setup
    assert engine.start('task', 'execution', 'APPROACH_A', 1.)
    tick(engine, 1., cloud=False, motion=False)
    output = tick(engine, 4.1, cloud=False, motion=False)
    assert output.status.state == State.FAILED
    assert output.status.error_code == 'READY_TIMEOUT'


def test_execution_deadline_excludes_only_time_spent_paused(setup):
    route, _, engine = setup
    route['defaults']['timeouts']['execution'] = .4
    executing(engine)
    tick(engine, 1.2, blocked=True)
    assert tick(engine, 10., blocked=True).status.state == State.PAUSED_OBSTACLE
    assert tick(engine, 10.1).status.state == State.EXECUTING
    assert tick(engine, 10.15).command.valid
    output = tick(engine, 10.55)
    assert output.status.error_code == 'EXECUTION_TIMEOUT'


def test_arrival_pause_does_not_accumulate_success_confirmation(setup):
    _, _, engine = setup
    executing(engine)
    assert tick(engine, 1.2, x=0.).status.state == State.ARRIVAL_CONFIRM
    tick(engine, 1.25, x=0., blocked=True)
    assert tick(engine, 5., x=0., blocked=True).status.state == State.PAUSED_OBSTACLE
    resumed = tick(engine, 5.05, x=0.)
    assert resumed.status.state == State.ARRIVAL_CONFIRM
    assert not resumed.status.geometry_reached
    assert tick(engine, 5.1, x=0.).status.state == State.ARRIVAL_CONFIRM
    assert tick(engine, 5.35, x=0.).status.state == State.ARRIVAL_CONFIRM
    assert tick(engine, 5.5, x=0.).status.state == State.SUCCEEDED


def test_paused_task_still_honors_cancel_and_never_auto_resumes(setup):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    engine.cancel(1.25)
    output = tick(engine, 1.3)
    assert output.status.state == State.CANCELED and not engine.active
    assert output.command.behavior == 'HOLD'


@pytest.mark.parametrize('fault,code', [('motion', 'MOTION_STALE'),
                                      ('pose', 'POSE_STALE'), ('reported', 'MOTION_FAULT')])
def test_real_faults_remain_active_while_obstacle_paused(setup, fault, code):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    output = tick(engine, 1.8, blocked=True, motion=fault != 'motion',
                  pose=fault != 'pose', fault_code='ADAPTER_FAULT' if fault == 'reported' else '')
    assert output.status.state == State.FAILED and output.status.error_code == code
    assert not output.command.valid


def test_resuming_does_not_hide_path_deviation(setup):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    assert tick(engine, 1.3, y=2.).status.state == State.EXECUTING
    output = tick(engine, 1.35, y=2.)
    assert output.status.error_code == 'PATH_DEVIATION' and not output.command.valid


def test_optional_maximum_pause_fails_explicitly(setup):
    route, _, engine = setup
    route['segments']['APPROACH_A']['obstacle_stop']['max_pause'] = .2
    executing(engine)
    tick(engine, 1.2, blocked=True)
    output = tick(engine, 1.45, blocked=True)
    assert output.status.state == State.FAILED
    assert output.status.error_code == 'OBSTACLE_PAUSE_TIMEOUT'


def test_hard_channel_block_is_still_terminal_even_during_pause(setup):
    _, _, engine = setup
    executing(engine)
    tick(engine, 1.2, blocked=True)
    engine.set_blocked('APPROACH_A', True, 'closed route', 1.25)
    output = tick(engine, 1.3)
    assert output.status.error_code == 'CHANNEL_BLOCKED' and output.status.state == State.FAILED


def test_feature_is_disabled_by_default(setup):
    route, _, engine = setup
    route['segments']['APPROACH_A'].pop('obstacle_stop')
    assert segment_config(route, 'APPROACH_A')['obstacle_stop'] == {'enabled': False, 'max_pause': 0.}
    assert engine.start('task', 'execution', 'APPROACH_A', 1.)
    assert tick(engine, 1., cloud=False).status.state == State.PLANNING


@pytest.mark.parametrize('settings', [None, {'enabled': 'true'}, {'max_pause': -.1},
                                     {'max_pause': True}, {'max_pause': float('nan')},
                                     {'max_pause': float('inf')}, {'typo': 1}])
def test_obstacle_configuration_rejects_invalid_values(setup, settings):
    route, _, _ = setup
    route['segments']['APPROACH_A']['obstacle_stop'] = settings
    with pytest.raises(NavError, match='obstacle_stop'):
        validate_route(route)


def test_stair_obstacle_pause_cannot_be_enabled_by_accident(setup):
    route, _, _ = setup
    route['segments']['AB']['obstacle_stop'] = {'enabled': True}
    with pytest.raises(NavError, match='platform segments in flat mode'):
        validate_route(route)
