"""Behavior tests drive the real planning/tracking/execution core through vx/wz."""
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from stair_nav.core.config import load_route, segment_config
from stair_nav.core.engine import State
from stair_nav.core.planning import OccupancyGrid
from stair_nav.demo import Harness


@pytest.fixture
def route():
    return load_route(Path(__file__).resolve().parent / 'fixtures' / 'legacy' / 'demo_route.yaml')


def executing(harness, segment='AB'):
    harness.start(segment)
    for _ in range(200):
        result = harness.step()
        if result.status.state == State.EXECUTING and result.command.valid:
            return result
        assert result.status.state != State.FAILED, result.status
    pytest.fail('executor never entered tracking')


def test_three_segments_close_real_feedback_loop(route):
    harness = Harness(route)
    report = harness.run_sequence()
    assert report['success'], report
    assert [r['segment_id'] for r in report['results']] == ['AB', 'BC', 'CD']
    assert harness.command_count > 100
    assert abs(harness.sim.pose.x-9.0) < .15
    assert abs(harness.sim.pose.z-2.0) < .12
    # A* segment must actually detour in XY rather than teleport at handover.
    assert max(abs(y) for x, y, z in harness.trajectory if x > 6.4) > .45
    assert all(r['hold_confirmed'] for r in report['results'])


def test_not_ready_times_out_without_motion(route):
    harness = Harness(route)
    harness.sim.faults['not_ready'] = True
    harness.start('AB')
    result = harness.until_terminal()
    assert result.status.state == State.FAILED
    assert result.status.error_code == 'READY_TIMEOUT'
    assert not result.command.valid
    assert harness.sim.pose.x == 0.0


def test_localization_dropout_revokes_command(route):
    route['defaults']['timeouts']['pose_wait'] = 3.
    harness = Harness(route)
    executing(harness)
    harness.sim.faults['localization_dropout'] = True
    result = harness.until_terminal()
    assert result.status.error_code == 'POSE_STALE'
    assert not result.command.valid
    assert result.status.navigation_stopped


def test_tracking_localization_recovery_requires_stop_and_new_pose(route):
    route['segments']['AB']['timeouts'] = {'pose_recovery': 3.}
    harness = Harness(route)
    executing(harness)
    engine = harness.engine
    original_path = engine.path
    execution_started = engine._execution_started
    stale_at = engine.pose.stamp + engine.seg['timeouts']['pose'] + .01
    waiting = engine.tick(stale_at, .05)
    assert waiting.status.phase == 'recovering_pose'
    assert waiting.command.behavior == 'HOLD' and not waiting.command.valid
    assert waiting.path is None

    # A previous HOLD acknowledgement may still be fresh but predates the stop.
    engine.update_motion(replace(engine.motion, stamp=stale_at - .01,
                                 stable=True, hold_confirmed=True))
    engine.update_pose(replace(engine.pose, stamp=stale_at + .05))
    assert engine.tick(stale_at + .05, .05).status.phase == 'recovering_pose'

    engine.update_motion(replace(engine.motion, stamp=stale_at + .10))
    assert engine.tick(stale_at + .10, .05).status.phase == 'recovering_pose'
    engine.update_pose(replace(engine.pose, stamp=stale_at + .15, x=engine.pose.x + .1))
    planning = engine.tick(stale_at + .15, .05)
    assert planning.status.state == State.PLANNING and not planning.command.valid
    assert engine._plan_start == engine.pose
    ready = engine.tick(stale_at + .20, .05)
    assert ready.status.state == State.WAIT_READY and ready.path is not original_path
    assert not engine.tick(stale_at + .25, .05).command.valid
    assert engine.tick(stale_at + .30, .05).command.valid
    assert engine._execution_started == execution_started


def test_tracking_localization_recovery_expires_while_holding(route):
    route['segments']['AB']['timeouts'] = {'pose_recovery': 3.}
    harness = Harness(route)
    executing(harness)
    engine = harness.engine
    stale_at = engine.pose.stamp + engine.seg['timeouts']['pose'] + .01
    assert engine.tick(stale_at, .05).status.phase == 'recovering_pose'
    result = engine.tick(stale_at + 3., .05)
    assert result.status.state == State.FAILED
    assert result.status.error_code == 'POSE_STALE'
    assert result.command.behavior == 'HOLD' and not result.command.valid


def test_tracking_localization_recovery_respects_execution_deadline(route):
    route['segments']['AB']['timeouts'] = {'pose_recovery': 3., 'execution': 2.}
    harness = Harness(route)
    executing(harness)
    engine = harness.engine
    stale_at = engine.pose.stamp + engine.seg['timeouts']['pose'] + .01
    assert engine.tick(stale_at, .05).status.phase == 'recovering_pose'
    result = engine.tick(engine._execution_started + 2.01, .05)
    assert result.status.state == State.FAILED
    assert result.status.error_code == 'EXECUTION_TIMEOUT'
    assert result.command.behavior == 'HOLD' and not result.command.valid


@pytest.mark.parametrize('stage', [State.PLANNING, State.WAIT_READY])
def test_stale_pose_before_motion_discards_plan_and_replans(route, stage):
    route['defaults']['timeouts']['pose_wait'] = 3.
    harness = Harness(route)
    harness.start('AB')
    engine = harness.engine
    assert harness.step().status.state == State.PLANNING
    if stage == State.WAIT_READY:
        assert harness.step().status.state == State.WAIT_READY
    cancel_event = engine._cancel_event
    stale_at = engine.pose.stamp + engine.seg['timeouts']['pose'] + .01
    waiting = engine.tick(stale_at, .05)
    assert waiting.status.phase == 'waiting_pose'
    assert waiting.path is None and cancel_event.is_set()
    assert waiting.command.behavior == 'HOLD' and not waiting.command.valid
    fresh_at = stale_at + .2
    fresh = replace(engine.pose, stamp=fresh_at, x=engine.pose.x + .1)
    engine.update_pose(fresh)
    assert engine.tick(fresh_at, .05).status.state == State.PLANNING
    assert engine._plan_start == fresh
    assert engine.tick(fresh_at + .05, .05).status.state == State.WAIT_READY


def test_stale_pose_wait_cannot_extend_its_deadline(route):
    route['defaults']['timeouts']['pose_wait'] = 3.
    harness = Harness(route)
    harness.start('AB')
    harness.step()
    engine = harness.engine
    stale_at = engine.pose.stamp + engine.seg['timeouts']['pose'] + .01
    assert engine.tick(stale_at, .05).status.phase == 'waiting_pose'
    resumed_at = stale_at + 1.
    engine.update_pose(replace(engine.pose, stamp=resumed_at))
    assert engine.tick(resumed_at, .05).status.state == State.PLANNING
    assert engine.tick(resumed_at + 1., .05).status.phase == 'waiting_pose'
    result = engine.tick(stale_at + 3., .05)
    assert result.status.state == State.FAILED
    assert result.status.error_code == 'POSE_STALE'
    assert not result.command.valid


def test_cancel_invalidates_before_hold_acknowledgement(route):
    harness = Harness(route)
    executing(harness)
    # Ensure the simulation has physically moved and no earlier HOLD ACK remains.
    for _ in range(20):
        harness.step()
    assert harness.sim.pose.x > .1
    harness.engine.cancel(harness.now)
    result = harness.step()
    assert result.status.state == State.CANCELED
    assert result.status.navigation_stopped and not result.command.valid
    assert not result.status.hold_confirmed
    for _ in range(12):
        result = harness.step()
    assert result.status.hold_confirmed


def test_old_execution_feedback_is_rejected(route):
    harness = Harness(route)
    executing(harness)
    old = harness.sim.motion_status(harness.now)
    harness.engine.cancel(harness.now)
    harness.step()
    assert harness.start('AB', 'new-task')
    harness.step()
    assert harness.engine.update_motion(old) is False
    future_old = replace(old, stamp=harness.now+100, fault_code='LATE_FAULT')
    assert harness.engine.update_motion(future_old) is False
    current = harness.sim.motion_status(harness.now)
    assert harness.engine.update_motion(current) is True
    assert harness.engine.motion.fault_code == ''


def test_geometric_candidate_waits_for_exit_confirmation(route):
    harness = Harness(route)
    harness.sim.faults['missing_exit'] = True
    harness.start('AB')
    candidate_seen = False
    for _ in range(1000):
        result = harness.step()
        if result.status.state == State.ARRIVAL_CONFIRM:
            candidate_seen = True
            assert result.status.geometry_reached
            assert not result.command.valid
        assert result.status.state != State.SUCCEEDED
        if not harness.engine.active:
            break
    assert candidate_seen
    assert result.status.state == State.FAILED
    assert result.status.error_code == 'ARRIVAL_CONFIRM_TIMEOUT'


def test_motion_feedback_dropout_revokes_command(route):
    harness = Harness(route)
    executing(harness)
    for _ in range(40):
        result = harness.step(publish_motion=False)
        if not harness.engine.active:
            break
    assert result.status.error_code == 'MOTION_STALE'
    assert not result.command.valid


def test_channel_blocked_event_revokes_command(route):
    harness = Harness(route)
    executing(harness)
    harness.engine.set_blocked('AB', True, 'test corridor closed', harness.now)
    result = harness.step()
    assert result.status.error_code == 'CHANNEL_BLOCKED'
    assert not result.command.valid


def test_execution_fault_is_not_misreported_as_success(route):
    harness = Harness(route)
    executing(harness)
    harness.sim.faults['execution_fault'] = True
    result = harness.step()
    assert result.status.error_code == 'MOTION_FAULT'
    assert not result.command.valid


def test_integrated_upper_platform_trajectory_clears_inflated_obstacles(route):
    harness = Harness(route)
    assert harness.run_sequence()['success']
    segment = segment_config(route, 'CD')
    grid_data = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    grid = OccupancyGrid.from_mapping(grid_data, segment['planning'])
    checked = 0
    for a, b in zip(harness.trajectory, harness.trajectory[1:]):
        if a[0] >= 6.0 and b[0] >= 6.0:
            assert grid.line_is_free(a[:2], b[:2]), (a, b)
            checked += 1
    assert checked > 100
