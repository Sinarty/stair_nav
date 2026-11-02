"""Planner/executor -> M20 adapter -> velocity-integrated plant -> native feedback.

No ROS or robot is used. The plant has no contact, inertia or leg dynamics;
these tests verify the deployed interface handshake, not physical locomotion.
"""
from pathlib import Path

import pytest

from stair_nav.core.config import load_route, segment_config
from stair_nav.core.engine import SegmentExecutor, State
from stair_nav.core.m20_motion import M20Config, M20MotionAdapter, MotionObservation
from stair_nav.core.models import Command
from stair_nav.demo import InlineExecutor
from stair_nav.demo.simulation import Simulator


class M20ExecutionLoop:
    dt = .05

    def __init__(self):
        route = load_route(Path(__file__).resolve().parents[1]/'stair_nav/demo/config/route.yaml')
        route['segments'] = {'BC': segment_config(route, 'BC')}
        route['sequence'] = ['BC']
        route.pop('approach_segment', None)
        route['simulation'] = dict(capabilities=['turn_in_place'], supported_modes=['flat'])
        self.route = route
        start = route['nodes']['B']
        self.plant = Simulator(route, initial_pose=[*start['position'], start['yaw']])
        self.engine = SegmentExecutor(route, planning_executor=InlineExecutor())
        self.adapter = M20MotionAdapter(M20Config(
            dry_run=False, navigation_mode_confirmed=True, planner_stopped=True,
            normal_location_status=1, supported_modes=('flat',), capabilities=('turn_in_place',),
            max_vx=.5, max_wz=1.))
        self.now, self.steady = 1., 101.
        self.gait, self.pending_gait = 4097, None
        self.output = None
        self.states, self.gait_requests, self.gait_acks = [], [], []
        self.arrival_time = self.first_hold_time = None
        self.velocity_count = 0
        # No command is held during preparation: readiness comes from native data.
        for _ in range(12):
            self._advance_native()
            self.output = self.adapter.tick(self.now, self.steady)
        assert self.output.feedback.stable and not self.output.publish_velocity
        assert self.adapter.enable(True, self.now, self.steady)[0]
        assert self.engine.start('integration-task', 'integration-execution', 'BC', self.now)

    def _advance_native(self):
        self.now += self.dt
        self.steady += self.dt
        if self.pending_gait is not None:
            self.gait = self.pending_gait
            self.gait_acks.append((self.now, self.gait))
            self.pending_gait = None
        if self.output is not None and self.output.publish_velocity:
            # Simulator is solely the velocity integrator. It receives the SDK
            # velocity actually emitted, not the planner's candidate command.
            physical = Command(
                self.now, self.now+.2, 'virtual-plant', 'physical-output', 'BC',
                'platform', 'flat', 'TRACK', self.output.vx, self.output.wz, True)
            assert self.plant.accept(physical)
            self.velocity_count += int(physical.vx != 0 or physical.wz != 0)
        else:
            self.plant.command = None
        self.plant.step(self.dt, self.now)
        actual = self.plant.command
        vx, wz = (self.plant._speed, self.plant._turn) if actual else (0., 0.)
        self.adapter.observe_motion(MotionObservation(self.now, 17, self.gait, vx, 0., wz))
        self.adapter.observe_imu(self.now, abs(wz))
        self.adapter.observe_location(self.now, 1)
        self.adapter.observe_estop(self.steady, False)
        self.adapter.observe_charge(self.steady, 0, 0)

    def step(self):
        previous_feedback = self.output.feedback
        self._advance_native()
        self.engine.update_pose(self.plant.pose)
        self.engine.update_motion(previous_feedback)
        navigation = self.engine.tick(self.now, self.dt)
        self.states.append(navigation.status.state)
        assert self.adapter.accept(navigation.command, self.now, self.steady), (
            navigation.command, self.adapter.tick(self.now, self.steady).diagnostics)
        self.output = self.adapter.tick(self.now, self.steady)
        if self.output.gait_request is not None:
            self.gait_requests.append((self.now, self.output.gait_request))
            self.pending_gait = self.output.gait_request
        if navigation.status.state == State.ARRIVAL_CONFIRM and self.arrival_time is None:
            self.arrival_time = self.now
            assert not self.output.feedback.hold_confirmed
        if self.arrival_time is not None and self.output.feedback.hold_confirmed and self.first_hold_time is None:
            self.first_hold_time = self.now
        assert self.output.feedback.fault_code == '', self.output.feedback
        return navigation

    def close(self):
        self.engine.close()


@pytest.fixture
def loop():
    harness = M20ExecutionLoop()
    try:
        yield harness
    finally:
        harness.close()


def test_flat_grid_segment_completes_through_real_m20_interface_handshake(loop):
    initial = loop.plant.pose
    for _ in range(2000):
        navigation = loop.step()
        assert navigation.status.state != State.FAILED, navigation.status
        if not loop.engine.active:
            break
    assert navigation.status.state == State.SUCCEEDED, navigation.status
    assert {State.PLANNING, State.WAIT_READY, State.EXECUTING, State.ARRIVAL_CONFIRM} <= set(loop.states)
    assert loop.gait_requests == [(loop.gait_requests[0][0], 12290)]
    assert len(loop.gait_acks) == 1 and loop.gait_acks[0][1] == 12290
    assert loop.gait_acks[0][0] > loop.gait_requests[0][0]
    assert loop.velocity_count > 50
    goal = loop.route['nodes']['C']
    assert abs(loop.plant.pose.x-goal['position'][0]) <= goal['arrival']['xy']
    assert abs(loop.plant.pose.y-goal['position'][1]) <= goal['arrival']['xy']
    assert loop.plant.pose.x-initial.x > 5.
    assert loop.plant.pose.z == initial.z
    assert navigation.status.hold_confirmed and navigation.status.navigation_stopped
    assert loop.output.feedback.stable and loop.output.feedback.hold_confirmed
    # Geometry cannot complete a segment until fresh post-stop native samples
    # prove stability and then the segment's own confirmation dwell elapses.
    assert loop.first_hold_time-loop.arrival_time >= loop.adapter.config.stable_seconds
    assert loop.now-loop.first_hold_time >= goal['arrival']['confirm_seconds']


def test_cancel_revokes_velocity_before_m20_native_hold_confirmation(loop):
    initial_x = loop.plant.pose.x
    for _ in range(1000):
        navigation = loop.step()
        assert navigation.status.state != State.FAILED, navigation.status
        if loop.plant.pose.x-initial_x > .3 and loop.output.vx > .05:
            break
    else:
        pytest.fail('virtual robot never executed platform translation')
    loop.engine.cancel(loop.now)
    navigation = loop.step()
    canceled_at = loop.now
    assert navigation.status.state == State.CANCELED
    assert not navigation.command.valid and navigation.command.behavior == 'HOLD'
    assert loop.output.publish_velocity and loop.output.vx == loop.output.wz == 0.
    assert not loop.output.feedback.hold_confirmed and not navigation.status.hold_confirmed
    for _ in range(30):
        navigation = loop.step()
        assert not navigation.command.valid
        if navigation.status.hold_confirmed:
            break
    assert navigation.status.state == State.CANCELED and navigation.status.hold_confirmed
    assert loop.output.feedback.stable and loop.output.feedback.hold_confirmed
    assert loop.now-canceled_at >= loop.adapter.config.stable_seconds
