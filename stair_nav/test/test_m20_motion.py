"""Native M20 feedback and dual-clock command leases, without ROS or hardware."""
import math

import pytest

from stair_nav.core.m20_motion import M20Config, M20MotionAdapter, MotionObservation
from stair_nav.core.models import Command, MotionFeedback


class Rig:
    def __init__(self, **options):
        settings = dict(dry_run=False, navigation_mode_confirmed=True, planner_stopped=True,
                        normal_location_status=0, supported_modes=('flat', 'stair'))
        settings.update(options)
        self.adapter = M20MotionAdapter(M20Config(**settings))
        self.now, self.steady, self.gait = 10., 100., settings.get('flat_gait', 12290)
        self.feed()
        self.adapter.tick(self.now, self.steady)

    def feed(self, vx=0., vy=0., wz=0., gyro=0., state=17, estop=False, charge=0, error=0):
        a = self.adapter
        a.observe_motion(MotionObservation(self.now, state, self.gait, vx, vy, wz))
        a.observe_imu(self.now, gyro)
        a.observe_location(self.now, 0)
        a.observe_estop(self.steady, estop)
        a.observe_charge(self.steady, charge, error)

    def step(self, dt=.1, **readings):
        self.now += dt
        self.steady += dt
        self.feed(**readings)
        return self.adapter.tick(self.now, self.steady)

    def enable(self, hold=True):
        for _ in range(4):
            self.step()
        result = self.adapter.enable(True, self.now, self.steady)
        if result[0] and hold:
            assert self.accept()
            for _ in range(5):
                self.step()
                assert self.accept()
            self.step(.01)  # A following command has a strictly newer issue time.
        return result

    def command(self, behavior='HOLD', mode='flat', **changes):
        values = dict(stamp=self.now, valid_until=self.now+.4, task_id='task',
                      execution_id='run', segment_id='AB', phase='platform', mode=mode,
                      behavior=behavior, vx=.2 if behavior == 'TRACK' else 0.,
                      wz=0., valid=behavior == 'TRACK')
        values.update(changes)
        return Command(**values)

    def accept(self, **changes):
        return self.adapter.accept(self.command(**changes), self.now, self.steady)

    def tick(self):
        return self.adapter.tick(self.now, self.steady)

    def confirm(self, **changes):
        values = dict(stamp=self.now, execution_id='run', segment_id='AB', mode='stair',
                      stable=True, hold_confirmed=True, exit_confirmed=False)
        values.update(changes)
        self.adapter.confirm(MotionFeedback(**values))


def test_default_dry_run_never_writes_and_idle_ready_needs_no_command():
    dry = Rig(dry_run=True)
    assert not dry.enable()[0]
    assert dry.accept(mode='stair')
    out = dry.tick()
    assert not out.publish_velocity and out.gait_request is None and not out.feedback.ready
    rig = Rig()
    assert rig.enable(hold=False)[0]
    out = rig.tick()
    assert out.feedback.ready and out.feedback.stable and not out.publish_velocity
    observed = out.feedback.stamp
    out = rig.adapter.tick(rig.now+.05, rig.steady+.05)
    assert out.feedback.stamp == observed  # No timer-based timestamp renewal.
    assert not M20MotionAdapter(M20Config()).tick(1., 1.).feedback.stamp


@pytest.mark.parametrize('options,reason', [
    ({'navigation_mode_confirmed': False}, 'NAVIGATION_MODE_UNCONFIRMED'),
    ({'planner_stopped': False}, 'PLANNER_NOT_STOPPED'),
    ({'normal_location_status': -1}, 'LOCATION_STATUS_UNCONFIRMED'),
    ({'normal_location_status': 1}, 'LOCATION_ABNORMAL'),
])
def test_enable_requires_verified_usage_and_location_code(options, reason):
    assert Rig(**options).enable() == (False, reason)


@pytest.mark.parametrize('readings,reason', [
    ({'estop': True}, 'ESTOP_ACTIVE'), ({'charge': 1}, 'CHARGING_OR_ERROR'),
    ({'error': 5}, 'CHARGING_OR_ERROR'), ({'state': 3}, 'MOTION_STATE'),
])
def test_native_safety_interlocks_block_enable(readings, reason):
    rig = Rig()
    for _ in range(4):
        rig.step(**readings)
    assert rig.adapter.enable(True, rig.now, rig.steady) == (False, reason)


def test_stability_needs_both_linear_components_yaw_imu_and_new_samples():
    for readings in ({'vy': .2}, {'wz': .2}, {'gyro': .2}, {'vx': .2}):
        rig = Rig()
        for _ in range(5):
            assert not rig.step(**readings).feedback.stable
    rig = Rig()
    assert not rig.adapter.tick(rig.now+.4, rig.steady+.4).feedback.stable
    assert rig.adapter.enable(True, rig.now+.4, rig.steady+.4) == (False, 'NOT_STABLE')


@pytest.mark.parametrize('changes', [
    {'task_id': ''}, {'execution_id': ''}, {'segment_id': ''}, {'execution_id': 4},
    {'vx': math.nan}, {'wz': math.inf}, {'stamp': 20.}, {'valid_until': 9.},
    {'valid_until': 11.}, {'behavior': 'TRACK', 'valid': False},
    {'behavior': 'HOLD', 'valid': True}, {'vx': .1}, {'mode': 'fly'},
])
def test_bad_commands_do_not_create_execution(changes):
    rig = Rig()
    assert not rig.accept(**changes)
    assert rig.tick().feedback.execution_id == ''


def test_hold_registers_identity_and_stale_replays_cannot_replace_or_renew_lease():
    rig = Rig()
    assert rig.enable()[0] and rig.accept()
    assert rig.tick().feedback.hold_confirmed
    rig.step(.01)
    track = rig.command('TRACK', wz=.2)
    assert rig.adapter.accept(track, rig.now, rig.steady)
    out = rig.tick()
    assert (out.vx, out.wz) == (.2, .2) and out.publish_velocity
    assert not rig.adapter.accept(track, rig.now, rig.steady)
    assert not rig.accept(stamp=rig.now-.01)
    assert not rig.accept(task_id='other')
    assert not rig.accept(execution_id='new')
    # Same-stamp HOLD revocation wins over TRACK; its timestamp cannot reauthorize TRACK.
    assert rig.accept()
    assert not rig.adapter.accept(track, rig.now, rig.steady)
    for _ in range(5):
        rig.step()
    assert rig.accept(execution_id='new')
    rig.step(.01)
    assert not rig.accept(execution_id='run')


def test_steady_watchdog_expires_with_ros_time_paused_and_ros_rollback_disables():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    assert rig.tick().vx == .2
    out = rig.adapter.tick(rig.now, rig.steady+.41)
    assert out.publish_velocity and out.vx == out.wz == 0.
    assert out.diagnostics['command_expired']
    out = rig.adapter.tick(rig.now-.1, rig.steady+.42)
    assert not rig.adapter.enabled and out.feedback.fault_code == 'CLOCK_JUMP'
    assert out.vx == 0.


def test_limits_fault_without_changing_curvature_and_deadzone_is_diagnostic_only():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK', vx=.05, wz=.1)
    out = rig.tick()
    assert (out.vx, out.wz) == (.05, .1)
    assert out.diagnostics['velocity_deadzone'] == {'vx': True, 'wz': True}
    rig.step(.01)
    assert not rig.accept(behavior='TRACK', vx=.31, wz=.2)
    out = rig.tick()
    assert out.feedback.fault_code == 'COMMAND_LIMIT'
    assert out.vx == out.wz == 0. and out.publish_velocity


@pytest.mark.parametrize('source', ['motion_nan', 'imu_nan', 'motion_stale', 'imu_stale',
                                     'location_stale', 'safety_stale'])
def test_native_invalid_or_stale_data_never_authorizes_velocity(source):
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    if source == 'motion_nan':
        rig.adapter.observe_motion(MotionObservation(rig.now+.01, 17, rig.gait, math.nan, 0., 0.))
    elif source == 'imu_nan':
        rig.adapter.observe_imu(rig.now+.01, math.nan)
    else:
        setattr(rig.adapter, '_' + source.removesuffix('_stale'), None)
        if source == 'safety_stale':
            rig.adapter._estop = None
    out = rig.tick()
    assert not out.feedback.ready and out.vx == 0.
    if source == 'motion_nan':
        assert out.feedback.stamp == 0.


def test_gait_request_is_hold_only_once_and_native_acknowledged_before_track():
    rig = Rig()
    rig.gait = 4097  # Known initial basic gait can be enabled but is not target flat mode.
    assert rig.enable(hold=False)[0]
    assert rig.tick().feedback.mode == ''
    assert rig.accept(mode='stair')
    for _ in range(5):
        out = rig.step()
        assert rig.accept(mode='stair')
        if out.gait_request:
            break
    assert out.gait_request == 12291 and out.feedback.ready
    assert rig.tick().gait_request is None
    rig.step(.05)
    assert rig.accept(behavior='TRACK', mode='stair')
    assert rig.tick().vx == 0. and rig.tick().feedback.ready
    rig.gait = 12291
    out = rig.step(.05)
    assert out.feedback.mode == 'stair' and out.vx == .2


def test_gait_timeout_is_a_fault_and_never_reissues_switch():
    rig = Rig(gait_timeout=.3)
    assert rig.enable()[0]
    rig.step(.01)
    assert rig.accept(mode='stair')
    assert rig.tick().gait_request == 12291
    for _ in range(4):
        out = rig.step()
    assert out.feedback.fault_code == 'GAIT_TIMEOUT' and out.gait_request is None
    assert out.vx == 0.


def test_stair_hold_can_cancel_midflight_but_flat_switch_requires_exit_evidence():
    rig = Rig()
    rig.gait = 12291
    assert rig.enable(hold=False)[0] and rig.accept(mode='stair')
    rig.step(.01)
    assert rig.accept(behavior='TRACK', mode='stair')
    assert rig.tick().vx == .2
    rig.step(.05, vx=.2)
    assert rig.accept(mode='flat')  # Stop request does not assume the centre is off stairs.
    for _ in range(5):
        out = rig.step()
    assert not out.feedback.hold_confirmed and out.gait_request is None
    rig.confirm(execution_id='old', exit_confirmed=True)
    rig.confirm(stamp=rig.now+1., exit_confirmed=True)
    rig.confirm(stamp=rig.now-2., exit_confirmed=True)
    assert not rig.tick().feedback.hold_confirmed
    rig.confirm()  # Same execution's physical safe hold; still mid-stair.
    out = rig.tick()
    assert out.feedback.hold_confirmed and not out.feedback.exit_confirmed
    assert out.gait_request is None
    rig.step(.01)
    assert rig.accept(mode='flat')
    rig.confirm(exit_confirmed=True)
    out = rig.tick()
    assert out.gait_request == 12290
    rig.gait = 12290
    for _ in range(6):
        out = rig.step()
        assert rig.accept(mode='flat')
    assert rig.tick().feedback.exit_confirmed
    # External flags alone cannot override body motion or active IMU readings.
    out = rig.step(.01, gyro=.2)
    assert not out.feedback.stable and not out.feedback.hold_confirmed


@pytest.mark.parametrize('reading', [{'estop': True}, {'charge': 1}, {'state': 3}])
def test_does_not_fight_other_controller_when_motion_authority_is_lost(reading):
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    assert rig.tick().publish_velocity
    out = rig.step(.05, **reading)
    assert out.vx == 0. and not out.publish_velocity and out.gait_request is None


def test_disable_stops_until_native_stability_then_stops_publishing():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    rig.tick()
    rig.step(.05, vx=.2)
    assert rig.adapter.enable(False, rig.now, rig.steady)[0]
    out = rig.tick()
    assert out.publish_velocity and out.vx == out.wz == 0.
    for _ in range(5):
        out = rig.step()
    assert out.feedback.stable and not out.publish_velocity and not rig.adapter.enabled


def test_hold_after_velocity_cannot_reuse_pre_motion_stationary_samples():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    assert rig.tick().vx == .2
    assert rig.accept()  # Revoke before the next native MotionInfo/IMU frame arrives.
    out = rig.tick()
    assert not out.feedback.stable and not out.feedback.hold_confirmed
    rig.now += .36
    rig.steady += .36
    out = rig.tick()
    assert not out.feedback.hold_confirmed  # The old zero-velocity frame is still fresh.
    for _ in range(5):
        rig.step(.08)
        assert rig.accept()
    assert not rig.tick().feedback.hold_confirmed  # Only .32 s of new observations.
    rig.step(.08)
    assert rig.tick().feedback.hold_confirmed


def test_enable_epoch_and_first_hold_prevent_delayed_track_authorization():
    rig = Rig()
    assert rig.enable(hold=False)[0]
    assert not rig.accept(behavior='TRACK')
    old_stamp = rig.now-.1
    assert rig.accept()
    rig.step(.01)
    assert not rig.accept(behavior='TRACK', stamp=old_stamp, valid_until=rig.now+.2)
    assert rig.accept(behavior='TRACK')


def test_each_execution_requires_its_own_initial_hold_after_previous_release():
    rig = Rig()
    assert rig.enable()[0]
    assert rig.tick().feedback.hold_confirmed
    assert not rig.accept(behavior='TRACK', execution_id='next-run')
    assert rig.tick().feedback.execution_id == 'run'
    assert rig.tick().diagnostics['last_rejection'] == 'INITIAL_HOLD_REQUIRED'
    assert rig.accept(execution_id='next-run')
    rig.step(.01)
    assert rig.accept(behavior='TRACK', execution_id='next-run')
    assert rig.tick().vx == .2


@pytest.mark.parametrize('source,code', [
    ('estop_active', 'ESTOP_ACTIVE'), ('charge_active', 'CHARGING_OR_ERROR'),
    ('charge_error', 'CHARGING_OR_ERROR'), ('motion_state', 'MOTION_STATE'),
    ('location_abnormal', 'LOCATION_ABNORMAL'), ('motion_stale', 'MOTION_STALE'),
    ('imu_stale', 'IMU_STALE'), ('location_stale', 'LOCATION_STALE'),
    ('estop_stale', 'ESTOP_STALE'), ('charge_stale', 'CHARGE_STALE'),
    ('motion_nan', 'MOTION_STALE'), ('imu_nan', 'IMU_STALE'),
])
def test_transient_native_fault_cannot_revive_old_track_without_reauthorization(source, code):
    rig = Rig(command_max_ttl=2., motion_timeout=.1, imu_timeout=.1,
              location_timeout=.1, safety_timeout=.1)
    assert rig.enable()[0]
    track = rig.command('TRACK', valid_until=rig.now+1.5)
    assert rig.adapter.accept(track, rig.now, rig.steady)
    assert rig.tick().vx == .2
    # One missing stream becomes stale while all other native streams advance.
    # The command's independent lease is deliberately still live throughout.
    rig.now += .15
    rig.steady += .15
    a = rig.adapter
    if source != 'motion_stale':
        a.observe_motion(MotionObservation(rig.now, 3 if source == 'motion_state' else 17,
                         rig.gait, math.nan if source == 'motion_nan' else 0., 0., 0.))
    if source != 'imu_stale':
        a.observe_imu(rig.now, math.nan if source == 'imu_nan' else 0.)
    if source != 'location_stale':
        a.observe_location(rig.now, 1 if source == 'location_abnormal' else 0)
    if source != 'estop_stale':
        a.observe_estop(rig.steady, source == 'estop_active')
    if source != 'charge_stale':
        a.observe_charge(rig.steady, int(source == 'charge_active'), int(source == 'charge_error'))
    fault = rig.tick()
    assert fault.feedback.fault_code == code and not a.enabled
    assert fault.vx == fault.wz == 0.
    if source in ('estop_active', 'charge_active', 'charge_error', 'motion_state'):
        assert not fault.publish_velocity
    restored = rig.step(.05)
    assert rig.now < track.valid_until
    assert not restored.feedback.ready and restored.vx == restored.wz == 0.
    assert not rig.accept(behavior='TRACK')
    for _ in range(9):
        rig.step(.05)
    assert a.enable(True, rig.now, rig.steady)[0]
    assert rig.tick().vx == 0.
    assert not rig.accept(behavior='TRACK')  # Re-enable alone cannot revive context.
    assert rig.accept()
    rig.step(.01)
    assert rig.accept(behavior='TRACK') and rig.tick().vx == .2


def test_confirmation_callback_uses_current_time_without_retiming_feedback():
    rig = Rig()
    rig.gait = 12291
    assert rig.enable(hold=False)[0] and rig.accept(mode='stair')
    for _ in range(5):
        rig.step()
        assert rig.accept(mode='stair')
    message = MotionFeedback(rig.now+.01, 'run', 'AB', 'stair',
                             stable=True, hold_confirmed=True)
    rig.adapter.confirm(message, rig.now+.01)
    rig.now += .02
    rig.steady += .02
    rig.feed()
    assert rig.tick().feedback.hold_confirmed
    rig.now += 1.01
    rig.steady += 1.01
    rig.feed()
    assert not rig.tick().feedback.hold_confirmed


def test_gait_ack_cannot_use_an_observation_from_before_switch_request():
    rig = Rig()
    assert rig.enable()[0]
    rig.step(.01)
    assert rig.accept(mode='stair')
    request_time = rig.now
    assert rig.tick().gait_request == 12291
    # Delayed native sample is newer than the last received frame, but older
    # than the actual switch send time. It must not acknowledge this request.
    rig.adapter._motion = MotionObservation(request_time-.001, 17, 12291, 0., 0., 0.)
    assert rig.tick().diagnostics['gait_pending']
    rig.gait = 12291
    assert not rig.step(.01).diagnostics['gait_pending']


def test_pending_gait_cannot_release_canceled_execution_or_finish_disable():
    rig = Rig()
    rig.gait = 4097
    assert rig.enable(hold=False)[0] and rig.accept()
    for _ in range(8):
        out = rig.step(.1)
        assert rig.accept()
        if out.gait_request is not None:
            break
    assert out.gait_request == 12290
    # A cancel is still HOLD, but the old basic gait remaining stationary does
    # not acknowledge the outstanding physical switch to navigation-flat.
    for _ in range(8):
        out = rig.step(.1)
        assert rig.accept(phase='canceling')
        assert out.diagnostics['gait_pending']
        assert not out.feedback.stable and not out.feedback.hold_confirmed
        assert not out.feedback.exit_confirmed
        assert out.gait_request is None
    rig.step(.01)
    assert not rig.accept(execution_id='next', segment_id='BC')
    assert rig.tick().diagnostics['last_rejection'] == 'PREVIOUS_EXECUTION_NOT_RELEASED'
    assert rig.adapter.enable(False, rig.now, rig.steady)[0]
    for _ in range(6):
        out = rig.step(.1)
        assert out.diagnostics['stopping'] and out.publish_velocity
        assert out.vx == out.wz == 0.
        assert not out.feedback.hold_confirmed
    rig.gait = 12290
    out = rig.step(.01)
    acknowledged_at = rig.now
    assert not out.diagnostics['gait_pending']
    assert not out.feedback.stable and not out.feedback.hold_confirmed
    assert out.diagnostics['stopping'] and out.publish_velocity
    # Even after ACK, the ACK sample itself and old quiet history are not a
    # post-switch stability window. Both native streams must advance anew.
    for _ in range(4):
        out = rig.step(.05)
        assert not out.feedback.hold_confirmed
        assert out.diagnostics['stopping'] and out.publish_velocity
    for _ in range(6):
        out = rig.step(.05)
        if out.feedback.hold_confirmed:
            break
    assert out.feedback.hold_confirmed
    assert rig.now-acknowledged_at >= rig.adapter.config.stable_seconds
    assert not out.diagnostics['stopping'] and not out.publish_velocity


def test_gait_ack_requires_new_motion_and_imu_before_accumulating_stability():
    rig = Rig()
    assert rig.enable()[0]
    rig.step(.01)
    assert rig.accept(mode='stair') and rig.tick().gait_request == 12291
    rig.gait = 12291
    out = rig.step(.01)
    ack = rig.now
    assert not out.feedback.stable
    # Motion advances for longer than the dwell, but the IMU is still the ACK
    # frame and fresh according to its ordinary timeout. It cannot prove hold.
    for _ in range(4):
        rig.now += .1
        rig.steady += .1
        rig.adapter.observe_motion(MotionObservation(rig.now, 17, rig.gait, 0., 0., 0.))
        rig.adapter.observe_location(rig.now, 0)
        rig.adapter.observe_estop(rig.steady, False)
        rig.adapter.observe_charge(rig.steady, 0)
        assert rig.accept(mode='stair')
        rig.confirm()
        out = rig.tick()
        assert out.feedback.ready and not out.feedback.stable
        assert not out.feedback.hold_confirmed
    assert rig.now-ack > rig.adapter.config.stable_seconds
    for _ in range(4):
        out = rig.step(.08)
        assert rig.accept(mode='stair')
        rig.confirm()
        assert not rig.tick().feedback.hold_confirmed
    for _ in range(2):
        rig.step(.08)
        assert rig.accept(mode='stair')
        rig.confirm()
    assert rig.tick().feedback.hold_confirmed


def test_late_gait_ack_after_timeout_does_not_release_or_rearm():
    rig = Rig(gait_timeout=.2)
    assert rig.enable()[0]
    rig.step(.01)
    assert rig.accept(mode='stair') and rig.tick().gait_request == 12291
    out = rig.step(.25)
    assert out.feedback.fault_code == 'GAIT_TIMEOUT'
    rig.gait = 12291
    for _ in range(6):
        out = rig.step(.1)
        rig.accept(mode='stair')
        rig.confirm(exit_confirmed=True)
        assert out.diagnostics['gait_pending']
        assert out.feedback.fault_code == 'GAIT_TIMEOUT'
        assert not out.feedback.hold_confirmed and not out.feedback.exit_confirmed
        assert out.gait_request is None and out.vx == 0.
    assert not rig.adapter.enable(True, rig.now, rig.steady)[0]
    rig.step(.01)
    assert not rig.accept(execution_id='next', segment_id='BC', mode='stair')


def inject_native_fault(rig, source):
    a = rig.adapter
    if source == 'estop':
        a.observe_estop(rig.steady, True)
    elif source == 'charge':
        a.observe_charge(rig.steady, 1)
    elif source == 'charge_error':
        a.observe_charge(rig.steady, 0, 7)
    elif source == 'state':
        a.observe_motion(MotionObservation(rig.now, 3, rig.gait, 0., 0., 0.))
    elif source == 'gait':
        a.observe_motion(MotionObservation(rig.now, 17, 0x4001, 0., 0., 0.))
    elif source == 'location':
        a.observe_location(rig.now, 1)
    elif source == 'motion_nan':
        a.observe_motion(MotionObservation(rig.now, 17, rig.gait, math.nan, 0., 0.))
    elif source == 'imu_nan':
        a.observe_imu(rig.now, math.nan)
    elif source == 'motion_schema':
        a.observe_motion(None)
    elif source == 'imu_schema':
        a.observe_imu(rig.now, None)
    elif source == 'location_schema':
        a.observe_location(rig.now, None)
    elif source == 'estop_schema':
        a.observe_estop(rig.steady, None)
    elif source == 'charge_schema':
        a.observe_charge(rig.steady, None)
    else:
        raise AssertionError(source)


@pytest.mark.parametrize('source,code', [
    ('estop', 'ESTOP_ACTIVE'), ('charge', 'CHARGING_OR_ERROR'),
    ('charge_error', 'CHARGING_OR_ERROR'), ('state', 'MOTION_STATE'),
    ('gait', 'GAIT_UNSUPPORTED'),
    ('location', 'LOCATION_ABNORMAL'), ('motion_nan', 'MOTION_STALE'),
    ('imu_nan', 'IMU_STALE'), ('motion_schema', 'MOTION_STALE'),
    ('imu_schema', 'IMU_STALE'), ('location_schema', 'LOCATION_STALE'),
    ('estop_schema', 'ESTOP_STALE'), ('charge_schema', 'CHARGE_STALE'),
])
def test_unsafe_callback_then_healthy_callback_before_tick_retains_revocation(source, code):
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    track = rig.adapter._command
    assert rig.tick().vx == .2
    rig.now += .01
    rig.steady += .01
    inject_native_fault(rig, source)
    assert not rig.adapter.enabled  # No timer tick has processed this edge yet.
    rig.now += .01
    rig.steady += .01
    rig.feed()  # Recovery packets overwrite every latest-value cache before tick.
    assert rig.now < track.valid_until
    out = rig.tick()
    assert out.feedback.fault_code == code and not out.feedback.ready
    assert out.vx == out.wz == 0. and not out.feedback.hold_confirmed
    assert not rig.accept(behavior='TRACK')
    for _ in range(10):
        rig.step(.05)
    assert not rig.tick().feedback.ready and rig.tick().vx == 0.
    assert rig.adapter.enable(True, rig.now, rig.steady)[0]
    assert not rig.accept(behavior='TRACK')
    assert rig.accept()
    rig.step(.01)
    assert rig.accept(behavior='TRACK') and rig.tick().vx == .2


def test_old_unsafe_observations_do_not_revoke_newer_healthy_state():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK')
    assert rig.tick().vx == .2
    old = rig.now-.01
    rig.adapter.observe_motion(MotionObservation(old, 3, rig.gait, 0., 0., 0.))
    rig.adapter.observe_motion(MotionObservation(old, 17, rig.gait, math.nan, 0., 0.))
    rig.adapter.observe_imu(old, math.nan)
    rig.adapter.observe_location(old, 1)
    rig.adapter.observe_estop(rig.steady-.01, True)
    rig.adapter.observe_charge(rig.steady-.01, 1)
    out = rig.tick()
    assert rig.adapter.enabled and out.feedback.ready
    assert out.feedback.fault_code == '' and out.vx == .2


def test_normal_turning_and_new_samples_between_ticks_do_not_latch_a_fault():
    rig = Rig()
    assert rig.enable()[0] and rig.accept(behavior='TRACK', wz=.2)
    rig.adapter.observe_imu(rig.now+.01, .2)
    rig.adapter.observe_motion(MotionObservation(rig.now+.01, 17, rig.gait, .2, 0., .2))
    assert rig.adapter.enabled  # No comparison against the previous tick's ROS time.
    rig.now += .02
    rig.steady += .02
    out = rig.tick()
    assert out.feedback.ready and (out.vx, out.wz) == (.2, .2)
    assert not out.feedback.stable


@pytest.mark.parametrize('source', ['estop', 'charge', 'state', 'location', 'motion_nan', 'imu_nan'])
def test_unenabled_startup_can_recover_from_unsafe_native_data(source):
    rig = Rig()
    rig.now += .01
    rig.steady += .01
    inject_native_fault(rig, source)
    rig.step(.01)
    for _ in range(5):
        rig.step(.1)
    assert rig.adapter.enable(True, rig.now, rig.steady)[0]


@pytest.mark.parametrize('values', [{'normal_location_status': 2}, {'motion_timeout': 0.},
                                   {'max_vx': math.nan}, {'flat_gait': 12291}, {'dry_run': 'false'}])
def test_invalid_configuration_is_rejected(values):
    with pytest.raises(ValueError):
        M20Config(**values)
