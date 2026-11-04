"""Source-time, status-version, and 3-D support reference conversion checks."""
import math
from types import SimpleNamespace

import pytest

from stair_nav.core.localization import (
    LocalizationGate, compose_reference_pose, flat_ground_projection_transform,
    normalized_quaternion, timestamp_seconds,
)


def test_full_tilted_body_transform_preserves_support_geometry():
    # A 90° body pitch rotates its local downward axis into map -X. Simply
    # subtracting height in map Z would give the wrong support position.
    q = (0., math.sqrt(.5), 0., math.sqrt(.5))
    xyz, orientation = compose_reference_pose((2., 3., 4.), q, (0., 0., -1.), (0., 0., 0., 1.))
    assert xyz == pytest.approx((1., 3., 4.))
    assert orientation == pytest.approx(q)


def test_composition_combines_body_and_reference_orientation():
    half_turn = (0., 0., 1., 0.)
    xyz, orientation = compose_reference_pose((0., 0., 0.), half_turn,
                                             (1., 0., 0.), half_turn)
    assert xyz == pytest.approx((-1., 0., 0.))
    assert abs(orientation[3]) == pytest.approx(1.)
    assert orientation[:3] == pytest.approx((0., 0., 0.))


def test_flat_ground_projection_stays_vertically_below_a_tilted_body():
    yaw, pitch = .6, .4
    q = (-math.sin(yaw/2)*math.sin(pitch/2), math.cos(yaw/2)*math.sin(pitch/2),
         math.sin(yaw/2)*math.cos(pitch/2), math.cos(yaw/2)*math.cos(pitch/2))
    position = (2., 3., .55)
    translation, rotation = flat_ground_projection_transform(position, q, .05)
    xyz, orientation = compose_reference_pose(position, q, translation, rotation)
    assert xyz == pytest.approx((2., 3., .05))
    assert orientation == pytest.approx((0., 0., math.sin(yaw/2), math.cos(yaw/2)))
    # A fixed offset along tilted body -Z would incorrectly move map X/Y.
    assert abs(translation[0]) > 0


@pytest.mark.parametrize('body_z', [.55, .65])
def test_flat_ground_projection_uses_configured_ground_instead_of_fixed_body_height(body_z):
    translation, rotation = flat_ground_projection_transform(
        (2., 3., body_z), (0., 0., 0., 1.), .05)
    assert translation == pytest.approx((0., 0., .05-body_z))
    assert rotation == pytest.approx((0., 0., 0., 1.))


@pytest.mark.parametrize('ground_z', [math.nan, math.inf, True])
def test_flat_ground_projection_rejects_invalid_ground_height(ground_z):
    with pytest.raises(ValueError):
        flat_ground_projection_transform((0., 0., .55), (0., 0., 0., 1.), ground_z)


@pytest.mark.parametrize('quaternion', [(0., 0., 0., 0.), (0., 0., 0., 2.),
                                        (math.nan, 0., 0., 1.)])
def test_invalid_quaternion_is_rejected(quaternion):
    with pytest.raises(ValueError):
        normalized_quaternion(quaternion)


@pytest.mark.parametrize('position,translation', [((math.inf, 0., 0.), (0., 0., 0.)),
                                                 ((0., 0., 0.), (0., math.nan, 0.))])
def test_nonfinite_position_or_transform_is_rejected(position, translation):
    with pytest.raises(ValueError):
        compose_reference_pose(position, (0., 0., 0., 1.), translation, (0., 0., 0., 1.))


def test_unknown_firmware_status_mapping_blocks_even_with_fresh_status():
    gate = LocalizationGate()
    assert not gate.update_status(0, 10., 10.)
    assert not gate.ready(10.)
    assert 'firmware' in gate.reason


def test_disabled_status_gate_keeps_pose_age_and_order_checks():
    gate = LocalizationGate(require_status=False)
    assert gate.ready(10.)
    assert gate.accept_pose_stamp(10., 10.)
    assert not gate.accept_pose_stamp(9., 10.)
    assert not gate.accept_pose_stamp(10.2, 10.)
    assert not gate.accept_pose_stamp(math.nan, 10.)
    assert not gate.accept_pose_stamp(10., math.nan)
    gate.mark_published(10.)
    assert not gate.accept_pose_stamp(10., 10.1)
    assert gate.accept_pose_stamp(10.1, 10.1)
    assert gate.update_status(5, 0., 10.1)
    assert gate.update_status(None, math.nan, 10.1)
    assert gate.status is None
    assert gate.ready(10.1)


@pytest.mark.parametrize('value', ['false', 0, None])
def test_status_gate_setting_requires_a_boolean(value):
    with pytest.raises(ValueError, match='require_location_status'):
        LocalizationGate(require_status=value)


@pytest.mark.parametrize('normal,other', [(0, 1), (1, 0)])
def test_only_explicitly_selected_normal_code_is_accepted(normal, other):
    gate = LocalizationGate(normal_status=normal)
    assert gate.update_status(normal, 10., 10.)
    assert gate.accept_pose_stamp(10., 10.)
    assert not gate.update_status(other, 10.1, 10.1)
    assert not gate.accept_pose_stamp(10.1, 10.1)


def test_status_age_matches_two_hertz_source_and_does_not_refresh_on_pose():
    gate = LocalizationGate(normal_status=1)
    gate.update_status(1, 10., 10.)
    assert gate.accept_pose_stamp(11.4, 11.4)
    assert not gate.accept_pose_stamp(11.6, 11.6)
    assert 'status source timestamp' in gate.reason


def test_out_of_order_status_cannot_restore_old_good_quality():
    gate = LocalizationGate(normal_status=1)
    gate.update_status(3, 10., 10.)
    assert not gate.update_status(1, 9.9, 10.)
    assert not gate.ready(10.)


def test_stale_future_and_duplicate_pose_are_rejected_without_restamping():
    gate = LocalizationGate(normal_status=1)
    gate.update_status(1, 10., 10.)
    assert not gate.accept_pose_stamp(9., 10.)
    assert not gate.accept_pose_stamp(10.2, 10.)
    assert gate.accept_pose_stamp(10., 10.)
    gate.mark_published(10.)
    assert not gate.accept_pose_stamp(10., 10.1)
    assert not gate.accept_pose_stamp(9.99, 10.1)
    assert gate.accept_pose_stamp(10.1, 10.1)


def test_failed_tf_does_not_consume_pose_timestamp():
    gate = LocalizationGate(normal_status=1)
    gate.update_status(1, 10., 10.)
    assert gate.accept_pose_stamp(10., 10.)
    # Caller failed to obtain TF and did not call mark_published.
    assert gate.accept_pose_stamp(10., 10.05)


def test_future_status_invalidates_previous_good_quality():
    gate = LocalizationGate(normal_status=1)
    gate.update_status(1, 10., 10.)
    assert not gate.update_status(1, 30., 10.)
    assert not gate.ready(10.)
    assert gate.update_status(1, 10.1, 10.1)


@pytest.mark.parametrize('header', [SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=34)),
                                   SimpleNamespace(timestamp=SimpleNamespace(sec=12, nsec=34))])
def test_vendor_timestamp_schemas_are_read_without_receipt_time(header):
    assert timestamp_seconds(header) == pytest.approx(12.000000034)


@pytest.mark.parametrize('header', [SimpleNamespace(),
                                   SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=-1)),
                                   SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=1_000_000_000)),
                                   SimpleNamespace(stamp=SimpleNamespace(sec=True, nanosec=0))])
def test_invalid_vendor_timestamp_is_rejected(header):
    with pytest.raises((AttributeError, ValueError)):
        timestamp_seconds(header)
