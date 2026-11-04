#!/usr/bin/env python3
"""Vendor field conversion checks; no robot, DDS participant or drdds substitute.

SimpleNamespace fixtures exercise the two published header layouts. They are
never registered as ROS types or used on the wire.
"""
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from stair_nav.core.models import MotionFeedback
from stair_nav.core.m20_motion import M20Output
from stair_nav.m20_motion_node import M20MotionNode, motion_observation, native_header
from stair_nav_interfaces.msg import MotionStatus


def native():
    return S(header=S(frame_id=0, stamp=S(sec=0, nanosec=0)),
             data=S(x_vel=0., y_vel=0., yaw_vel=0., gait=0))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class NativeInterfaceTests(unittest.TestCase):
    def test_native_sequence_is_not_a_tf_frame_and_timestamp_is_preserved(self):
        message = native_header(native(), 19, 12.25)
        self.assertEqual(message.header.frame_id, 19)
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (12, 250000000))

    def test_guide_timestamp_layout_uses_its_actual_fields(self):
        message = S(header=S(frame_id=0, timestamp=S(sec=0, nsec=0)))
        native_header(message, 3, 7.5)
        self.assertEqual(message.header.timestamp.nsec, 500000000)
        with self.assertRaises(ValueError):
            native_header(S(header=S(frame_id=0)), 1, 1.)

    def test_motion_info_uses_verified_nested_state_and_gait(self):
        message = S(header=S(stamp=S(sec=10, nanosec=500000000)),
                    data=S(motion_state=S(state=17), gait_state=S(gait=12291),
                           vel_x=.2, vel_y=.03, vel_yaw=-.4))
        value = motion_observation(message)
        self.assertEqual((value.stamp, value.state, value.gait), (10.5, 17, 12291))
        self.assertEqual((value.vx, value.vy, value.wz), (.2, .03, -.4))
        del message.data.motion_state
        with self.assertRaises(AttributeError):
            motion_observation(message)

    def test_location_status_reads_meta_and_preserves_source_time(self):
        received = []
        node = S(adapter=S(observe_location=lambda *value: received.append(value)))
        message = S(meta=S(stamp=S(sec=10, nanosec=125000000)),
                    data=S(total_status=1))
        M20MotionNode.on_location(node, message)
        self.assertEqual(received, [(10.125, 1)])

    def test_wrong_location_status_schema_is_rejected(self):
        errors, received = [], []
        node = S(adapter=S(observe_location=lambda *value: received.append(value)),
                 invalid_input=lambda source, exc: errors.append(source))
        message = S(header=S(stamp=S(sec=10, nanosec=0)), data=S(total_status=1))
        M20MotionNode.on_location(node, message)
        self.assertEqual(errors, ['LOCATION_STATUS'])
        self.assertEqual(received, [])

    def test_estop_reads_value_with_receipt_time(self):
        received = []
        node = S(adapter=S(observe_estop=lambda *value: received.append(value)))
        with patch('stair_nav.m20_motion_node.time.monotonic', return_value=101.25):
            for value in (0, 1, -1):
                M20MotionNode.on_estop(node, S(value=value))
        self.assertEqual(received, [(101.25, False), (101.25, True), (101.25, True)])

    def test_wrong_estop_schema_is_rejected(self):
        errors, received = [], []
        node = S(adapter=S(observe_estop=lambda *value: received.append(value)),
                 invalid_input=lambda source, exc: errors.append(source))
        M20MotionNode.on_estop(node, S(data=0))
        self.assertEqual(errors, ['HES_STATUS'])
        self.assertEqual(received, [])

    def test_wrapper_dry_run_cannot_write_even_if_an_output_requests_motion(self):
        node = S(adapter=S(config=S(dry_run=True)), now=lambda: 10., frame='map',
                 sequence=0, nav=Publisher(), gaits=Publisher(), status=Publisher(),
                 diagnostic=Publisher(), types={'nav': native, 'gait': native}, last_input_error='')
        feedback = MotionFeedback(9.8, 'execution', 'AB', 'flat', ready=False)
        output = M20Output(feedback, .2, .4, True, 12290, {})
        M20MotionNode.publish_output(node, output)
        self.assertEqual(node.nav.messages, [])
        self.assertEqual(node.gaits.messages, [])
        self.assertEqual(len(node.status.messages), 1)
        self.assertEqual(node.status.messages[0].header.stamp.sec, 9)
        self.assertAlmostEqual(node.status.messages[0].header.stamp.nanosec * 1e-9, .8)

    def test_live_wrapper_maps_velocity_and_does_not_request_sideways_motion(self):
        node = S(adapter=S(config=S(dry_run=False)), now=lambda: 10., frame='map',
                 sequence=0, nav=Publisher(), gaits=Publisher(), status=Publisher(),
                 diagnostic=Publisher(), types={'nav': native, 'gait': native}, last_input_error='')
        output = M20Output(MotionFeedback(9.9, 'execution', 'AB', 'stair'), .2, -.4, True, None, {})
        M20MotionNode.publish_output(node, output)
        command = node.nav.messages[0]
        self.assertEqual((command.data.x_vel, command.data.y_vel, command.data.yaw_vel), (.2, 0., -.4))
        self.assertEqual(node.gaits.messages, [])

    def test_live_safety_parameters_cannot_be_silently_changed(self):
        self.assertFalse(M20MotionNode.startup_parameters_only([]).successful)

    def test_supervisor_fault_and_actual_callback_time_are_preserved(self):
        received = []
        node = S(frame='map', now=lambda: 10.05,
                 adapter=S(confirm=lambda feedback, now: received.append((feedback, now))))
        message = MotionStatus()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 10
        message.execution_id, message.segment_id = 'execution', 'AB'
        message.hold_confirmed = message.stable = message.exit_confirmed = True
        message.fault_code, message.fault_message = 'UNSAFE_CONTACT', 'Contact is uncertain'
        M20MotionNode.on_confirmation(node, message)
        self.assertEqual(received[0][0].fault_code, 'UNSAFE_CONTACT')
        self.assertEqual(received[0][1], 10.05)


if __name__ == '__main__':
    unittest.main()
