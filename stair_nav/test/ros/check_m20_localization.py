#!/usr/bin/env python3
"""Real ROS message/TF conversion checks; no DDS graph or robot is started."""
import math
from collections import deque
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer

from stair_nav.core.localization import LocalizationGate
from stair_nav.m20_localization_node import M20Localization


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ReferenceBroadcaster(Publisher):
    sendTransform = Publisher.publish


class LocalizationHarness(SimpleNamespace):
    retry_pending = M20Localization.retry_pending
    publish_test_ground_reference = M20Localization.publish_test_ground_reference


def harness(require_status=True, test_projection=False):
    gate = LocalizationGate(normal_status=1 if require_status else -1,
                            require_status=require_status)
    gate.update_status(1, 10., 10.)
    return LocalizationHarness(
        gate=gate, now=lambda: 10.1, route_frame='map',
        reference_frame='navigation_ground_reference', buffer=Buffer(),
        test_reference_broadcaster=ReferenceBroadcaster() if test_projection else None,
        test_ground_z=.05,
        publisher=Publisher(), last_odom_stamp=None, last_received_stamp=None,
        max_pending_odometry=32, pending_odometry=deque())


def observation(stamp=None):
    message = Odometry()
    message.header.frame_id = 'map'
    message.header.stamp = stamp if stamp is not None else Time(sec=10, nanosec=123)
    message.child_frame_id = 'base_link'
    message.pose.pose.position.x = 2.
    message.pose.pose.position.y = 3.
    message.pose.pose.position.z = 4.
    message.pose.pose.orientation.y = math.sqrt(.5)
    message.pose.pose.orientation.w = math.sqrt(.5)
    return message


def support_transform(stamp):
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'navigation_ground_reference'
    transform.header.stamp = stamp
    transform.transform.translation.z = -1.
    transform.transform.rotation.w = 1.
    return transform


def status(code, second, nanosecond=0):
    return SimpleNamespace(
        meta=SimpleNamespace(stamp=Time(sec=second, nanosec=nanosecond)),
        data=SimpleNamespace(total_status=code))


class LocalizationConversionTests(unittest.TestCase):
    def test_test_projection_publishes_source_timed_body_to_ground_tf_and_pose(self):
        node, message = harness(require_status=False, test_projection=True), observation()
        q = message.pose.pose.orientation
        q.y = 0.
        q.z, q.w = math.sin(.3), math.cos(.3)
        message.pose.pose.position.z = .55
        M20Localization.on_odom(node, message)
        self.assertEqual(len(node.test_reference_broadcaster.messages), 1)
        transform = node.test_reference_broadcaster.messages[0]
        self.assertEqual(transform.header.frame_id, 'base_link')
        self.assertEqual(transform.child_frame_id, node.reference_frame)
        self.assertEqual(transform.header.stamp, message.header.stamp)
        self.assertAlmostEqual(transform.transform.translation.z, -.5)
        self.assertEqual(len(node.publisher.messages), 1)
        output = node.publisher.messages[0]
        self.assertEqual(output.header, message.header)
        self.assertAlmostEqual(output.pose.position.x, 2.)
        self.assertAlmostEqual(output.pose.position.y, 3.)
        self.assertAlmostEqual(output.pose.position.z, .05)
        self.assertAlmostEqual(output.pose.orientation.z, math.sin(.3))
        self.assertFalse(node.pending_odometry)

    def test_test_projection_follows_body_height_and_does_not_replay_duplicates(self):
        node = harness(require_status=False, test_projection=True)
        for index, body_z in enumerate((.55, .65)):
            message = observation(Time(sec=10, nanosec=index*100_000_000))
            message.pose.pose.position.z = body_z
            node.now = lambda index=index: 10. + index*.1 + .01
            M20Localization.on_odom(node, message)
            M20Localization.on_odom(node, message)
        self.assertEqual(len(node.test_reference_broadcaster.messages), 2)
        self.assertEqual(len(node.publisher.messages), 2)
        for output in node.publisher.messages:
            self.assertAlmostEqual(output.pose.position.x, 2.)
            self.assertAlmostEqual(output.pose.position.y, 3.)
            self.assertAlmostEqual(output.pose.position.z, .05)

    def test_test_projection_does_not_broadcast_invalid_or_stale_odometry(self):
        for invalid in ('stale', 'frame', 'quaternion', 'missing_body', 'position'):
            with self.subTest(invalid=invalid):
                node = harness(require_status=False, test_projection=True)
                message = observation()
                if invalid == 'stale':
                    message.header.stamp.sec = 9
                elif invalid == 'frame':
                    message.header.frame_id = 'odom'
                elif invalid == 'quaternion':
                    message.pose.pose.orientation.y = 0.
                    message.pose.pose.orientation.w = 0.
                elif invalid == 'missing_body':
                    message.child_frame_id = ''
                else:
                    message.pose.pose.position.x = math.nan
                M20Localization.on_odom(node, message)
                self.assertFalse(node.test_reference_broadcaster.messages)
                self.assertFalse(node.publisher.messages)

    def test_corrected_odometry_does_not_need_vendor_status_when_disabled(self):
        node, message = harness(require_status=False), observation()
        self.assertIsNone(node.gate.status)
        M20Localization.on_status(node, SimpleNamespace())
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        M20Localization.on_odom(node, message)
        self.assertEqual(len(node.publisher.messages), 1)
        self.assertEqual(node.publisher.messages[0].header, message.header)

    def test_disabling_status_keeps_pose_and_tf_validation(self):
        for invalid in ('stale', 'frame', 'quaternion', 'missing_tf'):
            with self.subTest(invalid=invalid):
                node, message = harness(require_status=False), observation()
                if invalid != 'missing_tf':
                    node.buffer.set_transform(support_transform(message.header.stamp), 'test')
                if invalid == 'stale':
                    message.header.stamp.sec = 9
                elif invalid == 'frame':
                    message.header.frame_id = 'odom'
                elif invalid == 'quaternion':
                    message.pose.pose.orientation.y = 0.
                    message.pose.pose.orientation.w = 0.
                M20Localization.on_odom(node, message)
                self.assertFalse(node.publisher.messages)

    def test_timestamped_tf_uses_full_3d_pose_and_preserves_source_stamp(self):
        node, message = harness(), observation()
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        M20Localization.on_odom(node, message)
        self.assertEqual(len(node.publisher.messages), 1)
        output = node.publisher.messages[0]
        self.assertEqual(output.header, message.header)
        self.assertAlmostEqual(output.pose.position.x, 1.)
        self.assertAlmostEqual(output.pose.position.y, 3.)
        self.assertAlmostEqual(output.pose.position.z, 4.)
        self.assertAlmostEqual(output.pose.orientation.y, math.sqrt(.5))
        self.assertAlmostEqual(output.pose.orientation.w, math.sqrt(.5))
        self.assertEqual(node.gate.last_pose_stamp, 10.000000123)

    def test_timer_retries_after_tf_without_republishing_the_odom(self):
        node, message = harness(), observation()
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)
        self.assertIsNone(node.gate.last_pose_stamp)
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.retry_pending()
        self.assertEqual(len(node.publisher.messages), 1)
        self.assertFalse(node.pending_odometry)
        node.retry_pending()
        self.assertEqual(len(node.publisher.messages), 1)

    def test_wrong_map_frame_is_not_silently_relabelled(self):
        node, message = harness(), observation()
        message.header.frame_id = 'odom'
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)
        self.assertIn('route_frame', node.gate.reason)

    def test_lost_localization_does_not_publish_fresh_body_pose(self):
        node, message = harness(), observation()
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.gate.update_status(3, 10.05, 10.1)
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)

    def test_transform_from_another_time_cannot_substitute_for_current_support(self):
        node, message = harness(), observation()
        node.buffer.set_transform(support_transform(Time(sec=9)), 'test')
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)

    def test_body_reference_cannot_be_mistaken_for_support_reference(self):
        node, message = harness(), observation()
        message.child_frame_id = node.reference_frame
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)

    def test_continuous_odom_before_matching_tf_still_publishes_each_frame(self):
        node = harness()
        stamps = []
        for i in range(5):
            stamp = Time(sec=10, nanosec=i*100_000_000)
            node.now = lambda i=i: 10. + i*.1 + .01
            message = observation(stamp)
            M20Localization.on_odom(node, message)
            self.assertEqual(len(node.publisher.messages), i)
            # TF for every frame arrives later than its ODOM, as with a
            # support estimator that subscribes to that ODOM itself.
            node.buffer.set_transform(support_transform(stamp), 'test')
            node.now = lambda i=i: 10. + i*.1 + .02
            node.retry_pending()  # The production timer callback; no ODOM replay.
            stamps.append(stamp)
            self.assertEqual(len(node.publisher.messages), i+1)
        self.assertEqual([value.header.stamp for value in node.publisher.messages], stamps)

    def test_multiple_pending_frames_are_not_overwritten_and_publish_in_order(self):
        node = harness()
        stamps = [Time(sec=10, nanosec=i*100_000_000) for i in range(3)]
        for i, stamp in enumerate(stamps):
            node.now = lambda i=i: 10. + i*.1 + .01
            M20Localization.on_odom(node, observation(stamp))
        self.assertEqual(len(node.pending_odometry), 3)
        for stamp in stamps:
            node.buffer.set_transform(support_transform(stamp), 'test')
        node.now = lambda: 10.25
        node.retry_pending()
        self.assertEqual([value.header.stamp for value in node.publisher.messages], stamps)
        self.assertFalse(node.pending_odometry)

    def test_tf_after_pose_expiry_cannot_renew_the_original_observation(self):
        node, message = harness(), observation()
        M20Localization.on_odom(node, message)
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.now = lambda: 10.6
        node.retry_pending()
        self.assertFalse(node.publisher.messages)
        self.assertFalse(node.pending_odometry)

    def test_bad_quality_drops_waiting_samples_even_if_tf_arrives_after_recovery(self):
        node, message = harness(), observation()
        M20Localization.on_odom(node, message)
        M20Localization.on_status(node, status(3, 10, 50_000_000))
        self.assertFalse(node.pending_odometry)
        node.now = lambda: 10.12
        M20Localization.on_status(node, status(1, 10, 100_000_000))
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.retry_pending()
        self.assertFalse(node.publisher.messages)
        M20Localization.on_odom(node, message)
        self.assertFalse(node.publisher.messages)
        current = observation(Time(sec=10, nanosec=200_000_000))
        node.now = lambda: 10.21
        node.buffer.set_transform(support_transform(current.header.stamp), 'test')
        M20Localization.on_odom(node, current)
        self.assertEqual(len(node.publisher.messages), 1)
        self.assertEqual(node.publisher.messages[0].header.stamp, current.header.stamp)

    def test_out_of_order_or_duplicate_pending_and_published_frames_are_not_replayed(self):
        node = harness()
        message = observation(Time(sec=10, nanosec=200_000_000))
        node.now = lambda: 10.25
        M20Localization.on_odom(node, message)
        M20Localization.on_odom(node, observation(Time(sec=10, nanosec=100_000_000)))
        M20Localization.on_odom(node, message)
        self.assertEqual(len(node.pending_odometry), 1)
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.retry_pending()
        M20Localization.on_odom(node, message)
        node.retry_pending()
        self.assertEqual(len(node.publisher.messages), 1)

    def test_queue_bound_keeps_older_pending_frames_instead_of_chasing_latest_tf(self):
        node = harness()
        node.max_pending_odometry = 2
        stamps = [Time(sec=10, nanosec=i*100_000_000) for i in range(4)]
        for i, stamp in enumerate(stamps):
            node.now = lambda i=i: 10. + i*.1 + .01
            M20Localization.on_odom(node, observation(stamp))
            self.assertLessEqual(len(node.pending_odometry), 2)
        self.assertEqual([value.header.stamp for value in node.pending_odometry], stamps[:2])
        for stamp in stamps[:2]:
            node.buffer.set_transform(support_transform(stamp), 'test')
        node.retry_pending()
        self.assertEqual([value.header.stamp for value in node.publisher.messages], stamps[:2])
        current = observation(Time(sec=10, nanosec=400_000_000))
        node.now = lambda: 10.41
        node.buffer.set_transform(support_transform(current.header.stamp), 'test')
        M20Localization.on_odom(node, current)
        self.assertEqual(len(node.publisher.messages), 3)

    def test_invalid_new_odometry_cannot_leave_old_pending_position_to_be_published(self):
        node, message = harness(), observation()
        M20Localization.on_odom(node, message)
        invalid = observation(Time(sec=10, nanosec=100_000_000))
        invalid.pose.pose.position.x = math.nan
        M20Localization.on_odom(node, invalid)
        self.assertFalse(node.pending_odometry)
        node.buffer.set_transform(support_transform(message.header.stamp), 'test')
        node.retry_pending()
        self.assertFalse(node.publisher.messages)


if __name__ == '__main__':
    unittest.main(verbosity=2)
