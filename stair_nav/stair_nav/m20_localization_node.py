"""M20 /ODOM + /LOCATION_STATUS → the calibrated navigation support pose."""
from copy import deepcopy
from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from stair_nav.core.localization import (
    LocalizationGate, compose_reference_pose, normalized_quaternion, timestamp_seconds,
)
from stair_nav.ros_utils import live_qos, pose_qos, seconds


class M20Localization(Node):
    def __init__(self):
        super().__init__('m20_localization')
        defaults = {
            'odom_topic': '/ODOM',
            'location_status_topic': '/LOCATION_STATUS',
            'location_status_type': 'drdds/msg/LocationStatus',
            'output_pose_topic': '/localization/pose',
            'diagnostic_topic': '/m20/localization_diagnostic',
            'normal_location_status': -1,
            'route_frame': 'map',
            'navigation_reference_frame': 'navigation_ground_reference',
            'odom_qos_reliability': 'best_effort',
            'status_qos_reliability': 'best_effort',
            'location_status_timeout': 1.5,
            'odom_timeout': .5,
            'future_tolerance': .1,
            'tf_retry_period': .05,
            'max_pending_odometry': 32,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.route_frame = values['route_frame']
        self.reference_frame = values['navigation_reference_frame']
        self.max_pending_odometry = values['max_pending_odometry']
        retry_period = values['tf_retry_period']
        if (type(self.max_pending_odometry) is not int or self.max_pending_odometry <= 0):
            raise ValueError('max_pending_odometry must be a positive integer')
        if (isinstance(retry_period, bool) or not isinstance(retry_period, (int, float)) or
                not math.isfinite(retry_period) or retry_period <= 0):
            raise ValueError('tf_retry_period must be finite and positive')
        if not self.route_frame or not self.reference_frame:
            raise ValueError('route_frame and navigation_reference_frame must be nonempty')
        if self.reference_frame == 'base_link':
            raise ValueError('base_link is a body pose; configure a calibrated support reference TF')
        self.gate = LocalizationGate(
            normal_status=values['normal_location_status'],
            pose_max_age=values['odom_timeout'],
            status_max_age=values['location_status_timeout'],
            future_tolerance=values['future_tolerance'])
        try:
            status_type = get_message(values['location_status_type'])
        except (ImportError, AttributeError, ModuleNotFoundError, ValueError) as exc:
            raise RuntimeError(
                'Install/source the robot firmware-matching drdds messages including '
                'LocationStatus; a similarly named SDK message package is insufficient') from exc
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PoseStamped, values['output_pose_topic'], live_qos(5))
        self.diagnostics = self.create_publisher(String, values['diagnostic_topic'], live_qos())
        self.create_subscription(Odometry, values['odom_topic'], self.on_odom,
                                 pose_qos(values['odom_qos_reliability']))
        self.create_subscription(status_type, values['location_status_topic'], self.on_status,
                                 pose_qos(values['status_qos_reliability']))
        self.last_diagnostic = None
        self.last_odom_stamp = None
        self.last_received_stamp = None
        self.pending_odometry = deque()
        # The support estimator may publish TF after its matching ODOM. Retry
        # without blocking the executor that must also service the TF listener.
        self.create_timer(retry_period, self.retry_pending)
        self.create_timer(.2, self.diagnose)
        self.add_on_set_parameters_callback(self.startup_parameters_only)
        self.get_logger().info('M20 localization requires healthy status and timestamped body→support TF')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def startup_parameters_only(parameters):
        return SetParametersResult(successful=False,
                                   reason='Localization adapter parameters are startup-only; restart with new configuration')

    def on_status(self, message):
        try:
            observed = timestamp_seconds(message.header)
            self.gate.update_status(message.data.total_status, observed, self.now())
            if not self.gate.ready(self.now()):
                self.pending_odometry.clear()
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.gate.status = None
            self.gate.reason = 'LOCATION_STATUS schema or timestamp is invalid'
            self.pending_odometry.clear()

    def on_odom(self, message):
        try:
            observed = seconds(message.header.stamp)
            self.last_odom_stamp = observed
            if not self.gate.ready(self.now()):
                self.pending_odometry.clear()
                return
            if not self.gate.accept_pose_stamp(observed, self.now()):
                return
            if message.header.frame_id != self.route_frame:
                raise ValueError('ODOM frame must equal route_frame; frame relabeling is not a transform')
            child = message.child_frame_id
            if not child or child == self.reference_frame:
                raise ValueError('ODOM must identify its body frame separately from the support reference')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
                raise ValueError('ODOM position must be finite')
            normalized_quaternion((q.x, q.y, q.z, q.w))
            if self.last_received_stamp is not None and observed <= self.last_received_stamp:
                self.gate.reason = 'duplicate or out-of-order pending ODOM source timestamp'
                return
            self.retry_pending()
            self.last_received_stamp = observed
            if len(self.pending_odometry) >= self.max_pending_odometry:
                # Preserve older still-fresh samples waiting for TF. Replacing
                # them with each latest ODOM can indefinitely chase the TF tail.
                self.gate.reason = 'pending ODOM queue is full; newest sample dropped'
                return
            self.pending_odometry.append(deepcopy(message))
            self.retry_pending()
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self.pending_odometry.clear()
            self.gate.reason = f'ODOM conversion blocked: {exc}'

    def retry_pending(self):
        if not self.gate.ready(self.now()):
            self.pending_odometry.clear()
            return
        while self.pending_odometry:
            message = self.pending_odometry[0]
            observed = seconds(message.header.stamp)
            # Recheck source age and publication ordering on every retry; a TF
            # becoming available later does not renew an expired ODOM sample.
            if not self.gate.accept_pose_stamp(observed, self.now()):
                self.pending_odometry.popleft()
                continue
            try:
                transform = self.buffer.lookup_transform(
                    message.child_frame_id, self.reference_frame,
                    Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.))
            except TransformException as exc:
                self.gate.reason = f'ODOM waiting for timestamped support TF: {exc}'
                return
            try:
                p, q = message.pose.pose.position, message.pose.pose.orientation
                t, r = transform.transform.translation, transform.transform.rotation
                xyz, xyzw = compose_reference_pose(
                    (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                    (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
                # Quality may have changed since reception. Never publish a
                # queued sample after its status or original timestamp expires.
                if not self.gate.accept_pose_stamp(observed, self.now()):
                    self.pending_odometry.popleft()
                    continue
                output = PoseStamped()
                output.header = deepcopy(message.header)
                output.pose.position.x, output.pose.position.y, output.pose.position.z = xyz
                (output.pose.orientation.x, output.pose.orientation.y,
                 output.pose.orientation.z, output.pose.orientation.w) = xyzw
                self.publisher.publish(output)
                self.gate.mark_published(observed)
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                self.gate.reason = f'ODOM conversion blocked: {exc}'
            self.pending_odometry.popleft()

    def diagnose(self):
        now = self.now()
        if not self.gate.ready(now):
            reason = self.gate.reason
        elif (self.last_odom_stamp is None or not math.isfinite(self.last_odom_stamp) or
              not -self.gate.future_tolerance <= now - self.last_odom_stamp <= self.gate.pose_max_age):
            reason = 'ODOM source timestamp is stale or unavailable'
        elif self.gate.last_pose_stamp is None:
            reason = self.gate.reason or 'no support reference pose has been published'
        elif not -self.gate.future_tolerance <= now - self.gate.last_pose_stamp <= self.gate.pose_max_age:
            reason = 'no recent converted support reference pose (check TF and ODOM)'
        else:
            reason = self.gate.reason
        self.diagnostics.publish(String(data=reason or 'OK'))
        if reason != self.last_diagnostic:
            if reason:
                self.get_logger().warning(reason)
            else:
                self.get_logger().info('M20 localization support pose ready')
            self.last_diagnostic = reason


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = M20Localization()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
