"""Leased point-cloud obstacle decisions; never publishes robot velocity commands."""
from dataclasses import fields
from collections import deque
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from stair_nav_interfaces.msg import ObstacleStatus, SegmentStatus

from stair_nav.core.cloud_geometry import rigid_transform, remaining_path
from stair_nav.core.config import load_route
from stair_nav.core.obstacles import ObstacleConfig, ObstacleDetector
from stair_nav.ros_utils import durable_qos, live_qos, pose_qos, seconds, stamp


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')
        self.declare_parameter('route_file', '')
        self.declare_parameter('reference_frame', 'navigation_ground_reference')
        self.declare_parameter('cloud_topic', '/points_raw')
        self.declare_parameter('cloud_qos_reliability', 'best_effort')
        self.declare_parameter('publish_rate', 20.)
        self.declare_parameter('max_cloud_points', 250000)
        defaults = ObstacleConfig()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self.config = ObstacleConfig(**{field.name: self.get_parameter(field.name).value
                                       for field in fields(defaults)})
        self.detector = ObstacleDetector(self.config)
        self.route = load_route(self.get_parameter('route_file').value)
        self.frame = self.route['frame']
        self.reference_frame = self.get_parameter('reference_frame').value
        self.max_points = self.get_parameter('max_cloud_points').value
        rate = self.get_parameter('publish_rate').value
        if not self.reference_frame or not isinstance(self.max_points, int) or self.max_points <= 0:
            raise ValueError('reference_frame and positive max_cloud_points are required')
        if not math.isfinite(rate) or not 1 <= rate <= 100:
            raise ValueError('publish_rate must be between 1 and 100 Hz')
        self.execution_id = ''
        self.execution_started = 0.
        self.expected_path_stamp = 0.
        self.status_stamp = -math.inf
        self.progress = 0.
        self.path = None
        self.path_candidate = None
        self.zone_path = None
        self.pending = deque()
        self.latest = None
        self.received_at = -math.inf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(ObstacleStatus, '/navigation/obstacle_status', live_qos(5))
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation/obstacle_zone', durable_qos())
        self.create_subscription(SegmentStatus, '/navigation/status', self.on_status, durable_qos())
        self.create_subscription(Path, '/navigation/reference_path', self.on_path, durable_qos())
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value, self.on_cloud,
                                 pose_qos(self.get_parameter('cloud_qos_reliability').value, 1))
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./rate, self.publish, clock=self.watchdog_clock)
        self.get_logger().info('Flat obstacle monitor ready; invalid or expired sensing cannot clear a pause')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_status(self, msg):
        issued = seconds(msg.header.stamp)
        if msg.header.frame_id != self.frame or issued < self.status_stamp:
            return
        self.status_stamp = issued
        self.progress = float(msg.progress)
        expected = seconds(msg.path_stamp)
        changed = expected != self.expected_path_stamp
        self.expected_path_stamp = expected
        if msg.execution_id != self.execution_id:
            self.execution_id = msg.execution_id
            self.execution_started = seconds(msg.header.stamp)
            self.path = None
            self.latest = None
            self.pending.clear()
            self.detector = ObstacleDetector(self.config)
            changed = True
        if changed:
            self.path = (self.path_candidate if self.matches_path(self.path_candidate) else None)
            self.detector = ObstacleDetector(self.config)
            self.latest = None

    def matches_path(self, path):
        return (path is not None and self.expected_path_stamp > 0 and
                abs(seconds(path.header.stamp)-self.expected_path_stamp) <= 1e-6)

    def on_path(self, msg):
        if msg.header.frame_id != self.frame:
            return
        if (self.path_candidate is not None and
                seconds(msg.header.stamp) < seconds(self.path_candidate.header.stamp)):
            return
        self.path_candidate = msg if msg.poses else None
        if self.execution_id and self.matches_path(self.path_candidate):
            self.path = self.path_candidate
            self.detector = ObstacleDetector(self.config)
            self.latest = None

    def transform(self, target, source, timestamp):
        if target == source:
            return lambda p: tuple(map(float, p))
        value = self.tf_buffer.lookup_transform(target, source, Time.from_msg(timestamp)).transform
        p, q = value.translation, value.rotation
        return rigid_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def invalidate(self, reason, issued=None):
        issued = self.now() if issued is None else issued
        self.detector = ObstacleDetector(self.config)
        self.latest = (issued, self.execution_id, False, True, reason, math.inf, 0, 0.)
        self.received_at = time.monotonic()

    def on_cloud(self, msg):
        now, issued = self.now(), seconds(msg.header.stamp)
        if not self.execution_id:
            return
        if issued <= 0 or not 0 <= now-issued <= self.config.cloud_timeout:
            self.pending.clear()
            self.invalidate('pointcloud_timestamp_invalid')
            return
        if not msg.header.frame_id or not 0 < msg.width*msg.height <= self.max_points:
            self.pending.clear()
            self.invalidate('pointcloud_empty_or_too_large', issued)
            return
        if len(self.pending) >= 8:
            self.pending.popleft()
            self.invalidate('pointcloud_transform_queue_full')
        self.pending.append((msg, time.monotonic()))
        self.drain_pending()

    def drain_pending(self):
        while self.pending:
            msg, received_at = self.pending[0]
            issued = seconds(msg.header.stamp)
            if (not 0 <= self.now()-issued <= self.config.cloud_timeout or
                    time.monotonic()-received_at > self.config.cloud_timeout):
                self.pending.popleft()
                self.invalidate('pointcloud_transform_timeout', issued)
                continue
            if not self.process_cloud(msg, received_at):
                break
            self.pending.popleft()

    def process_cloud(self, msg, received_at):
        issued = seconds(msg.header.stamp)
        if self.expected_path_stamp > 0 and not self.matches_path(self.path):
            self.invalidate('waiting_for_current_reference_path', issued)
            return True
        try:
            apply = self.transform(self.reference_frame, msg.header.frame_id, msg.header.stamp)
            path_transform = (self.transform(self.reference_frame, self.frame, msg.header.stamp)
                              if self.path is not None else None)
        except TransformException:
            # Moving-frame TF usually arrives after the cloud. Retry this source
            # sample in order, bounded by its original source and receipt leases.
            return False
        except Exception as exc:
            self.invalidate('pointcloud_transform_invalid: '+str(exc)[:160], issued)
            return True
        try:
            points = [apply(p) for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)]
            path_xy = None
            if self.path is not None:
                path_xy = remaining_path([path_transform((p.pose.position.x, p.pose.position.y,
                                                         p.pose.position.z))[:2]
                                          for p in self.path.poses], progress=self.progress)
            self.zone_path = path_xy
            result = self.detector.observe(points, issued, path_xy=path_xy)
            self.latest = (issued, self.execution_id, result.valid, result.blocked,
                           result.reason, result.nearest_distance, result.points,
                           seconds(self.path.header.stamp) if self.path is not None else 0.)
            self.received_at = received_at
        except Exception as exc:
            # A TF, field-layout or transform error is visible as invalid sensing,
            # never as an empty free corridor. The next fresh cloud retries it.
            self.invalidate('pointcloud_transform_or_decode: '+str(exc)[:160], issued)
        return True

    def publish(self):
        self.drain_pending()
        now = self.now()
        msg = ObstacleStatus()
        msg.header.frame_id = self.frame
        msg.execution_id = self.execution_id
        msg.nearest_distance = math.inf
        if self.latest is None:
            msg.header.stamp = stamp(now)
            msg.valid_until = stamp(now)
            msg.valid, msg.blocked, msg.reason = False, True, 'waiting_for_pointcloud'
        else:
            issued, execution, valid, blocked, reason, distance, count, path_stamp = self.latest
            fresh = (execution == self.execution_id and 0 <= now-issued <= self.config.cloud_timeout and
                     time.monotonic()-self.received_at <= self.config.cloud_timeout)
            msg.header.stamp = stamp(issued)
            msg.valid_until = stamp(issued+self.config.cloud_timeout)
            msg.valid, msg.blocked = bool(valid and fresh), bool(blocked or not fresh)
            msg.reason = reason if fresh else 'pointcloud_stale'
            msg.nearest_distance, msg.occupied_voxels = distance, count
            msg.path_stamp = stamp(path_stamp)
            if not fresh:
                self.detector = ObstacleDetector(self.config)
        self.status_pub.publish(msg)
        self.publish_zone(msg)

    def publish_zone(self, status):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        # Debug geometry follows the moving reference using its latest TF.
        # Its source-time lease is carried separately in ObstacleStatus.
        marker.header.stamp = stamp(0.)
        marker.frame_locked = True
        marker.ns, marker.id = 'obstacle_stop_zone', 0
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = .035
        marker.color.r = 1. if status.blocked else .1
        marker.color.g = .2 if status.valid and status.blocked else (.65 if not status.valid else 1.)
        marker.color.b, marker.color.a = .1, .95
        length, width = self.config.forward_distance, self.config.half_width
        path = self.zone_path or [(0., 0.), (length, 0.)]
        retained, used = [(0., 0.)], 0.
        for end in path:
            start = retained[-1]
            span = math.dist(start, end)
            if span < 1e-8:
                continue
            portion = min(span, length-used)
            end = (start[0]+(end[0]-start[0])*portion/span,
                   start[1]+(end[1]-start[1])*portion/span)
            retained.append(end)
            used += portion
            if used >= length:
                break
        if len(retained) == 1:
            retained.append((length, 0.))
        for start, end in zip(retained, retained[1:]):
            span = math.dist(start, end)
            nx, ny = -(end[1]-start[1])*width/span, (end[0]-start[0])*width/span
            for side in (-1., 1.):
                marker.points.extend(Point(x=p[0]+side*nx, y=p[1]+side*ny, z=.08) for p in (start, end))
        for center in retained:
            for index in range(24):
                for angle in (index*math.pi/12, (index+1)*math.pi/12):
                    marker.points.append(Point(x=center[0]+width*math.cos(angle),
                                               y=center[1]+width*math.sin(angle), z=.08))
        label = Marker()
        label.header = marker.header
        label.frame_locked = True
        label.ns, label.id = marker.ns, 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.orientation.w = 1.
        label.pose.position.z = 1.7
        label.scale.z = .2
        label.color = marker.color
        label.text = ('SENSOR INVALID' if not status.valid else 'OBSTACLE / WAIT' if status.blocked else 'CLEAR')
        self.marker_pub.publish(MarkerArray(markers=[marker, label]))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
