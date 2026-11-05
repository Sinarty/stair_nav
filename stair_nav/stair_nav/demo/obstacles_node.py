"""Synthetic live cloud and pedestrian for the flat-ground obstacle-stop demo."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from stair_nav_interfaces.msg import SegmentStatus

from stair_nav.core.config import load_route
from stair_nav.ros_utils import durable_qos, live_qos, pose_from_msg, stamp


class DemoObstacles(Node):
    def __init__(self):
        super().__init__('demo_obstacles')
        self.declare_parameter('route_file', '')
        self.declare_parameter('auto_obstacle', True)
        self.declare_parameter('obstacle_duration', 5.)
        self.declare_parameter('trigger_progress', .22)
        self.route = load_route(self.get_parameter('route_file').value)
        self.auto = self.get_parameter('auto_obstacle').value
        self.duration = self.get_parameter('obstacle_duration').value
        self.trigger_progress = self.get_parameter('trigger_progress').value
        if not math.isfinite(self.duration) or self.duration <= 0 or not 0 < self.trigger_progress < 1:
            raise ValueError('obstacle_duration > 0 and 0 < trigger_progress < 1 are required')
        self.pose = None
        self.navigation = None
        self.active = False
        self.cloud_enabled = True
        self.person = None
        self.remove_at = None
        self.triggered = set()
        self.cloud_pub = self.create_publisher(PointCloud2, '/simulation/points', live_qos())
        self.marker_pub = self.create_publisher(MarkerArray, '/simulation/obstacles', durable_qos())
        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose, live_qos(5))
        self.create_subscription(SegmentStatus, '/navigation/status', self.on_status, durable_qos())
        self.create_service(SetBool, '~/set_obstacle', self.set_obstacle)
        self.create_service(SetBool, '~/set_cloud_enabled', self.set_cloud_enabled)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Synthetic pedestrian/cloud ready; use ~/set_obstacle and ~/set_cloud_enabled')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_pose(self, msg):
        try:
            self.pose = pose_from_msg(msg)
        except ValueError:
            self.pose = None

    def on_status(self, msg):
        self.navigation = msg

    def place_person(self):
        if self.pose is None:
            return False
        p = self.pose
        self.person = (p.x+.9*math.cos(p.yaw), p.y+.9*math.sin(p.yaw), p.z)
        self.active = True
        self.get_logger().info('Pedestrian entered the route: obstacle cloud ON')
        return True

    def set_obstacle(self, request, response):
        self.remove_at = None
        if request.data:
            response.success = self.place_person()
            response.message = 'Pedestrian placed ahead' if response.success else 'Waiting for localization'
        else:
            self.active = False
            response.success, response.message = True, 'Pedestrian removed; waiting for continuous clear observations'
            self.get_logger().info(response.message)
        return response

    def set_cloud_enabled(self, request, response):
        self.cloud_enabled = request.data
        response.success = True
        response.message = 'Point cloud stream {}'.format('ON' if request.data else 'OFF')
        self.get_logger().info(response.message)
        return response

    def tick(self):
        if self.pose is None:
            return
        now, nav = self.now(), self.navigation
        if (self.auto and nav is not None and nav.state == 'EXECUTING' and
                nav.progress >= self.trigger_progress and nav.execution_id not in self.triggered):
            self.triggered.add(nav.execution_id)
            if self.place_person():
                self.remove_at = now+self.duration
        if self.remove_at is not None and now >= self.remove_at:
            self.active, self.remove_at = False, None
            self.get_logger().info('Pedestrian left the route; obstacle cloud OFF')
        self.publish_person(now)
        if not self.cloud_enabled:
            return
        p = self.pose
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        points = []
        # Finite ground returns remain when the pedestrian leaves. Clearance
        # is inferred from a live observation, never an artificial empty cloud.
        for ix in range(-3, 16):
            for iy in range(-6, 7):
                x, y = ix*.12, iy*.12
                points.append((p.x+c*x-s*y, p.y+s*x+c*y, p.z))
        # Self returns exercise the body mask; one isolated return exercises
        # the occupied-voxel threshold without holding up a clear corridor.
        points.extend(((p.x, p.y, p.z+.4),
                       (p.x+c*.65-s*.1, p.y+s*.65+c*.1, p.z+.6)))
        if self.active and self.person is not None:
            x, y, z = self.person
            for height in range(2, 17):
                for angle in range(16):
                    a = 2*math.pi*angle/16
                    points.append((x+.18*math.cos(a), y+.18*math.sin(a), z+height*.1))
        header = Header(frame_id=self.route['frame'], stamp=stamp(now))
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def publish_person(self, now):
        markers = []
        for ident, head in ((0, False), (1, True)):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.route['frame'], stamp(now)
            marker.ns, marker.id = 'synthetic_pedestrian', ident
            marker.type = Marker.SPHERE if head else Marker.CYLINDER
            marker.action = Marker.ADD if self.active else Marker.DELETE
            marker.pose.orientation.w = 1.
            if self.person is not None:
                x, y, z = self.person
                marker.pose.position.x, marker.pose.position.y = x, y
                marker.pose.position.z = z+(1.55 if head else .75)
            marker.scale.x = marker.scale.y = .32 if head else .36
            marker.scale.z = .32 if head else 1.25
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .25, .18, .9
            markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = DemoObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
