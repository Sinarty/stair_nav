"""Small ROS conversions; no algorithmic behavior is hidden here."""
import math
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from stair_nav.core.models import Pose


def durable_qos(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def live_qos(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp(seconds_value):
    value = max(0.0, seconds_value)
    sec = int(value)
    nano = int(round((value - sec) * 1e9))
    if nano >= 1000000000:
        sec += 1
        nano = 0
    return Time(sec=sec, nanosec=nano)


def pose_from_msg(msg):
    q = msg.pose.orientation
    # A non-unit or zero quaternion is invalid input, not an implicit yaw=0.
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.05:
        raise ValueError('localization quaternion must be finite and normalized')
    yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                     1.0 - 2.0 * (q.y*q.y + q.z*q.z))
    p = msg.pose.position
    return Pose(p.x, p.y, p.z, yaw, seconds(msg.header.stamp), msg.header.frame_id)


def pose_msg(pose, frame, now):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp(now)
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    msg.pose.position.z = float(pose.z)
    yaw = getattr(pose, 'yaw', 0.0)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def path_msg(path, frame, now):
    msg = Path()
    msg.header.frame_id = frame
    msg.header.stamp = stamp(now)
    for i, point in enumerate(path.points):
        p = pose_msg(point, frame, now)
        if i + 1 < len(path.points):
            nxt = path.points[i+1]
            yaw = math.atan2(nxt.y-point.y, nxt.x-point.x)
            p.pose.orientation.z = math.sin(yaw/2)
            p.pose.orientation.w = math.cos(yaw/2)
        msg.poses.append(p)
    return msg
