"""ROS-independent validation and full 3-D composition for M20 localization.

The vendor's ODOM is a body pose. A separate calibrated, possibly dynamic
body-to-navigation-reference transform supplies the support reference; this
module never guesses it from the reported body height.
"""
import math


def timestamp_seconds(header):
    """Read the two explicitly documented/vendor-published MetaType schemas.

    Public drdds sources use ``stamp.sec/nanosec``; the M20 guide spells the
    fields ``timestamp.sec/nsec``. The installed generated message determines
    which schema exists. No receipt timestamp is substituted.
    """
    if hasattr(header, 'stamp'):
        value, nanos_name = header.stamp, 'nanosec'
    elif hasattr(header, 'timestamp'):
        value, nanos_name = header.timestamp, 'nsec'
    else:
        raise ValueError('header requires stamp or timestamp')
    sec, nano = value.sec, getattr(value, nanos_name)
    if (isinstance(sec, bool) or not isinstance(sec, int) or
            isinstance(nano, bool) or not isinstance(nano, int) or
            not 0 <= nano < 1_000_000_000):
        raise ValueError('timestamp requires integer seconds and valid nanoseconds')
    return sec + nano * 1e-9


def normalized_quaternion(quaternion):
    """Validate near-unit XYZW input without discarding roll or pitch."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError('quaternion must contain four finite values')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if abs(norm - 1.0) > .05:
        raise ValueError('quaternion must be normalized within 0.05')
    return tuple(v / norm for v in quaternion)


def _multiply(a, b):
    x, y, z, w = a
    bx, by, bz, bw = b
    return (w*bx + x*bw + y*bz - z*by,
            w*by - x*bz + y*bw + z*bx,
            w*bz + x*by - y*bx + z*bw,
            w*bw - x*bx - y*by - z*bz)


def compose_reference_pose(position, orientation, translation, rotation):
    """Compose map→body and body→support as SE(3), returning XYZ and XYZW."""
    if (len(position) != 3 or len(translation) != 3 or
            not all(math.isfinite(v) for v in (*position, *translation))):
        raise ValueError('position and transform translation must be finite XYZ')
    q, r = normalized_quaternion(orientation), normalized_quaternion(rotation)
    rotated = _multiply(_multiply(q, (*translation, 0.)), (-q[0], -q[1], -q[2], q[3]))
    return (tuple(position[i] + rotated[i] for i in range(3)),
            normalized_quaternion(_multiply(q, r)))


class LocalizationGate:
    """Require one explicitly selected good status and fresh source times."""
    def __init__(self, normal_status=-1, pose_max_age=.5, status_max_age=1.5,
                 future_tolerance=.1):
        if (isinstance(normal_status, bool) or not isinstance(normal_status, int) or
                normal_status not in (-1, 0, 1)):
            raise ValueError('normal_location_status must be -1, 0, or 1')
        for name, value in (('pose_max_age', pose_max_age),
                            ('status_max_age', status_max_age),
                            ('future_tolerance', future_tolerance)):
            if (isinstance(value, bool) or not isinstance(value, (float, int)) or
                    not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
        self.normal_status = normal_status
        self.pose_max_age = pose_max_age
        self.status_max_age = status_max_age
        self.future_tolerance = future_tolerance
        self.status = None
        self.status_stamp = None
        self.last_pose_stamp = None
        self.reason = 'normal_location_status requires firmware-version confirmation'

    def update_status(self, status, stamp, now):
        if (isinstance(status, bool) or not isinstance(status, int) or
                not math.isfinite(stamp) or not math.isfinite(now) or
                stamp > now + self.future_tolerance):
            self.status = None
            self.reason = 'invalid localization status or source timestamp'
            return False
        if self.status_stamp is not None and stamp <= self.status_stamp:
            return False
        self.status, self.status_stamp = status, stamp
        return self.ready(now)

    def ready(self, now):
        if self.normal_status == -1:
            self.reason = 'normal_location_status requires firmware-version confirmation'
        elif self.status is None or self.status_stamp is None:
            self.reason = 'no valid localization status'
        elif not math.isfinite(now) or not -self.future_tolerance <= now - self.status_stamp <= self.status_max_age:
            self.reason = 'localization status source timestamp is stale or in the future'
        elif self.status != self.normal_status:
            self.reason = f'localization status {self.status} is not configured normal value {self.normal_status}'
        else:
            self.reason = ''
            return True
        return False

    def accept_pose_stamp(self, stamp, now):
        """Validate only. Call mark_published after TF conversion succeeds."""
        if not self.ready(now):
            return False
        if (not math.isfinite(stamp) or not math.isfinite(now) or
                not -self.future_tolerance <= now - stamp <= self.pose_max_age):
            self.reason = 'ODOM source timestamp is stale or in the future'
        elif self.last_pose_stamp is not None and stamp <= self.last_pose_stamp:
            self.reason = 'duplicate or out-of-order ODOM source timestamp'
        else:
            return True
        return False

    def mark_published(self, stamp):
        self.last_pose_stamp = stamp
        self.reason = ''
