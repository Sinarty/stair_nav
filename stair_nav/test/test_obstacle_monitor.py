"""Exercise actual ROS-boundary callbacks with messages/publishers, without DDS.

The lightweight boundary modules only supply ROS constructors/imports. Geometry,
clear debouncing, path adoption, and watchdog callbacks are the production code.
This also runs before newly introduced interfaces have been built by colcon.
"""
from dataclasses import replace
from collections import deque
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as S

import pytest

from stair_nav.core.obstacles import ObstacleConfig, ObstacleDetector


def stamp(value):
    whole = math.floor(value)
    return S(sec=whole, nanosec=round((value-whole)*1e9))


def seconds(value):
    return value.sec+value.nanosec*1e-9


class FakeObstacleStatus:
    def __init__(self):
        self.header = S(frame_id='', stamp=stamp(0.))
        self.valid_until = stamp(0.)
        self.path_stamp = stamp(0.)
        self.execution_id = self.reason = ''
        self.valid = self.blocked = False
        self.nearest_distance = 0.
        self.occupied_voxels = 0


class FakeTransformException(Exception):
    pass


def module(name, **attributes):
    result = ModuleType(name)
    result.__dict__.update(attributes)
    result.__path__ = []
    return result


@pytest.fixture
def monitor(monkeypatch):
    # Isolate import dependencies instead of globally importing a ROS runtime.
    cloud_reader = S(read_points=lambda message, **_: iter(message.points))
    replacements = {
        'rclpy': module('rclpy'),
        'rclpy.clock': module('rclpy.clock', Clock=object, ClockType=S(STEADY_TIME=1)),
        'rclpy.node': module('rclpy.node', Node=object),
        'rclpy.time': module('rclpy.time', Time=S(from_msg=lambda value: value)),
        'nav_msgs': module('nav_msgs'),
        'nav_msgs.msg': module('nav_msgs.msg', Path=object),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', PointCloud2=object),
        'sensor_msgs_py': module('sensor_msgs_py', point_cloud2=cloud_reader),
        'tf2_ros': module('tf2_ros', Buffer=object, TransformListener=object,
                          TransformException=FakeTransformException),
        'visualization_msgs': module('visualization_msgs'),
        'visualization_msgs.msg': module('visualization_msgs.msg', Marker=object, MarkerArray=object),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', Point=object),
        'stair_nav_interfaces': module('stair_nav_interfaces'),
        'stair_nav_interfaces.msg': module('stair_nav_interfaces.msg',
                                          ObstacleStatus=FakeObstacleStatus, SegmentStatus=object),
        'stair_nav.ros_utils': module('stair_nav.ros_utils', durable_qos=lambda *a: None,
                                     live_qos=lambda *a: None, pose_qos=lambda *a: None,
                                     stamp=stamp, seconds=seconds),
    }
    for name, replacement in replacements.items():
        monkeypatch.setitem(sys.modules, name, replacement)
    source = Path(__file__).resolve().parents[1]/'stair_nav/obstacle_monitor_node.py'
    spec = importlib.util.spec_from_file_location('_obstacle_monitor_callbacks', source)
    implementation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(implementation)
    node = object.__new__(implementation.ObstacleMonitor)
    clock = S(ros=10., steady=100.)
    # Avoid mutating the actual time module used by the surrounding test runner.
    implementation.time = S(monotonic=lambda: clock.steady)
    node.now = lambda: clock.ros
    node.config = replace(ObstacleConfig(), clear_seconds=.2)
    node.detector = ObstacleDetector(node.config)
    node.frame, node.reference_frame = 'map', 'ground'
    node.execution_id, node.execution_started, node.status_stamp = '', 0., -math.inf
    node.expected_path_stamp = 0.
    node.progress = 0.
    node.path = node.path_candidate = node.zone_path = node.latest = None
    node.pending = deque()
    node.received_at, node.max_points = -math.inf, 1000
    node.transform = lambda target, source, timestamp: lambda point: tuple(point)
    published = []
    node.status_pub = S(publish=published.append)
    node.publish_zone = lambda status: None
    return S(node=node, clock=clock, published=published)


def status(execution='run-a', issued=10.15, progress=0., path_stamp=10.1):
    return S(header=S(frame_id='map', stamp=stamp(issued)), execution_id=execution,
             progress=progress, path_stamp=stamp(path_stamp))


def path(issued=10.1, points=((-1., 0.), (1., 0.))):
    return S(header=S(frame_id='map', stamp=stamp(issued)), poses=[
        S(pose=S(position=S(x=x, y=y, z=0.))) for x, y in points])


def cloud(issued, points=((.8, 0., 0.),)):
    return S(header=S(frame_id='ground', stamp=stamp(issued)), width=len(points), height=1, points=points)


def test_path_arriving_before_initial_execution_status_is_retained(monitor):
    node = monitor.node
    candidate = path()
    node.on_path(candidate)
    assert node.path_candidate is candidate and node.path is None
    node.on_status(status())
    assert node.path is candidate and node.execution_id == 'run-a'


def test_execution_switch_does_not_reuse_previous_path_or_clearance(monitor):
    node = monitor.node
    node.on_status(status())
    old = path()
    node.on_path(old)
    node.latest = (10.2, 'run-a', True, False, 'CLEAR', math.inf, 0, 10.1)
    node.on_status(status('run-b', issued=11.2, path_stamp=11.1))
    assert node.path is None and node.latest is None
    assert not node.detector.result.valid and node.detector.result.blocked
    # A delayed copy of the old route cannot become the new execution's route.
    node.on_path(old)
    assert node.path is None


def test_next_execution_path_received_before_status_survives_switch(monitor):
    node = monitor.node
    node.on_status(status())
    node.on_path(path())
    new = path(11.1, ((0., 0.), (0., 1.)))
    node.on_path(new)
    node.on_status(status('run-b', issued=11.2, path_stamp=11.1))
    assert node.path is new
    node.on_path(path())
    assert node.path is new


def test_path_update_discards_old_clear_lease_and_restarts_clear_countdown(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status())
    node.on_path(path())
    for issued in (10.2, 10.4):
        clock.ros = issued
        node.on_cloud(cloud(issued))
    assert node.latest[2:4] == (True, False)
    node.on_path(path(10.5, ((0., 0.), (0., 1.))))
    node.on_status(status(issued=10.55, path_stamp=10.5))
    assert node.latest is None and node.detector.result.blocked
    clock.ros = 10.6
    node.on_cloud(cloud(10.6))
    assert node.latest[2:5] == (True, True, 'CLEARING')


def test_cloud_uses_its_timestamp_for_tf_and_carries_inspected_path_stamp(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status())
    candidate = path()
    node.on_path(candidate)
    transforms = []

    def transform(target, source, timestamp):
        transforms.append((target, source, seconds(timestamp)))
        return lambda point: tuple(point)

    node.transform = transform
    clock.ros = 10.3
    node.on_cloud(cloud(10.2))
    assert transforms == [('ground', 'ground', 10.2), ('ground', 'map', 10.2)]
    assert node.latest[-1] == pytest.approx(10.1)
    node.publish()
    message = monitor.published[-1]
    assert seconds(message.header.stamp) == pytest.approx(10.2)
    assert seconds(message.path_stamp) == pytest.approx(10.1)


@pytest.mark.parametrize('expired_clock', ['ros', 'steady'])
def test_republishing_keeps_source_lease_and_either_clock_expiring_blocks(monitor, expired_clock):
    node, clock = monitor.node, monitor.clock
    node.on_status(status())
    node.latest = (10.2, 'run-a', True, False, 'CLEAR', .8, 3, 10.1)
    node.received_at = 100.
    for now, steady in ((10.3, 100.1), (10.4, 100.2)):
        clock.ros, clock.steady = now, steady
        node.publish()
        msg = monitor.published[-1]
        assert msg.valid and not msg.blocked
        assert seconds(msg.header.stamp) == pytest.approx(10.2)
        assert seconds(msg.valid_until) == pytest.approx(10.7)
        assert seconds(msg.path_stamp) == pytest.approx(10.1)
    if expired_clock == 'ros':
        clock.ros = 10.8
    else:
        clock.steady = 100.8
    node.publish()
    msg = monitor.published[-1]
    assert not msg.valid and msg.blocked and msg.reason == 'pointcloud_stale'
    assert seconds(msg.header.stamp) == pytest.approx(10.2)
    assert seconds(msg.valid_until) == pytest.approx(10.7)
    assert node.detector.result.blocked and not node.detector.result.valid


def test_waiting_for_cloud_never_publishes_a_valid_clearance(monitor):
    node = monitor.node
    node.on_status(status())
    node.publish()
    msg = monitor.published[-1]
    assert not msg.valid and msg.blocked and msg.reason == 'waiting_for_pointcloud'
    assert seconds(msg.valid_until) == seconds(msg.header.stamp)


def test_tf_arriving_late_retries_original_sample_without_extending_receipt_lease(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status(path_stamp=0.))

    def unavailable(*_):
        raise FakeTransformException('transform is slightly behind cloud')

    node.transform = unavailable
    clock.ros, clock.steady = 10.2, 100.2
    message = cloud(10.2)
    node.on_cloud(message)
    assert len(node.pending) == 1 and node.latest is None
    clock.ros, clock.steady = 10.3, 100.3
    node.transform = lambda *_: lambda point: tuple(point)
    node.publish()
    assert not node.pending and node.latest[0] == pytest.approx(10.2)
    assert node.received_at == pytest.approx(100.2)
    assert node.latest[2:5] == (True, True, 'CLEARING')
    assert seconds(monitor.published[-1].header.stamp) == pytest.approx(10.2)
    assert seconds(monitor.published[-1].valid_until) == pytest.approx(10.7)


@pytest.mark.parametrize('expired_clock', ['ros', 'steady'])
def test_pending_tf_timeout_cannot_clear_or_extend_source_time(monitor, expired_clock):
    node, clock = monitor.node, monitor.clock
    node.on_status(status(path_stamp=0.))

    def unavailable(*_):
        raise FakeTransformException('missing transform')

    node.transform = unavailable
    clock.ros, clock.steady = 10.2, 100.2
    node.on_cloud(cloud(10.2))
    assert len(node.pending) == 1
    if expired_clock == 'ros':
        clock.ros = 10.8
    else:
        clock.steady = 100.8
    node.transform = lambda *_: lambda point: tuple(point)
    node.publish()
    msg = monitor.published[-1]
    assert not node.pending and not msg.valid and msg.blocked
    assert seconds(msg.header.stamp) == pytest.approx(10.2)
    assert seconds(msg.valid_until) == pytest.approx(10.7)
    assert not node.detector.result.valid and node.detector.result.blocked


def test_execution_change_discards_clouds_still_waiting_for_tf(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status(path_stamp=0.))

    def unavailable(*_):
        raise FakeTransformException('waiting')

    node.transform = unavailable
    clock.ros = 10.2
    node.on_cloud(cloud(10.2))
    assert node.pending
    node.on_status(status('run-b', issued=10.3, path_stamp=0.))
    assert not node.pending and node.latest is None


def test_malformed_tf_is_invalid_immediately_instead_of_crashing_or_waiting(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status(path_stamp=0.))

    def malformed(*_):
        raise ValueError('nonfinite transform rotation')

    node.transform = malformed
    clock.ros = 10.2
    node.on_cloud(cloud(10.2))
    assert not node.pending
    assert node.latest[2:4] == (False, True)
    assert 'nonfinite transform rotation' in node.latest[4]


def test_obstacle_message_schema_includes_inspected_path_generation_stamp():
    schema = Path(__file__).resolve().parents[2]/'stair_nav_interfaces/msg/ObstacleStatus.msg'
    assert 'builtin_interfaces/Time path_stamp' in schema.read_text()


@pytest.mark.parametrize('path_first', [True, False])
def test_late_monitor_matches_old_retained_path_to_current_periodic_status(monitor, path_first):
    node, clock = monitor.node, monitor.clock
    clock.ros = 25.
    retained = path(10.1)
    current = status(issued=25., progress=.6, path_stamp=10.1)
    if path_first:
        node.on_path(retained)
        node.on_status(current)
    else:
        node.on_status(current)
        node.on_path(retained)
    assert node.path is retained
    node.on_cloud(cloud(25.))
    assert node.latest[2] and node.latest[-1] == pytest.approx(10.1)


def test_monitor_restart_rebuilds_clearance_against_the_same_running_path(monitor):
    node, clock = monitor.node, monitor.clock
    clock.ros = 25.
    node.on_status(status(issued=25., progress=.6, path_stamp=10.1))
    assert node.path is None and node.detector.result.blocked
    node.on_cloud(cloud(25.))
    assert node.latest[2:5] == (False, True, 'waiting_for_current_reference_path')
    node.on_path(path(10.1))
    assert node.latest is None
    node.on_cloud(cloud(25.))
    assert node.latest[2:5] == (True, True, 'CLEARING')
    clock.ros = 25.2
    node.on_cloud(cloud(25.2))
    assert node.latest[2:5] == (True, False, 'CLEAR')


def test_new_execution_rejects_old_path_until_exact_generation_arrives(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status())
    old = path(10.1)
    node.on_path(old)
    node.on_status(status('run-b', issued=11.2, path_stamp=11.1))
    node.on_path(old)
    clock.ros = 11.3
    node.on_cloud(cloud(11.3))
    assert node.path is None
    assert node.latest[2:5] == (False, True, 'waiting_for_current_reference_path')
    # Even a nearby timestamp is a different route generation.
    node.on_path(path(11.099))
    assert node.path is None
    node.on_path(path(11.1))
    assert node.matches_path(node.path)
    assert node.latest is None and node.detector.result.blocked


def test_checking_can_use_forward_corridor_until_a_path_generation_is_declared(monitor):
    node, clock = monitor.node, monitor.clock
    node.on_status(status(path_stamp=0.))
    for issued in (10.2, 10.4):
        clock.ros = issued
        node.on_cloud(cloud(issued))
    assert node.latest[2:5] == (True, False, 'CLEAR') and node.latest[-1] == 0.
    node.on_status(status(issued=10.5, path_stamp=10.45))
    assert node.latest is None and node.path is None
    clock.ros = 10.6
    node.on_cloud(cloud(10.6))
    assert node.latest[2:5] == (False, True, 'waiting_for_current_reference_path')


def test_segment_status_schema_carries_generation_independent_of_periodic_header():
    schema = Path(__file__).resolve().parents[2]/'stair_nav_interfaces/msg/SegmentStatus.msg'
    assert 'builtin_interfaces/Time path_stamp' in schema.read_text()
