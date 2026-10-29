"""ROS Action wrapper around the non-ROS segment execution state machine."""
import math
import uuid
from copy import deepcopy
from threading import RLock

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from visualization_msgs.msg import Marker, MarkerArray
from stair_nav_interfaces.action import NavigateSegment
from stair_nav_interfaces.srv import GetNavigationState
from stair_nav_interfaces.msg import ControlCommand, MotionStatus, SegmentStatus, ChannelEvent
from stair_nav.core.config import load_route
from stair_nav.core.engine import SegmentExecutor
from stair_nav.core.models import Pose, MotionFeedback
from stair_nav.ros_utils import (
    durable_qos, live_qos, seconds, stamp, pose_from_msg, pose_msg, path_msg,
)


class SegmentNavigation(Node):
    def __init__(self):
        super().__init__('segment_navigation')
        self.declare_parameter('route_file', '')
        route_file = self.get_parameter('route_file').value
        if not route_file:
            raise ValueError('route_file is required; provide a validated route YAML')
        self.route = load_route(route_file)
        self.core = SegmentExecutor(self.route)
        self.lock = RLock()
        self.reserved = False
        self.reserved_execution_id = ''
        self.pending_cancel = False
        self.server_instance = str(uuid.uuid4())
        self.latest_motion = None
        self.last_status = None
        self.seen_ids = set()
        self.goal_handle = None
        self.completion = None
        self.last_path_key = None
        self.last_log_state = None
        self.group = ReentrantCallbackGroup()
        self.commands = self.create_publisher(ControlCommand, '/navigation/command', live_qos())
        self.statuses = self.create_publisher(SegmentStatus, '/navigation/status', durable_qos())
        self.paths = self.create_publisher(PathMsg, '/navigation/reference_path', durable_qos())
        self.targets = self.create_publisher(PoseStamped, '/navigation/reference_point', durable_qos())
        self.markers = self.create_publisher(MarkerArray, '/navigation/scene', durable_qos())
        self.create_subscription(PoseStamped, '/localization/pose', self.on_pose,
                                 live_qos(5), callback_group=self.group)
        self.create_subscription(MotionStatus, '/motion/status', self.on_motion,
                                 live_qos(5), callback_group=self.group)
        self.create_subscription(ChannelEvent, '/navigation/channel_event', self.on_channel,
                                 live_qos(5), callback_group=self.group)
        self.server = ActionServer(
            self, NavigateSegment, '/navigate_segment', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.group, result_timeout=60,
        )
        self.create_service(GetNavigationState, '/navigation/get_state', self.get_state,
                            callback_group=self.group)
        self.period = float(self.route.get('control_period', 0.05))
        self.last_tick = self.now()
        self.create_timer(self.period, self.tick, callback_group=self.group)
        self.publish_scene()
        self.get_logger().info('segment Action ready; candidate controls require an explicit adapter')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        with self.lock:
            try:
                pose = pose_from_msg(msg)
            except ValueError:
                pose = Pose(math.nan, math.nan, math.nan, math.nan,
                            seconds(msg.header.stamp), msg.header.frame_id)
            self.core.update_pose(pose)

    def on_motion(self, msg):
        if msg.header.frame_id != self.route['frame']:
            return
        feedback = MotionFeedback(
            stamp=seconds(msg.header.stamp), execution_id=msg.execution_id,
            segment_id=msg.segment_id, mode=msg.mode, ready=msg.ready,
            exit_confirmed=msg.exit_confirmed, stable=msg.stable,
            hold_confirmed=msg.hold_confirmed, fault_code=msg.fault_code,
            fault_message=msg.fault_message, capabilities=tuple(msg.capabilities),
            supported_modes=tuple(msg.supported_modes))
        with self.lock:
            previous = getattr(self, 'latest_motion', None)
            if (math.isfinite(feedback.stamp) and feedback.stamp <= self.now() + .1 and
                    (previous is None or feedback.stamp >= previous.stamp)):
                self.latest_motion = feedback
            self.core.update_motion(feedback)

    def get_state(self, request, response):
        """Snapshot reservations and release evidence under the execution lock."""
        with self.lock:
            now = self.now()
            response.header.stamp = stamp(now)
            response.header.frame_id = self.route['frame']
            response.server_instance = self.server_instance
            response.busy = bool(self.reserved or self.core.active)
            status = self.core.status
            m = self.latest_motion
            max_age = float(self.route.get('defaults', {}).get('timeouts', {}).get('status', .5))
            fresh = m is not None and -0.1 <= now - m.stamp <= max_age
            initial = status.state == 'IDLE' and not status.execution_id
            idle_release = bool(initial and fresh and not m.execution_id and m.stable
                                and not m.fault_code)
            terminal_release = bool(not initial and fresh and
                                    m.execution_id == status.execution_id and
                                    m.segment_id == status.segment_id and
                                    status.state in ('SUCCEEDED', 'FAILED', 'CANCELED') and
                                    status.navigation_stopped and m.hold_confirmed and
                                    m.stable and not m.fault_code)
            response.control_released = bool(not response.busy and (idle_release or terminal_release))
            response.status = self.status_message(status, now)
        return response

    def on_channel(self, msg):
        now = self.now()
        age = now-seconds(msg.header.stamp)
        if (msg.header.frame_id != self.route['frame'] or
                not math.isfinite(age) or age < -0.1 or age > 1.0):
            self.get_logger().warning('ignored channel event with stale time or wrong frame')
            return
        with self.lock:
            self.core.set_blocked(msg.segment_id, msg.blocked, msg.reason,
                                  seconds(msg.header.stamp))

    def on_goal(self, request):
        with self.lock:
            if (self.reserved or self.core.active or not request.task_id or
                    not request.execution_id or request.execution_id in self.seen_ids or
                    request.segment_id not in self.route['segments']):
                self.get_logger().warning('rejected busy, invalid, or reused segment request')
                return GoalResponse.REJECT
            self.reserved = True
            self.reserved_execution_id = request.execution_id
            self.pending_cancel = False
            self.seen_ids.add(request.execution_id)
            return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        with self.lock:
            # The Action goal can be ACCEPTED before execute() is scheduled.
            # Reserve by execution ID so this interval still accepts cancellation.
            if (not self.reserved or
                    goal_handle.request.execution_id != self.reserved_execution_id):
                return CancelResponse.REJECT
            if (self.goal_handle is goal_handle and not self.core.active and
                    self.core.status.state in ('SUCCEEDED', 'FAILED')):
                # Completion won this race; don't acknowledge a cancellation
                # that can no longer change the already selected result.
                return CancelResponse.REJECT
            self.pending_cancel = True
            if self.goal_handle is goal_handle and self.core.active:
                self.core.cancel(self.now())
                # Revoke immediately; the result waits until ROS enters CANCELING.
                self.publish_output(self.core.tick(self.now(), self.period))
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        try:
            with self.lock:
                request = goal_handle.request
                self.goal_handle = goal_handle
                self.completion = Future()
                started = self.core.start(request.task_id, request.execution_id,
                                          request.segment_id, self.now())
                if not started:
                    goal_handle.abort()
                    return NavigateSegment.Result(
                        success=False, error_code='BUSY', error_message='executor refused start',
                        navigation_stopped=True, hold_confirmed=False)
                initial = self.status_message(self.core.status, self.now())
                self.statuses.publish(initial)
                goal_handle.publish_feedback(NavigateSegment.Feedback(status=initial))
                if self.pending_cancel or goal_handle.is_cancel_requested:
                    self.core.cancel(self.now())
                    self.publish_output(self.core.tick(self.now(), self.period))
                completion = self.completion
            terminal = await completion
            with self.lock:
                result = NavigateSegment.Result(
                    success=terminal.state == 'SUCCEEDED', error_code=terminal.error_code,
                    error_message=terminal.error_message,
                    navigation_stopped=terminal.navigation_stopped,
                    hold_confirmed=terminal.hold_confirmed)
                if terminal.state == 'CANCELED':
                    goal_handle.canceled()
                elif terminal.state == 'SUCCEEDED':
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
        except Exception as exc:
            # No accepted request can strand the reservation after a wrapper error.
            with self.lock:
                self.core.cancel(self.now())
                self.publish_output(self.core.tick(self.now(), self.period))
                if goal_handle.is_active:
                    goal_handle.abort()
                self.get_logger().error(f'action wrapper error: {type(exc).__name__}: {exc}')
                return NavigateSegment.Result(
                    success=False, error_code='ACTION_INTERNAL_ERROR', error_message=str(exc),
                    navigation_stopped=True, hold_confirmed=False)
        finally:
            with self.lock:
                self.goal_handle = None
                self.completion = None
                self.reserved = False
                self.reserved_execution_id = ''
                self.pending_cancel = False

    def tick(self):
        with self.lock:
            now = self.now()
            dt = max(0.001, min(0.2, now-self.last_tick))
            self.last_tick = now
            output = self.core.tick(now, dt)
            msg = self.publish_output(output)
            if self.goal_handle is not None and self.completion is not None:
                if not self.completion.done():
                    self.goal_handle.publish_feedback(NavigateSegment.Feedback(status=msg))
                    terminal = output.status.state in ('SUCCEEDED', 'FAILED', 'CANCELED')
                    cancel_ready = (output.status.state != 'CANCELED' or
                                    self.goal_handle.is_cancel_requested)
                    if terminal and cancel_ready:
                        self.completion.set_result(output.status)

    def status_message(self, status, now):
        msg = SegmentStatus()
        for field in ('task_id', 'execution_id', 'segment_id', 'phase', 'state',
                      'progress', 'lateral_error', 'heading_error', 'error_code',
                      'error_message', 'geometry_reached', 'navigation_stopped', 'hold_confirmed'):
            setattr(msg, field, getattr(status, field))
        msg.header.stamp = stamp(now)
        msg.header.frame_id = self.route['frame']
        return msg

    def publish_output(self, output):
        now = self.now()
        status = output.status
        msg = self.status_message(status, now)
        self.last_status = msg
        self.statuses.publish(msg)
        command = output.command
        if command.execution_id:
            cmd = ControlCommand()
            for field in ('task_id', 'execution_id', 'segment_id', 'phase', 'mode',
                          'behavior', 'vx', 'wz', 'valid'):
                setattr(cmd, field, getattr(command, field))
            cmd.header.stamp = stamp(command.stamp)
            cmd.header.frame_id = self.route['frame']
            cmd.valid_until = stamp(command.valid_until)
            self.commands.publish(cmd)
        key = (status.execution_id, id(output.path))
        if key != self.last_path_key:
            self.last_path_key = key
            if output.path is not None:
                self.paths.publish(path_msg(output.path, self.route['frame'], now))
            else:
                empty = PathMsg()
                empty.header = msg.header
                self.paths.publish(empty)
        if output.target is not None:
            self.targets.publish(pose_msg(output.target, self.route['frame'], now))
        log_key = (status.execution_id, status.state, status.error_code)
        if log_key != self.last_log_state:
            self.get_logger().info(
                f'segment={status.segment_id} execution={status.execution_id} '
                f'state={status.state} phase={status.phase} '
                f'error={status.error_code} {status.error_message}')
            self.last_log_state = log_key
        return msg

    def publish_scene(self):
        array = MarkerArray()
        frame = self.route['frame']
        for node_id, node in self.route['nodes'].items():
            p = node['position']
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'task_nodes'
            marker.id = len(array.markers)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, p)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
            array.markers.append(marker)
            label = Marker()
            label.header = marker.header
            label.ns = 'node_labels'
            label.id = marker.id
            label.type = Marker.TEXT_VIEW_FACING
            label.pose = deepcopy(marker.pose)
            label.pose.position.z += 0.35
            label.scale.z = 0.28
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = node_id
            array.markers.append(label)
        for region in self.route.get('regions', []):
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'regions'
            marker.id = len(array.markers)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, region['center'])
            yaw = float(region.get('yaw', 0.0))
            marker.pose.orientation.z = math.sin(yaw/2)
            marker.pose.orientation.w = math.cos(yaw/2)
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, region['size'])
            marker.color.r = 1.0 if region['kind'] == 'stair' else 0.1
            marker.color.g = 0.5
            marker.color.b = 0.1 if region['kind'] == 'stair' else 1.0
            marker.color.a = 0.22
            array.markers.append(marker)
        self.markers.publish(array)

    def destroy_node(self):
        with self.lock:
            if self.core.active:
                self.core.cancel(self.now())
                self.publish_output(self.core.tick(self.now(), self.period))
            self.core.close()
        self.server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentNavigation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
