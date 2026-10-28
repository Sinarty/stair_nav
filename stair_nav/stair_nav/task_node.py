"""Serial task orchestration over the reusable segment Action."""
from copy import deepcopy
from pathlib import Path
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from stair_nav_interfaces.action import NavigateSegment
from stair_nav_interfaces.msg import SegmentStatus
from stair_nav.core.config import load_route
from stair_nav.ros_utils import durable_qos


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        default = str(Path(get_package_share_directory('stair_nav'))/'config'/'demo_route.yaml')
        self.declare_parameter('route_file', default)
        self.declare_parameter('autostart', True)
        self.route = load_route(self.get_parameter('route_file').value)
        self.sequence = self.route['sequence']
        self.publisher = self.create_publisher(SegmentStatus, '/task/status', durable_qos())
        self.client = ActionClient(self, NavigateSegment, '/navigate_segment')
        self.create_service(Trigger, '~/start', self.start_service)
        self.create_service(Trigger, '~/cancel', self.cancel_service)
        self.status = SegmentStatus(state='IDLE', navigation_stopped=True)
        self.active = False
        self.generation = 0
        self.index = 0
        self.goal_handle = None
        self.cancel_requested = False
        self.autostart_pending = bool(self.get_parameter('autostart').value)
        self.last_log_state = None
        self.last_contact = self.now()
        self.transport_timeout = float(self.route.get('task_transport_timeout', 5.0))
        self.transport_error = ''
        self.create_timer(0.2, self.timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self):
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status.header.frame_id = self.route['frame']
        self.publisher.publish(self.status)
        key = (self.status.task_id, self.status.segment_id, self.status.state)
        if key != self.last_log_state:
            self.get_logger().info(
                f'task={self.status.task_id} segment={self.status.segment_id} '
                f'state={self.status.state} error={self.status.error_code} '
                f'{self.status.error_message}')
            self.last_log_state = key

    def timer(self):
        if (self.active and not self.transport_error and
                self.now()-self.last_contact > self.transport_timeout):
            self.transport_error = 'No action acceptance/feedback within transport deadline'
            self.cancel_requested = True
            self.status.state = 'TRANSPORT_TIMEOUT'
            self.status.error_code = 'ACTION_TRANSPORT_TIMEOUT'
            self.status.error_message = self.transport_error
            # The unavailable server has not acknowledged revocation or hold.
            self.status.navigation_stopped = False
            self.status.hold_confirmed = False
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        if self.autostart_pending and self.client.server_is_ready():
            self.autostart_pending = False
            self.start_task()
        self.publish()

    def start_service(self, request, response):
        self.autostart_pending = False
        if self.active:
            response.success = False
            response.message = 'BUSY: one task is active; cancel and await terminal result first'
        elif not self.client.server_is_ready():
            response.success = False
            response.message = 'segment action server is unavailable'
        else:
            self.start_task()
            response.success = True
            response.message = f'task_id={self.status.task_id}'
        return response

    def start_task(self):
        self.generation += 1
        self.active = True
        self.index = 0
        self.cancel_requested = False
        self.transport_error = ''
        self.last_contact = self.now()
        self.goal_handle = None
        self.status = SegmentStatus(task_id=str(uuid.uuid4()), state='DISPATCHING',
                                    navigation_stopped=True)
        self.dispatch(self.generation)

    def dispatch(self, generation):
        if generation != self.generation or not self.active:
            return
        segment_id = self.sequence[self.index]
        execution_id = str(uuid.uuid4())
        self.status.segment_id = segment_id
        self.status.execution_id = execution_id
        self.last_contact = self.now()
        self.status.state = 'DISPATCHING'
        self.status.phase = ''
        self.status.progress = 0.0
        self.status.lateral_error = 0.0
        self.status.heading_error = 0.0
        self.status.navigation_stopped = True
        self.status.geometry_reached = False
        self.status.hold_confirmed = False
        self.status.error_code = ''
        self.status.error_message = ''
        goal = NavigateSegment.Goal(task_id=self.status.task_id,
                                    execution_id=execution_id, segment_id=segment_id)
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback(msg, generation, execution_id))
        future.add_done_callback(lambda f: self.accepted(f, generation, execution_id))
        self.publish()

    def feedback(self, message, generation, execution_id):
        if (generation != self.generation or not self.active or
                execution_id != self.status.execution_id or self.cancel_requested):
            return
        incoming = message.feedback.status
        if incoming.execution_id != execution_id or incoming.task_id != self.status.task_id:
            return
        self.last_contact = self.now()
        self.status = deepcopy(incoming)
        # A segment terminal feedback is not the terminal status of the route.
        # Only result() after the final configured segment may publish SUCCEEDED.
        if incoming.state in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            self.status.state = 'SEGMENT_' + incoming.state
        self.publish()

    def accepted(self, future, generation, execution_id):
        try:
            handle = future.result()
        except Exception as exc:
            if generation == self.generation and self.active:
                self.finish('FAILED', 'ACTION_TRANSPORT', str(exc))
            return
        if generation != self.generation or execution_id != self.status.execution_id:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.finish('CANCELED' if self.cancel_requested else 'FAILED',
                        'GOAL_REJECTED', 'segment server rejected the request (busy/invalid/reused ID)')
            return
        self.last_contact = self.now()
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result(f, generation, execution_id))
        if self.cancel_requested:
            handle.cancel_goal_async()

    def result(self, future, generation, execution_id):
        if (generation != self.generation or not self.active or
                execution_id != self.status.execution_id):
            return
        try:
            result = future.result().result
        except Exception as exc:
            self.finish('FAILED', 'ACTION_TRANSPORT', str(exc))
            return
        self.goal_handle = None
        self.status.navigation_stopped = result.navigation_stopped
        self.status.hold_confirmed = result.hold_confirmed
        if self.transport_error:
            self.finish('FAILED', 'ACTION_TRANSPORT_TIMEOUT', self.transport_error)
        elif self.cancel_requested:
            self.finish('CANCELED', 'CANCELED', 'task cancellation finished; check hold_confirmed separately')
        elif not result.success:
            self.finish('FAILED', result.error_code, result.error_message)
        else:
            self.index += 1
            if self.index == len(self.sequence):
                self.status.progress = 1.0
                self.finish('SUCCEEDED', '', '')
            else:
                self.dispatch(generation)

    def finish(self, state, code, message):
        self.active = False
        self.status.state = state
        self.status.error_code = code
        self.status.error_message = message
        self.publish()

    def cancel_service(self, request, response):
        self.autostart_pending = False
        if not self.active:
            response.success = False
            response.message = 'no active task'
            return response
        self.cancel_requested = True
        if not self.transport_error:
            self.status.state = 'CANCELLING'
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        # A pending send_goal response is canceled by accepted() when it arrives.
        response.success = True
        response.message = 'cancel requested; navigation stop and physical hold are reported separately'
        self.publish()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
