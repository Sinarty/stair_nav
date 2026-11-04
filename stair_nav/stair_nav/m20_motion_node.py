"""M20 production ROS interface; leaves the manufacturer's locomotion running.

drdds is loaded at runtime from the firmware-matched vendor interface package.
There is deliberately no stand, lie-down, emergency-stop or SDK-mode command.
"""
from dataclasses import fields
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool

from stair_nav.core.config import load_route
from stair_nav.core.localization import timestamp_seconds
from stair_nav.core.m20_motion import M20Config, M20MotionAdapter, MotionObservation
from stair_nav.core.models import Command, MotionFeedback
from stair_nav.ros_utils import durable_qos, live_qos, seconds, stamp
from stair_nav_interfaces.msg import ControlCommand, MotionStatus


def native_header(message, sequence, now):
    """Handle the two documented MetaType layouts without changing wire types."""
    header = message.header
    header.frame_id = sequence  # Vendor field is a uint64 sequence, not a TF frame.
    value = stamp(now)
    if hasattr(header, 'stamp'):
        header.stamp.sec, header.stamp.nanosec = value.sec, value.nanosec
    elif hasattr(header, 'timestamp'):
        header.timestamp.sec, header.timestamp.nsec = value.sec, value.nanosec
    else:
        raise ValueError('Unsupported drdds MetaType: expected stamp or timestamp')
    return message


def motion_observation(message):
    """Names verified against the vendor's MotionInfoValue.msg source."""
    data = message.data
    return MotionObservation(
        timestamp_seconds(message.header), int(data.motion_state.state),
        int(data.gait_state.gait), float(data.vel_x), float(data.vel_y), float(data.vel_yaw))


class M20MotionNode(Node):
    def __init__(self):
        super().__init__('m20_motion_adapter')
        self.declare_parameter('route_file', '')
        route_file = self.get_parameter('route_file').value
        if not route_file:
            raise ValueError('route_file is required to establish the navigation frame')
        self.frame = load_route(route_file)['frame']
        defaults = M20Config()
        values = {}
        for field in fields(defaults):
            default = getattr(defaults, field.name)
            # ROS cannot infer an empty string-array parameter from [].
            if isinstance(default, tuple):
                if default:
                    self.declare_parameter(field.name, list(default))
                else:
                    self.declare_parameter(field.name, [], ParameterDescriptor(dynamic_typing=True))
                values[field.name] = tuple(self.get_parameter(field.name).value or ())
            else:
                self.declare_parameter(field.name, default)
                values[field.name] = self.get_parameter(field.name).value
        self.adapter = M20MotionAdapter(M20Config(**values), self.frame)
        self.declare_parameter('publish_rate', 20.0)
        rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(rate) or not 10.0 <= rate <= 100.0:
            raise ValueError('publish_rate must be between 10 and 100 Hz; default is 20 Hz')
        topics = dict(
            command_topic='/navigation/command', motion_status_topic='/motion/status',
            nav_cmd_topic='/NAV_CMD', gait_topic='/GAIT', motion_info_topic='/MOTION_INFO',
            location_status_topic='/LOCATION_STATUS', imu_topic='/IMU',
            hard_estop_topic='/HES_STATUS', charge_status_topic='/CHARGE_STATUS',
            confirmation_topic='/m20/confirmation', diagnostic_topic='/m20/motion_diagnostic')
        for name, default in topics.items():
            self.declare_parameter(name, default)
            topics[name] = self.get_parameter(name).value
        self.declare_parameter('location_status_type', 'drdds/msg/LocationStatus')
        names = dict(nav='drdds/msg/NavCmd', gait='drdds/msg/Gait',
                     motion='drdds/msg/MotionInfo', estop='drdds/msg/StdMsgInt32',
                     charge='drdds/msg/StdStatus')
        if self.adapter.config.require_location_status:
            names['location'] = self.get_parameter('location_status_type').value
        try:
            self.types = {key: get_message(name) for key, name in names.items()}
            # Fail before creating command publishers on a mismatched interface package.
            motion_observation(self.types['motion']())
            native_header(self.types['nav'](), 0, 0.)
            native_header(self.types['gait'](), 0, 0.)
            self.types['nav']().data.x_vel
            self.types['gait']().data.gait
            if self.adapter.config.require_location_status:
                timestamp_seconds(self.types['location']().meta)
                self.types['location']().data.total_status
            self.types['estop']().value
            self.types['charge']().state
            self.types['charge']().error_code
        except (ImportError, ModuleNotFoundError, AttributeError, ValueError, TypeError) as exc:
            raise RuntimeError(
                'Install/source the firmware-matched drdds package, including LocationStatus. '
                'The public package may omit this type; obtain it from the manufacturer. '
                'Do not substitute the RL SDK or locally invent vendor message definitions. '
                f'Interface check failed: {exc}') from exc
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.nav = self.create_publisher(self.types['nav'], topics['nav_cmd_topic'], live_qos())
        self.gaits = self.create_publisher(self.types['gait'], topics['gait_topic'], live_qos())
        self.status = self.create_publisher(MotionStatus, topics['motion_status_topic'], live_qos(5))
        self.diagnostic = self.create_publisher(String, topics['diagnostic_topic'], live_qos(5))
        self.create_subscription(ControlCommand, topics['command_topic'], self.on_command, live_qos())
        self.create_subscription(self.types['motion'], topics['motion_info_topic'], self.on_motion, sensor_qos)
        if self.adapter.config.require_location_status:
            self.create_subscription(self.types['location'], topics['location_status_topic'], self.on_location, sensor_qos)
        self.create_subscription(Imu, topics['imu_topic'], self.on_imu, sensor_qos)
        # GOS retains the last hard-estop state for late subscribers.
        # Receipt still expires under safety_timeout; retained data is not a heartbeat.
        self.create_subscription(self.types['estop'], topics['hard_estop_topic'], self.on_estop, durable_qos(5))
        self.create_subscription(self.types['charge'], topics['charge_status_topic'], self.on_charge, sensor_qos)
        self.create_subscription(MotionStatus, topics['confirmation_topic'], self.on_confirmation, live_qos(5))
        self.create_service(SetBool, '~/enable', self.enable)
        self.add_on_set_parameters_callback(self.startup_parameters_only)
        self.sequence = 0
        self.last_input_error = ''
        # ROS time can pause or jump; the command watchdog must keep running.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1. / rate, self.tick, clock=self.watchdog_clock)
        self.get_logger().info(
            'M20 adapter starts disabled. dry_run suppresses ALL /NAV_CMD and /GAIT writes; '
            'prepare navigation usage mode and state=17 externally, then explicitly enable.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def startup_parameters_only(parameters):
        return SetParametersResult(
            successful=False,
            reason='Adapter parameters are startup-only; use ~/enable=false to revoke, then restart with new configuration')

    def invalid_input(self, source, exception):
        self.last_input_error = f'{source}: {exception}'
        self.adapter.enable(False, self.now(), time.monotonic())
        self.get_logger().error(self.last_input_error)

    def on_motion(self, message):
        try:
            self.adapter.observe_motion(motion_observation(message))
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalid_input('MOTION_INFO', exc)

    def on_location(self, message):
        if not self.adapter.config.require_location_status:
            return
        try:
            self.adapter.observe_location(timestamp_seconds(message.meta), int(message.data.total_status))
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalid_input('LOCATION_STATUS', exc)

    def on_imu(self, message):
        try:
            v = message.angular_velocity
            # Vector norm is invariant under body/world rotation. z alone is not world yaw.
            self.adapter.observe_imu(seconds(message.header.stamp), math.hypot(v.x, v.y, v.z))
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalid_input('IMU', exc)

    def on_estop(self, message):
        try:
            self.adapter.observe_estop(time.monotonic(), int(message.value) != 0)
        except (ValueError, TypeError, AttributeError) as exc:
            self.invalid_input('HES_STATUS', exc)

    def on_charge(self, message):
        self.adapter.observe_charge(time.monotonic(), int(message.state), int(message.error_code))

    def on_command(self, message):
        if message.header.frame_id != self.frame:
            self.invalid_input('ControlCommand', ValueError('wrong navigation frame'))
            return
        command = Command(
            stamp=seconds(message.header.stamp), valid_until=seconds(message.valid_until),
            task_id=message.task_id, execution_id=message.execution_id,
            segment_id=message.segment_id, phase=message.phase, mode=message.mode,
            behavior=message.behavior, vx=message.vx, wz=message.wz, valid=message.valid)
        self.adapter.accept(command, self.now(), time.monotonic())

    def on_confirmation(self, message):
        if message.header.frame_id != self.frame:
            return
        self.adapter.confirm(MotionFeedback(
            stamp=seconds(message.header.stamp), execution_id=message.execution_id,
            segment_id=message.segment_id, mode=message.mode,
            ready=message.ready, stable=message.stable,
            hold_confirmed=message.hold_confirmed, exit_confirmed=message.exit_confirmed,
            fault_code=message.fault_code, fault_message=message.fault_message), self.now())

    def enable(self, request, response):
        response.success, response.message = self.adapter.enable(
            bool(request.data), self.now(), time.monotonic())
        return response

    def publish_output(self, output):
        now = self.now()
        if not self.adapter.config.dry_run and output.publish_velocity:
            self.sequence += 1
            command = native_header(self.types['nav'](), self.sequence, now)
            command.data.x_vel, command.data.y_vel, command.data.yaw_vel = output.vx, 0., output.wz
            self.nav.publish(command)
        if not self.adapter.config.dry_run and output.gait_request is not None:
            self.sequence += 1
            command = native_header(self.types['gait'](), self.sequence, now)
            command.data.gait = output.gait_request
            self.gaits.publish(command)
        value = output.feedback
        feedback = MotionStatus()
        feedback.header.frame_id, feedback.header.stamp = self.frame, stamp(value.stamp)
        for name in ('execution_id', 'segment_id', 'mode', 'ready', 'stable',
                     'hold_confirmed', 'exit_confirmed', 'fault_code', 'fault_message'):
            setattr(feedback, name, getattr(value, name))
        feedback.capabilities, feedback.supported_modes = list(value.capabilities), list(value.supported_modes)
        self.status.publish(feedback)
        diagnostic = dict(output.diagnostics, input_error=self.last_input_error)
        self.diagnostic.publish(String(data=json.dumps(diagnostic, ensure_ascii=False, allow_nan=False)))

    def tick(self):
        self.publish_output(self.adapter.tick(self.now(), time.monotonic()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = M20MotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.adapter.enable(False, node.now(), time.monotonic())
                # One final revocation; measured hold still needs subsequent vendor feedback.
                node.tick()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
