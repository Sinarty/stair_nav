"""Hardware-free adapter: integrates the actual candidate command stream."""
from pathlib import Path
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path as PathMsg, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from std_srvs.srv import SetBool
from stair_nav_interfaces.msg import ControlCommand, MotionStatus
from stair_nav.core.config import load_route
from stair_nav.core.models import Command
from stair_nav.core.simulation import Simulator
from stair_nav.ros_utils import durable_qos, live_qos, seconds, stamp, pose_msg
import yaml


class SimExecutor(Node):
    def __init__(self):
        super().__init__('sim_executor')
        default = str(Path(get_package_share_directory('stair_nav'))/'config'/'demo_route.yaml')
        self.declare_parameter('route_file', default)
        self.route = load_route(self.get_parameter('route_file').value)
        self.sim = Simulator(self.route)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/localization/pose', live_qos(5))
        self.motion_publisher = self.create_publisher(MotionStatus, '/motion/status', live_qos(5))
        self.trajectory_publisher = self.create_publisher(PathMsg, '/simulation/trajectory', durable_qos())
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/simulation/platform_grid', durable_qos())
        self.create_subscription(ControlCommand, '/navigation/command', self.command, live_qos())
        self.trajectory = PathMsg()
        self.trajectory.header.frame_id = self.route['frame']
        self.tick_count = 0
        self.period = float(self.route.get('control_period', 0.05))
        self.last_tick = self.now()
        for fault_name in self.sim.faults:
            self.declare_parameter(fault_name, False)
            self.sim.faults[fault_name] = bool(self.get_parameter(fault_name).value)
            self.create_service(SetBool, f'~/{fault_name}',
                                lambda req, res, key=fault_name: self.set_fault(key, req, res))
        self.create_timer(self.period, self.tick)
        self.publish_grid()
        self.get_logger().info('SYNTHETIC simulation: velocity integration only; no stair dynamics or real hardware')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_fault(self, name, request, response):
        self.sim.faults[name] = request.data
        response.success = True
        response.message = f'{name}={request.data}'
        self.get_logger().info(response.message)
        return response

    def command(self, msg):
        if msg.header.frame_id != self.route['frame']:
            self.get_logger().warning('ignored command with wrong map frame')
            return
        now = self.now()
        issued = seconds(msg.header.stamp)
        expires = seconds(msg.valid_until)
        if (issued > now + 0.1 or expires <= now or
                not all(math.isfinite(value) for value in (issued, expires, msg.vx, msg.wz))):
            # Reject before accept(): a future packet must not poison the monotone stamp cache.
            self.get_logger().warning('ignored expired, future, or nonfinite command')
            return
        command = Command(
            stamp=seconds(msg.header.stamp), valid_until=seconds(msg.valid_until),
            task_id=msg.task_id, execution_id=msg.execution_id, segment_id=msg.segment_id,
            phase=msg.phase, mode=msg.mode, behavior=msg.behavior, vx=msg.vx, wz=msg.wz,
            valid=msg.valid)
        self.sim.accept(command)

    def tick(self):
        now = self.now()
        dt = max(0.001, min(0.2, now-self.last_tick))
        self.last_tick = now
        pose = self.sim.step(dt, now)
        if not self.sim.faults['localization_dropout']:
            msg = pose_msg(pose, self.route['frame'], now)
            self.pose_publisher.publish(msg)
            # Visualization TF for the same navigation reference as PoseStamped;
            # this is deliberately not a body frame or a fixed height offset.
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = 'simulation_nav_reference'
            transform.transform.translation.x = pose.x
            transform.transform.translation.y = pose.y
            transform.transform.translation.z = pose.z
            transform.transform.rotation = msg.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            if self.tick_count % 2 == 0:
                self.trajectory.poses.append(msg)
                self.trajectory.poses = self.trajectory.poses[-4000:]
                self.trajectory.header.stamp = stamp(now)
                self.trajectory_publisher.publish(self.trajectory)
        feedback = self.sim.motion_status(now)
        motion = MotionStatus()
        motion.header.frame_id = self.route['frame']
        motion.header.stamp = stamp(feedback.stamp)
        for field in ('execution_id', 'segment_id', 'mode', 'ready', 'exit_confirmed',
                      'stable', 'hold_confirmed', 'fault_code', 'fault_message'):
            setattr(motion, field, getattr(feedback, field))
        motion.capabilities = list(feedback.capabilities)
        motion.supported_modes = list(feedback.supported_modes)
        self.motion_publisher.publish(motion)
        self.tick_count += 1

    def publish_grid(self):
        for segment in self.route['segments'].values():
            if segment['backend'] != 'grid_astar':
                continue
            filename = Path(self.route['_base_dir'])/segment['grid_file']
            grid = yaml.safe_load(filename.read_text())
            msg = OccupancyGrid()
            msg.header.frame_id = self.route['frame']
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.map_load_time = msg.header.stamp
            msg.info.resolution = float(grid['resolution'])
            msg.info.width, msg.info.height = grid['width'], grid['height']
            msg.info.origin.position.x = float(grid['origin'][0])
            msg.info.origin.position.y = float(grid['origin'][1])
            msg.info.origin.position.z = float(grid['height_z'])
            msg.info.origin.orientation.w = 1.0
            data = grid['data']
            msg.data = [int(x) for row in data for x in row] if isinstance(data[0], list) else data
            self.grid_publisher.publish(msg)
            break


def main(args=None):
    rclpy.init(args=args)
    node = SimExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
