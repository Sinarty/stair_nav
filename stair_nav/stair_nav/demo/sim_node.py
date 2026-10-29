"""Hardware-free adapter: integrates the actual candidate command stream."""
from array import array
from pathlib import Path
import math

import rclpy
from nav_msgs.msg import Path as PathMsg, OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from stair_nav_interfaces.msg import ControlCommand, MotionStatus
from stair_nav.core.config import load_route
from stair_nav.core.models import Command
from .simulation import Simulator
from .headless import default_route
from stair_nav.ros_utils import durable_qos, live_qos, seconds, stamp, pose_msg
import yaml


def terrain_markers(grid, segment_id, stair, frame, timestamp):
    """Render walkable terrain cells while keeping planning boundaries hidden."""
    width, height = int(grid['width']), int(grid['height'])
    resolution = float(grid['resolution'])
    ox, oy = map(float, grid['origin'])
    rows = grid['data']
    if not isinstance(rows[0], list):
        rows = [rows[y*width:(y+1)*width] for y in range(height)]
    elevations = grid.get('heights')
    if elevations is None:
        elevations = [[float(grid['height_z'])]*width for _ in range(height)]
    free_heights = [float(elevations[y][x]) for y in range(height) for x in range(width)
                    if 0 <= rows[y][x] < 50]
    base = min(free_heights, default=float(grid.get('height_z', 0.)))
    groups = {}
    lines = Marker()
    lines.header.frame_id, lines.header.stamp = frame, timestamp
    lines.ns, lines.id = f'{segment_id}_cell_edges', 0
    lines.type, lines.action = Marker.LINE_LIST, Marker.ADD
    lines.pose.orientation.w = 1.
    lines.scale.x = .008
    lines.color.r, lines.color.g, lines.color.b, lines.color.a = .1, .15, .2, .8
    edges = set()
    for y in range(height):
        for x in range(width):
            if rows[y][x] >= 50 or rows[y][x] < 0:
                continue
            top = float(elevations[y][x])
            thickness = max(.04, top-base+.04) if stair else .04
            center_z = top-thickness/2
            key = round(thickness, 6)
            if key not in groups:
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = frame, timestamp
                marker.ns, marker.id = f'{segment_id}_cells', len(groups)
                marker.type, marker.action = Marker.CUBE_LIST, Marker.ADD
                marker.pose.orientation.w = 1.
                marker.scale.x, marker.scale.y, marker.scale.z = resolution, resolution, thickness
                if stair:
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = .95, .57, .18, .9
                else:
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = .3, .62, .78, .72
                groups[key] = marker
            groups[key].points.append(Point(x=ox+(x+.5)*resolution,
                                             y=oy+(y+.5)*resolution, z=center_z))
            corners = [(ox+x*resolution, oy+y*resolution),
                       (ox+(x+1)*resolution, oy+y*resolution),
                       (ox+(x+1)*resolution, oy+(y+1)*resolution),
                       (ox+x*resolution, oy+(y+1)*resolution)]
            for a, b in zip(corners, corners[1:]+corners[:1]):
                edge = tuple(sorted((tuple(round(v, 8) for v in a), tuple(round(v, 8) for v in b)))) + (round(top, 8),)
                if edge not in edges:
                    edges.add(edge)
                    lines.points.extend((Point(x=a[0], y=a[1], z=top+.012),
                                         Point(x=b[0], y=b[1], z=top+.012)))
    return [*groups.values(), lines]


class SimExecutor(Node):
    def __init__(self):
        super().__init__('sim_executor')
        default = default_route()
        self.declare_parameter('route_file', default)
        # An empty array retains simulation.initial_pose/initial_node. Dynamic
        # typing accommodates ROS's ambiguous type for an empty array; a
        # nonempty override is validated as finite numeric XYZYaw by Simulator.
        self.declare_parameter('initial_pose', [], ParameterDescriptor(dynamic_typing=True))
        self.route = load_route(self.get_parameter('route_file').value)
        initial_pose = self.get_parameter('initial_pose').value
        if isinstance(initial_pose, (list, tuple, array)):
            initial_pose = list(initial_pose) or None
        self.sim = Simulator(self.route, initial_pose=initial_pose)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/localization/pose', live_qos(5))
        self.motion_publisher = self.create_publisher(MotionStatus, '/motion/status', live_qos(5))
        self.trajectory_publisher = self.create_publisher(PathMsg, '/simulation/trajectory', durable_qos())
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/simulation/platform_grid', durable_qos())
        self.grid_publishers = {}
        self.terrain_publisher = self.create_publisher(MarkerArray, '/simulation/terrain_grid', durable_qos())
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
        terrain = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        terrain.markers.append(clear)
        legacy_published = False
        timestamp = self.get_clock().now().to_msg()
        for segment_id, segment in self.route['segments'].items():
            if segment['backend'] not in ('grid_astar', 'height_grid_astar'):
                continue
            filename = Path(self.route['_base_dir'])/segment['grid_file']
            grid = yaml.safe_load(filename.read_text())
            terrain.markers.extend(terrain_markers(grid, segment_id,
                segment['backend'] == 'height_grid_astar', self.route['frame'], timestamp))
            if segment['backend'] == 'height_grid_astar':
                continue  # A flat OccupancyGrid cannot encode the stair heights.
            msg = OccupancyGrid()
            msg.header.frame_id = self.route['frame']
            msg.header.stamp = timestamp
            msg.info.map_load_time = msg.header.stamp
            msg.info.resolution = float(grid['resolution'])
            msg.info.width, msg.info.height = grid['width'], grid['height']
            msg.info.origin.position.x = float(grid['origin'][0])
            msg.info.origin.position.y = float(grid['origin'][1])
            msg.info.origin.position.z = float(grid['height_z'])
            msg.info.origin.orientation.w = 1.0
            data = grid['data']
            msg.data = [int(x) for row in data for x in row] if isinstance(data[0], list) else data
            if segment_id not in self.grid_publishers:
                self.grid_publishers[segment_id] = self.create_publisher(
                    OccupancyGrid, f'/simulation/grids/{segment_id}', durable_qos())
            self.grid_publishers[segment_id].publish(msg)
            if not legacy_published:
                self.grid_publisher.publish(msg)
                legacy_published = True
        self.terrain_publisher.publish(terrain)


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
