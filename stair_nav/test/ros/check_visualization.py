#!/usr/bin/env python3
"""Check grid visualization messages without starting a ROS graph.

Run after sourcing ROS and the stair_nav install, like check_callbacks.py.
"""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import yaml
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from stair_nav.demo.sim_node import SimExecutor, terrain_markers
from stair_nav.core.config import load_route
from stair_nav.core.engine import State
from stair_nav.demo import Harness
from stair_nav.ros_utils import pose_from_msg, pose_msg


class ReferencePointVisualizationTests(unittest.TestCase):
    def test_reference_arrow_moves_and_turns_between_route_legs(self):
        config = Path(__file__).resolve().parents[2] / 'stair_nav' / 'demo' / 'config'
        harness = Harness(load_route(config / 'route.yaml'))
        try:
            for sid, heading in (('APPROACH_A', math.atan2(-.5, -4.)),
                                 ('AB', math.pi / 2), ('BC', 0.)):
                self.assertTrue(harness.start(sid))
                references = []
                for _ in range(2000):
                    output = harness.step()
                    if output.command.valid and output.target is not None:
                        message = pose_msg(output.target, 'map', harness.now)
                        references.append(pose_from_msg(message))
                    if not harness.engine.active:
                        break
                self.assertEqual(output.status.state, State.SUCCEEDED, output.status)
                self.assertGreater(len(references), 20)
                # Initial lateral correction can point off the segment axis;
                # the middle of each straight leg should show its travel heading.
                self.assertAlmostEqual(references[len(references) // 2].yaw,
                                       heading, delta=.05)
                first, last = references[0], references[-1]
                self.assertGreater(math.dist((first.x, first.y), (last.x, last.y)), 1.)
                # At the node, the arrow also shows the requested arrival yaw.
                terminal = pose_from_msg(pose_msg(output.target, 'map', harness.now))
                self.assertAlmostEqual(terminal.yaw, math.pi / 2, delta=1e-6)
        finally:
            harness.engine.close()


class TerrainVisualizationTests(unittest.TestCase):
    def test_stair_free_cell_tops_follow_each_input_height(self):
        grid = dict(width=3, height=2, resolution=.1, origin=[-4., 0.],
                    data=[[100, 0, 0], [100, 0, 0]],
                    heights=[[0., .2, .4], [0., .2, .4]])
        markers = terrain_markers(grid, 'AB', True, 'map', Time(sec=1))
        cells = [marker for marker in markers if marker.type == Marker.CUBE_LIST]
        self.assertEqual(sum(len(marker.points) for marker in cells), 4)
        free_tops = sorted(round(point.z+marker.scale.z/2, 8)
                           for marker in cells if marker.color.r > .9
                           for point in marker.points)
        self.assertEqual(free_tops, [.2, .2, .4, .4])
        edges = [marker for marker in markers if marker.type == Marker.LINE_LIST]
        self.assertEqual(len(edges), 1)
        self.assertTrue(edges[0].points)
        self.assertEqual({round(point.z, 3) for point in edges[0].points}, {.212, .412})
        self.assertTrue(all(marker.header.frame_id == 'map' for marker in markers))

    def test_flat_grid_tiles_keep_the_configured_platform_height(self):
        grid = dict(width=2, height=2, resolution=.1, origin=[0., 0.],
                    height_z=1.2, data=[[0, 0], [0, 0]])
        markers = terrain_markers(grid, 'BC', False, 'map', Time())
        cells = [marker for marker in markers if marker.type == Marker.CUBE_LIST]
        self.assertEqual(len(cells), 1)
        self.assertEqual(len(cells[0].points), 4)
        self.assertTrue(all(abs(point.z+cells[0].scale.z/2-1.2) < 1e-9
                            for point in cells[0].points))

    def test_all_platform_maps_and_both_stair_grids_are_published(self):
        with TemporaryDirectory() as directory:
            folder = Path(directory)
            flat = dict(width=2, height=2, resolution=.1, origin=[0., 0.],
                        height_z=0., data=[[0, 0], [0, 0]])
            stair = dict(flat, heights=[[0., .2], [0., .2]])
            maps = {'APPROACH_A': flat, 'AB': stair,
                    'BC': dict(flat, height_z=1.2),
                    'CD': dict(stair, heights=[[1.2, 1.4], [1.2, 1.4]])}
            segments = {}
            for segment_id, data in maps.items():
                filename = f'{segment_id}.yaml'
                (folder/filename).write_text(yaml.safe_dump(data))
                segments[segment_id] = dict(
                    backend='height_grid_astar' if segment_id in ('AB', 'CD') else 'grid_astar',
                    grid_file=filename)
            published = {}
            def publisher(topic):
                published[topic] = []
                return SimpleNamespace(publish=published[topic].append)
            harness = SimpleNamespace(
                route=dict(frame='map', _base_dir=str(folder), segments=segments),
                grid_publishers={}, grid_publisher=publisher('legacy'),
                terrain_publisher=publisher('terrain'),
                create_publisher=lambda kind, topic, qos: publisher(topic),
                get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
            SimExecutor.publish_grid(harness)
            self.assertEqual(set(harness.grid_publishers), {'APPROACH_A', 'BC'})
            self.assertEqual(len(published['/simulation/grids/APPROACH_A']), 1)
            self.assertEqual(len(published['/simulation/grids/BC']), 1)
            self.assertEqual(published['/simulation/grids/BC'][0].info.origin.position.z, 1.2)
            self.assertEqual(len(published['legacy']), 1)
            self.assertEqual(len(published['terrain']), 1)
            markers = published['terrain'][0].markers
            self.assertEqual(markers[0].action, Marker.DELETEALL)
            self.assertTrue({'AB_cells', 'CD_cells', 'APPROACH_A_cells', 'BC_cells'}.issubset(
                {marker.ns for marker in markers}))


if __name__ == '__main__':
    unittest.main(verbosity=2)
