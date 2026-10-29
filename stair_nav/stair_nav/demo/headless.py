"""Deterministic, ROS-independent exercise of the production core and adapter."""
import argparse
from concurrent.futures import Future
from dataclasses import asdict
import json
import math
from pathlib import Path

from ..core.config import load_route
from ..core.engine import SegmentExecutor, State
from .simulation import Simulator


class InlineExecutor:
    """Deterministic test scheduler; ROS uses the real asynchronous worker pool."""
    def submit(self, function, *args, **kwargs):
        future = Future()
        try:
            future.set_result(function(*args, **kwargs))
        except Exception as exc:
            future.set_exception(exc)
        return future


class Harness:
    def __init__(self, route, dt=.05):
        self.route = route
        self.dt = dt
        self.now = 0.0
        self.sim = Simulator(route)
        self.engine = SegmentExecutor(route, planning_executor=InlineExecutor())
        self.events = []
        self.trajectory = []
        self.command_count = 0
        self.output = None
        self._last_event = None
        self._generation = 0

    def start(self, segment_id, task_id='headless-demo'):
        self._generation += 1
        self.engine.update_pose(self.sim.pose)
        return self.engine.start(task_id, f'{task_id}-{self._generation}', segment_id, self.now)

    def step(self, publish_motion=True):
        if not self.sim.faults['localization_dropout']:
            self.engine.update_pose(self.sim.pose)
        if publish_motion:
            self.engine.update_motion(self.sim.motion_status(self.now))
        self.output = self.engine.tick(self.now, self.dt)
        self.sim.accept(self.output.command)
        self.sim.step(self.dt, self.now+self.dt)
        self.now += self.dt
        self.command_count += bool(self.output.command.valid)
        self.trajectory.append((self.sim.pose.x, self.sim.pose.y, self.sim.pose.z))
        event = (self.output.status.segment_id, self.output.status.state)
        if event != self._last_event:
            self.events.append(dict(time=round(self.now, 3), **asdict(self.output.status)))
            self._last_event = event
        return self.output

    def until_terminal(self, limit=120.0, hook=None):
        deadline = self.now + limit
        while self.engine.active and self.now < deadline:
            if hook:
                hook(self)
            self.step()
        if self.engine.active:
            raise RuntimeError('Harness deadline exceeded')
        return self.output

    def run_sequence(self, hook=None):
        results = []
        approach_result = None
        approach = self.route.get('approach_segment')
        if approach:
            first = self.route['segments'][self.route['sequence'][0]]
            start = self.route['nodes'][first['start']]
            pose, target, arrival = self.sim.pose, start['position'], start['arrival']
            yaw_error = math.atan2(math.sin(pose.yaw-start['yaw']), math.cos(pose.yaw-start['yaw']))
            at_start = (math.hypot(pose.x-target[0], pose.y-target[1]) <= arrival['xy'] and
                        abs(pose.z-target[2]) <= arrival['z'] and abs(yaw_error) <= arrival['yaw'])
            if not at_start:
                self.start(approach)
                approach_result = asdict(self.until_terminal(hook=hook).status)
        sequence = self.route['sequence'] if (approach_result is None or
                    approach_result['state'] == State.SUCCEEDED) else []
        for segment_id in sequence:
            self.start(segment_id)
            result = self.until_terminal(hook=hook)
            results.append(asdict(result.status))
            if result.status.state != State.SUCCEEDED:
                break
        return dict(success=len(results) == len(self.route['sequence'])
                    and all(s['state'] == State.SUCCEEDED for s in results),
                    simulated_seconds=round(self.now, 3),
                    valid_command_samples=self.command_count,
                    final_pose=asdict(self.sim.pose), approach_result=approach_result,
                    results=results, transitions=self.events)


def default_route():
    source = Path(__file__).resolve().parent / 'config' / 'route.yaml'
    if source.exists():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('stair_nav')) / 'demo' / 'config' / 'route.yaml')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default=default_route())
    parser.add_argument('--dt', type=float, default=.05)
    parser.add_argument('--fault', choices=['not_ready', 'localization_dropout', 'execution_fault', 'missing_exit', 'unstable'])
    parser.add_argument('--fault-at', type=float, default=0.0)
    parser.add_argument('--cancel-at', type=float)
    parser.add_argument('--output', help='Optional JSON report file')
    options = parser.parse_args(args)
    harness = Harness(load_route(options.route), options.dt)

    def inject(h):
        if options.fault and h.now >= options.fault_at:
            h.sim.faults[options.fault] = True
        if options.cancel_at is not None and h.now >= options.cancel_at:
            h.engine.cancel(h.now)

    try:
        report = harness.run_sequence(hook=inject)
    finally:
        harness.engine.close()
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if options.output:
        Path(options.output).write_text(rendered+'\n', encoding='utf-8')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
