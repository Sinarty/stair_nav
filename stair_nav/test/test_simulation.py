"""Adapter cache/TTL behavior is independent of the navigation producer."""
from dataclasses import replace
import math
from pathlib import Path

import pytest

from stair_nav.core.config import load_route
from stair_nav.core.models import Command
from stair_nav.demo.simulation import Simulator


@pytest.fixture
def sim():
    route = load_route(Path(__file__).resolve().parent / 'fixtures' / 'legacy' / 'demo_route.yaml')
    return Simulator(route)


def tracking(stamp=0.0, execution_id='run-1'):
    return Command(stamp, stamp+.2, 'task', execution_id, 'AB', 'entry', 'stair', 'TRACK', .5, 0.0, True)


def test_only_valid_unexpired_commands_move_pose(sim):
    initial = sim.pose
    sim.step(.05, .05)
    assert sim.pose.x == initial.x
    assert sim.accept(tracking(.05))
    sim.step(.05, .1)
    assert sim.pose.x == pytest.approx(.025)
    sim.step(.05, .3)  # No refresh: cache has expired.
    assert sim.pose.x == pytest.approx(.025)
    assert not sim.motion_status(.3).hold_confirmed
    sim.step(.05, .6)
    assert sim.motion_status(.6).hold_confirmed


@pytest.mark.parametrize('changes', [
    {'task_id': ''}, {'execution_id': ''}, {'segment_id': 'unknown'},
    {'vx': math.nan}, {'wz': math.inf}, {'stamp': math.nan},
    {'valid_until': math.inf}, {'valid_until': 0.0}, {'valid_until': 10.0},
    {'mode': 'unsupported'}, {'behavior': 'JUMP'}, {'behavior': 'HOLD'},
    {'vx': 100.0}, {'vx': -.1}, {'vx': 0.0, 'wz': .1},
])
def test_malformed_or_unsupported_commands_never_integrate(sim, changes):
    assert sim.accept(replace(tracking(), **changes)) is False
    sim.step(.1, .1)
    assert math.isfinite(sim.pose.x) and sim.pose.x == 0.0


def test_new_execution_retires_old_command_id_even_with_later_stamp(sim):
    assert sim.accept(tracking())
    assert sim.accept(tracking(.05, 'run-2'))
    assert not sim.accept(tracking(.1, 'run-1'))
    assert sim.command.execution_id == 'run-2'


def test_timestamp_tie_revocation_wins(sim):
    command = tracking()
    assert sim.accept(command)
    hold = replace(command, valid=False, behavior='HOLD', vx=0.0)
    assert sim.accept(hold)
    assert not sim.accept(command)
    sim.step(.05, .05)
    assert sim.pose.x == 0.0


def test_hardware_capability_check_is_independent_of_navigation(sim):
    sim.settings['capabilities'] = []
    assert not sim.accept(tracking())
