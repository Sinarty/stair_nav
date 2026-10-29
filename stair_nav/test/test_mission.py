"""Mission races and deadlines with explicit events and a controllable clock."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import pytest

from stair_nav.core.config import load_route
from stair_nav.core.mission import MissionCore, MissionState as S, Readiness, load_mission, validate_mission
from stair_nav.core.models import NavError


@pytest.fixture
def setup():
    root = Path(__file__).resolve().parents[1]
    route = load_route(root/'config/demo_route.yaml')
    config = load_mission(root/'config/inspection_mission.yaml', route)
    config.pop('approach_segment', None)
    counter = iter(f'exec-{n}' for n in range(100))
    return route, config, MissionCore(route, config, id_factory=lambda: next(counter))


def ready(now=1., execution_id='', target_segment_id='AB', **kwargs):
    return Readiness(ready=True, released=True, execution_id=execution_id,
                     stamp=now, server_instance='server-1',
                     target_segment_id=target_segment_id, **kwargs)


def kinds(effects, kind):
    return [e for e in effects if e.kind == kind]


def dispatch(core, now=1., task='task-1'):
    assert core.start(core.config['route_id'], task, now)
    effects = core.tick(now, ready(now, target_segment_id=core.status.segment_id), True)
    assert len(kinds(effects, 'SEND')) == 1
    assert core.status.state == S.DISPATCHING
    return core.status.execution_id


def event(core, kind, now=1.1, **data):
    core.post(kind, core.status.task_id, core.status.execution_id, **data)
    return core.tick(now, ready(now, core.status.execution_id, core.status.segment_id), True)


def running(core, now=1.):
    eid = dispatch(core, now)
    event(core, 'ACCEPTED', now+.01, accepted=True)
    assert core.status.state == S.RUNNING
    return eid


def success(core, now):
    execution_id = core.status.execution_id
    effects = event(core, 'RESULT', now, status=4, success=True,
                    navigation_stopped=True, hold_confirmed=True)
    if core.status.state == S.WAITING_READY:
        # A later adapter update checks the newly selected segment. The result
        # update itself must never dispatch using the previous segment's facts.
        assert not kinds(effects, 'SEND')
        effects += core.tick(now, ready(now, execution_id, core.status.segment_id), True)
    return effects


def test_ordered_success_exactly_once_and_no_feedback_terminal(setup):
    _, _, core = setup
    dispatch(core)
    sent = ['AB']
    for index, sid in enumerate(['AB', 'BC', 'CD']):
        eid = core.status.execution_id
        event(core, 'ACCEPTED', 1.1+index, accepted=True)
        event(core, 'FEEDBACK', 1.2+index, stamp=1.2+index,
              segment_state='SUCCEEDED', phase='geometric_candidate', progress=1.)
        assert core.active and core.status.completed_segments == index
        effects = success(core, 1.3+index)
        sent += [e.segment_id for e in kinds(effects, 'SEND')]
        # Duplicate current/retired success cannot schedule twice or finish early.
        core.post('RESULT', 'task-1', eid, status=4, success=True,
                  navigation_stopped=True, hold_confirmed=True)
        assert not kinds(core.tick(1.31+index, ready(1.31+index), True), 'SEND')
    assert sent == ['AB', 'BC', 'CD']
    assert core.status.state == S.SUCCEEDED and core.status.completed_segments == 3
    assert not core.active
    assert len(kinds(effects, 'TERMINAL')) == 1
    assert not kinds(core.tick(6., ready(6.), True), 'TERMINAL')


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('code', ['PLAN_FAILED', 'EXECUTION_FAILED'])
def test_any_segment_failure_never_advances(setup, index, code):
    _, _, core = setup
    dispatch(core)
    for i in range(index):
        event(core, 'ACCEPTED', 1.1+i, accepted=True)
        success(core, 1.2+i)
    event(core, 'ACCEPTED', 1.1+index, accepted=True)
    effects = event(core, 'RESULT', 1.2+index, status=6, success=False,
                    navigation_stopped=True, hold_confirmed=False,
                    error_code=code, error_message='injected')
    assert core.status.state == S.FAILED  # fresh release in this tick satisfies cleanup
    assert core.status.completed_segments == index
    assert core.status.error_code == code
    assert not kinds(effects, 'SEND')


def test_rejected_goal_has_no_execution_cleanup_and_no_next_segment(setup):
    _, _, core = setup
    dispatch(core)
    effects = event(core, 'ACCEPTED', accepted=False)
    assert core.status.state == S.FAILED and core.status.error_code == 'GOAL_REJECTED'
    assert not kinds(effects, 'SEND') and not core.status.blocked


def test_start_busy_unknown_route_and_reused_task_rejected(setup):
    _, _, core = setup
    assert not core.start('unknown', 'task', 1.)
    dispatch(core)
    assert not core.start(core.config['route_id'], 'another', 1.01)
    event(core, 'ACCEPTED', accepted=False)
    assert not core.start(core.config['route_id'], 'task-1', 2.)
    assert core.start(core.config['route_id'], 'task-2', 2.)


@pytest.mark.parametrize('server,code', [(False, 'SERVICE_TIMEOUT'), (True, 'READY_TIMEOUT')])
def test_not_ready_stale_pose_or_motion_never_dispatches(setup, server, code):
    _, _, core = setup
    assert core.start(core.config['route_id'], 'task', 0.)
    assert not kinds(core.tick(0., Readiness(reason='pose/motion stale'), server), 'SEND')
    effects = core.tick(100., Readiness(reason='pose/motion stale'), server)
    assert core.status.state == S.FAILED and core.status.error_code == code
    assert not kinds(effects, 'SEND')


def test_cancel_before_dispatch_is_immediate_and_idempotent(setup):
    _, _, core = setup
    assert core.start(core.config['route_id'], 'task', 0.)
    assert core.cancel(.1)
    assert not core.cancel(.2)
    effects = core.tick(.2, ready(.2), True)
    assert core.status.state == S.CANCELED
    assert len(kinds(effects, 'TERMINAL')) == 1 and not kinds(effects, 'SEND')


def test_cancel_before_acceptance_cleans_late_handle_and_waits_release(setup):
    _, _, core = setup
    dispatch(core)
    assert core.cancel(1.01)
    assert not kinds(core.tick(1.02), 'CANCEL')  # no handle yet
    effects = event(core, 'ACCEPTED', 1.1, accepted=True)
    assert len(kinds(effects, 'CANCEL')) == 1
    assert core.status.cancel_sent and not core.status.cancel_accepted
    event(core, 'CANCEL_RESPONSE', 1.2, accepted=True)
    assert core.status.cancel_accepted
    core.post('RESULT', 'task-1', core.status.execution_id, status=5, success=False,
              navigation_stopped=True, hold_confirmed=False)
    effects = core.tick(1.3, Readiness(), True)
    assert core.status.state == S.CANCELING and not kinds(effects, 'TERMINAL')
    # Neither old, wrong-execution, nor pre-result hold confirmation releases it.
    for evidence in (ready(1., core.status.execution_id), ready(1.4, 'old')):
        assert not kinds(core.tick(1.4, evidence, True), 'TERMINAL')
        assert core.status.state == S.CANCELING
    effects = core.tick(1.5, ready(1.5, core.status.execution_id), True)
    assert core.status.state == S.CANCELED and core.status.hold_confirmed
    assert len(kinds(effects, 'TERMINAL')) == 1


def test_success_after_cancel_intent_cannot_advance(setup):
    _, _, core = setup
    running(core)
    assert core.cancel(1.1)
    effects = success(core, 1.2)
    assert core.status.state == S.CANCELED and core.status.completed_segments == 0
    assert not kinds(effects, 'SEND')
    event(core, 'CANCEL_RESPONSE', 1.3, accepted=False)
    assert core.status.state == S.CANCELED  # result won cancel-response race


def test_acceptance_timeout_then_late_acceptance_cancel_no_retry(setup):
    _, config, core = setup
    eid = dispatch(core)
    effects = core.tick(1.+config['timeouts']['acceptance'], ready(4.), True)
    assert core.status.state == S.CANCELING and core.status.error_code == 'GOAL_ACCEPT_TIMEOUT'
    assert not kinds(effects, 'SEND')
    effects = event(core, 'ACCEPTED', 4.1, accepted=True)
    assert len(kinds(effects, 'CANCEL')) == 1
    effects = success(core, 4.2)
    assert core.status.state == S.FAILED and core.status.error_code == 'GOAL_ACCEPT_TIMEOUT'
    assert core.status.execution_id == eid and not kinds(effects, 'SEND')


@pytest.mark.parametrize('stage,error', [('acceptance','ACCEPTANCE_UNRESOLVED'),
                                       ('response','CANCEL_RESPONSE_TIMEOUT'),
                                       ('terminal','CANCEL_TERMINAL_TIMEOUT'),
                                       ('cleanup','CLEANUP_TIMEOUT'),
                                       ('reject','CANCEL_REJECTED')])
def test_cancel_unknown_blocked_until_explicit_resynchronization(setup, stage, error):
    _, _, core = setup
    eid = dispatch(core)
    if stage != 'acceptance':
        event(core, 'ACCEPTED', 1.01, accepted=True)
    core.cancel(1.1)
    core.tick(1.1)
    if stage == 'terminal':
        event(core, 'CANCEL_RESPONSE', 1.2, accepted=True)
    elif stage == 'cleanup':
        core.post('RESULT', 'task-1', eid, status=5, success=False,
                  navigation_stopped=True, hold_confirmed=False)
        core.tick(1.2)
    elif stage == 'reject':
        event(core, 'CANCEL_RESPONSE', 1.2, accepted=False)
    effects = core.tick(100.)
    assert core.status.state == S.FAULT_BLOCKED and core.status.error_code == error
    assert not core.start(core.config['route_id'], 'task-2', 101.)
    assert not core.reset(101., Readiness())[0]
    # Pending acceptance is still consumed for cleanup even after Action aborted.
    if stage == 'acceptance':
        effects = event(core, 'ACCEPTED', 101., accepted=True)
        assert len(kinds(effects, 'CANCEL')) == 1
    if stage != 'cleanup':
        assert not core.reset(101., ready(101., eid))[0]
        event(core, 'RESULT', 102., status=5, success=False,
              navigation_stopped=True, hold_confirmed=True)
    assert core.status.state == S.FAULT_BLOCKED  # sticky, never silently auto-release
    assert not core.reset(103., ready(100., eid))[0]  # stale evidence
    assert core.reset(103., ready(103., eid))[0]
    assert core.status.state == S.IDLE and not core.busy
    assert core.start(core.config['route_id'], 'task-2', 104.)
    assert not kinds(core.tick(104.1), 'TERMINAL')


@pytest.mark.parametrize('timeout,code', [('execution','EXECUTION_TIMEOUT'),
                                         ('confirmation','CONFIRMATION_TIMEOUT'),
                                         ('feedback','FEEDBACK_TIMEOUT')])
def test_execution_confirmation_feedback_deadlines_revoke(setup, timeout, code):
    route, config, _ = setup
    config['timeouts'][timeout] = .2
    core = MissionCore(route, config)
    running(core)
    if timeout == 'confirmation':
        event(core, 'FEEDBACK', 1.02, stamp=1.02, segment_state='ARRIVAL_CONFIRM',
              phase='geometric_candidate', progress=1.)
    effects = core.tick(1.25)
    assert core.status.state == S.CANCELING and core.status.error_code == code
    assert len(kinds(effects, 'CANCEL')) == 1 and not kinds(effects, 'SEND')


def test_geometric_success_feedback_without_confirmed_result_never_advances(setup):
    _, _, core = setup
    running(core)
    for t in (1.1, 1.2, 1.3):
        effects = event(core, 'FEEDBACK', t, stamp=t, segment_state='ARRIVAL_CONFIRM',
                        phase='geometric_candidate', progress=1.)
        assert not kinds(effects, 'SEND') and core.status.completed_segments == 0
    assert core.status.state == S.RUNNING


@pytest.mark.parametrize('status,success_flag,stopped,hold', [(6,True,True,True),
                                                          (4,True,False,True),
                                                          (4,True,True,False)])
def test_success_contract_requires_ros_success_and_navigation_release(setup, status, success_flag, stopped, hold):
    _, _, core = setup
    running(core)
    effects = event(core, 'RESULT', 1.2, status=status, success=success_flag,
                    navigation_stopped=stopped, hold_confirmed=hold)
    assert core.status.state in (S.FAILED, S.CANCELING)
    assert core.status.completed_segments == 0 and not kinds(effects, 'SEND')


def test_stale_and_duplicate_events_cannot_mutate_new_mission(setup):
    _, _, core = setup
    old = dispatch(core)
    event(core, 'ACCEPTED', accepted=False)
    dispatch(core, 2., 'task-2')
    original = replace(core.status)
    core.post('RESULT', 'task-1', old, status=4, success=True,
              navigation_stopped=True, hold_confirmed=True)
    core.post('FEEDBACK', 'task-1', old, progress=1., phase='late')
    core.post('ACCEPTED', 'task-1', old, accepted=True)
    effects = core.tick(2.1)
    assert core.status == original
    assert len(kinds(effects, 'CANCEL')) == 1  # stale accepted goal never abandoned
    assert not kinds(effects, 'SEND')


def test_completed_mission_never_reactivated_by_late_feedback(setup):
    _, _, core = setup
    running(core)
    core.cancel(1.1)
    success(core, 1.2)
    original = replace(core.status)
    effects = event(core, 'FEEDBACK', 2., stamp=2., phase='executing', progress=.5)
    assert core.status == original and not kinds(effects, 'TERMINAL')


def test_restart_constructs_idle_and_does_not_resume(setup):
    route, config, old = setup
    running(old)
    restarted = MissionCore(route, config)
    effects = restarted.tick(2., Readiness(reason='server still busy'), True)
    assert restarted.status.state == S.IDLE and not kinds(effects, 'SEND')
    assert not restarted.status.task_id


@pytest.mark.parametrize('operation', ['send', 'result', 'cancel'])
def test_async_transport_errors_do_not_release_unknown_execution(setup, operation):
    _, _, core = setup
    running(core)
    core.cancel(1.02) if operation == 'cancel' else None
    event(core, 'ERROR', 1.1, operation=operation, message='transport failed')
    assert core.status.state in (S.CANCELING, S.FAULT_BLOCKED)
    core.tick(100.)
    assert core.status.blocked
    assert not core.start(core.config['route_id'], 'task-2', 101.)


def test_clock_jump_requests_cleanup(setup):
    _, _, core = setup
    running(core)
    effects = core.tick(.5)
    assert core.status.state == S.CANCELING and core.status.error_code == 'CLOCK_JUMP'
    assert len(kinds(effects, 'CANCEL')) == 1


@pytest.mark.parametrize('mutation', [lambda c: c.update(route_id=''),
                                     lambda c: c.update(sequence=['missing']),
                                     lambda c: c.update(sequence=['AB','CD']),
                                     lambda c: c['timeouts'].update(cancel_terminal=0),
                                     lambda c: c['timeouts'].update(acceptance=float('nan')),
                                     lambda c: c['timeouts'].update(unknown=1),
                                     lambda c: c['readiness'].update(require_start_node='yes'),
                                     lambda c: c.update(per_segment={'missing':{}})])
def test_invalid_mission_config_rejected_before_dispatch(setup, mutation):
    route, config, _ = setup
    config = deepcopy(config)
    mutation(config)
    with pytest.raises(NavError):
        validate_mission(config, route)


def test_transition_log_has_time_ids_event_reason(setup):
    _, _, core = setup
    effects = core.tick(0.)
    assert effects == []
    dispatch(core)
    effects = event(core, 'ACCEPTED', accepted=True)
    transition = kinds(effects, 'TRANSITION')[0]
    assert transition.task_id == 'task-1' and transition.segment_id == 'AB'
    assert transition.execution_id and set(transition.data) == {'old','new','event','reason','code','time'}


def test_mission_route_file_reference_must_match_loaded_route(tmp_path):
    from stair_nav.core.mission import load_mission
    import yaml
    route_path = Path(__file__).resolve().parents[1]/'config'/'demo_route.yaml'
    route = load_route(route_path)
    mission = tmp_path/'mission.yaml'
    mission.write_text(yaml.safe_dump(dict(route_id='test', sequence=['AB', 'BC', 'CD'],
                                         route_file=str(route_path))))
    assert load_mission(mission, route)['route_file'] == str(route_path)
    mission.write_text(yaml.safe_dump(dict(route_id='test', sequence=['AB', 'BC', 'CD'],
                                         route_file='another_route.yaml')))
    with pytest.raises(NavError, match='differs from loaded route'):
        load_mission(mission, route)
    mission.write_text(yaml.safe_dump(dict(route_id='test', route_file='')))
    with pytest.raises(NavError, match='nonempty path'):
        load_mission(mission, route)


def test_duplicate_feedback_does_not_extend_freshness_deadline(setup):
    route, config, _ = setup
    config['timeouts']['feedback'] = .5
    core = MissionCore(route, config)
    running(core)
    event(core, 'FEEDBACK', 1.1, stamp=1.1, segment_state='EXECUTING', progress=.1)
    event(core, 'FEEDBACK', 1.4, stamp=1.1, segment_state='EXECUTING', progress=.1)
    effects = core.tick(1.61)
    assert core.status.error_code == 'FEEDBACK_TIMEOUT'
    assert len(kinds(effects, 'CANCEL')) == 1


def test_clock_reset_does_not_defer_cleanup_for_old_clock_epoch(setup):
    _, _, core = setup
    running(core, now=1000.)
    core.tick(0.)
    assert core.status.state == S.CANCELING
    event(core, 'CANCEL_RESPONSE', .1, accepted=True)
    core.tick(10.)
    assert core.status.state == S.FAULT_BLOCKED


def test_transport_result_before_acceptance_response_does_not_duplicate_send(setup):
    _, _, core = setup
    eid = dispatch(core)
    core.post('RESULT', 'task-1', eid, status=4, success=True,
              navigation_stopped=True, hold_confirmed=True)
    core.post('ACCEPTED', 'task-1', eid, accepted=True)
    effects = core.tick(1.1, ready(1.1), True)
    assert core.status.completed_segments == 1
    assert not kinds(effects, 'SEND')
    effects = core.tick(1.2, ready(1.2, eid, 'BC'), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['BC']


def test_dispatch_requires_release_even_if_ready_is_claimed(setup):
    _, _, core = setup
    assert core.start(core.config['route_id'], 'task', 1.)
    facts = replace(ready(1.), released=False)
    effects = core.tick(1., facts, True)
    assert core.status.state == S.WAITING_READY
    assert not kinds(effects, 'SEND')
    effects = core.tick(1.1, ready(1.1), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['AB']
    assert not kinds(core.tick(1.2, ready(1.2), True), 'SEND')


@pytest.mark.parametrize('target_segment_id', ['', 'BC', 'CD'])
def test_dispatch_rejects_readiness_for_another_or_unspecified_segment(setup, target_segment_id):
    _, _, core = setup
    assert core.start(core.config['route_id'], 'task', 1.)
    effects = core.tick(1., ready(1., target_segment_id=target_segment_id), True)
    assert core.status.state == S.WAITING_READY
    assert not kinds(effects, 'SEND')
    assert [e.segment_id for e in kinds(core.tick(1.1, ready(1.1), True), 'SEND')] == ['AB']


def test_next_segment_waits_for_its_own_readiness_before_dispatch(setup):
    _, _, core = setup
    eid = running(core)
    old_facts = ready(1.1, eid, 'AB')
    core.post('RESULT', core.status.task_id, eid, status=4, success=True,
              navigation_stopped=True, hold_confirmed=True)
    effects = core.tick(1.1, old_facts, True)
    assert core.status.state == S.WAITING_READY
    assert core.status.segment_id == 'BC' and core.status.execution_id == ''
    assert core.status.completed_segments == 1
    assert not kinds(effects, 'SEND')
    # Reusing old facts on a later update still cannot authorize BC.
    assert not kinds(core.tick(1.2, old_facts, True), 'SEND')
    effects = core.tick(1.3, ready(1.3, eid, 'BC'), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['BC']
    assert not kinds(core.tick(1.4, ready(1.4, eid, 'BC'), True), 'SEND')


def test_next_segment_missing_prerequisites_times_out_without_dispatch(setup):
    _, _, core = setup
    eid = running(core)
    core.post('RESULT', core.status.task_id, eid, status=4, success=True,
              navigation_stopped=True, hold_confirmed=True)
    core.tick(1.1, ready(1.1, eid, 'AB'), True)
    facts = Readiness(released=True, target_segment_id='BC',
                      reason='required motion capabilities unavailable')
    effects = core.tick(1.1 + core.config['timeouts']['ready'], facts, True)
    assert core.status.state == S.FAILED and core.status.error_code == 'READY_TIMEOUT'
    assert core.status.completed_segments == 1
    assert not kinds(effects, 'SEND')


@pytest.fixture
def approach_setup(setup):
    route, config, _ = setup
    # This suite exercises mission arbitration; the navigation tests own maps
    # and paths. Reuse a platform segment's shape for the synthetic approach.
    route['segments']['APPROACH_A'] = {
        **deepcopy(route['segments']['CD']), 'start': 'A', 'end': 'A',
        'platform_id': 'lower_platform',
    }
    config['approach_segment'] = 'APPROACH_A'
    counter = iter(f'approach-exec-{n}' for n in range(100))
    return route, config, MissionCore(route, config, id_factory=lambda: next(counter))


def approach_dispatch(core, now=1.):
    assert core.start(core.config['route_id'], 'approach-task', now)
    effects = core.tick(now, ready(now, target_segment_id='APPROACH_A', at_start=False), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['APPROACH_A']
    assert core.status.segment_index == 0 and core.status.segment_count == 3
    return core.status.execution_id


def approach_running(core, now=1.):
    execution_id = approach_dispatch(core, now)
    event(core, 'ACCEPTED', now+.01, accepted=True)
    assert core.status.state == S.APPROACHING_START
    return execution_id


def test_approach_skipped_at_start_then_main_target_gets_its_own_readiness(approach_setup):
    _, _, core = approach_setup
    assert core.start(core.config['route_id'], 'task', 1.)
    assert core.status.segment_id == 'APPROACH_A' and core.status.segment_index == 0
    old_facts = ready(1., target_segment_id='APPROACH_A', at_start=True)
    effects = core.tick(1., old_facts, True)
    assert core.status.segment_id == 'AB' and core.status.segment_index == 1
    assert core.status.state == S.WAITING_READY and core.status.completed_segments == 0
    assert not kinds(effects, 'SEND')
    assert not kinds(core.tick(1.1, old_facts, True), 'SEND')
    effects = core.tick(1.2, ready(1.2, target_segment_id='AB', at_start=True), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['AB']


def test_approach_unknown_localization_cannot_dispatch_and_times_out(approach_setup):
    _, config, core = approach_setup
    assert core.start(core.config['route_id'], 'task', 1.)
    facts = ready(1., target_segment_id='APPROACH_A', at_start=None)
    assert not kinds(core.tick(1., facts, True), 'SEND')
    assert core.status.state == S.WAITING_READY and core.status.segment_index == 0
    effects = core.tick(1.+config['timeouts']['ready'], facts, True)
    assert core.status.state == S.FAILED and core.status.error_code == 'READY_TIMEOUT'
    assert not kinds(effects, 'SEND')


def test_approach_waits_until_localization_can_choose_to_navigate(approach_setup):
    _, _, core = approach_setup
    assert core.start(core.config['route_id'], 'task', 1.)
    assert not kinds(core.tick(1., ready(1., target_segment_id='APPROACH_A'), True), 'SEND')
    facts = ready(1.1, target_segment_id='APPROACH_A', at_start=False)
    assert [e.segment_id for e in kinds(core.tick(1.1, facts, True), 'SEND')] == ['APPROACH_A']


@pytest.mark.parametrize('initial_at_start', [False, True])
def test_start_target_is_reconsidered_until_first_dispatch(approach_setup, initial_at_start):
    _, _, core = approach_setup
    core.start(core.config['route_id'], 'task', 1.)
    initial = replace(ready(1., target_segment_id='APPROACH_A', at_start=initial_at_start),
                      ready=False, released=False)
    assert not kinds(core.tick(1., initial, True), 'SEND')
    old_target = core.status.segment_id
    next_at_start = not initial_at_start
    # Even otherwise-ready facts for the old target cannot authorize the new
    # one after the latest localization changes the start decision.
    effects = core.tick(1.1, ready(1.1, target_segment_id=old_target,
                                  at_start=next_at_start), True)
    expected = 'AB' if next_at_start else 'APPROACH_A'
    assert core.status.segment_id == expected
    assert core.status.segment_index == (1 if next_at_start else 0)
    assert not kinds(effects, 'SEND')
    facts = ready(1.2, target_segment_id=expected, at_start=next_at_start)
    assert [e.segment_id for e in kinds(core.tick(1.2, facts, True), 'SEND')] == [expected]
    execution = core.status.execution_id
    # The decision is fixed once this execution has been dispatched.
    effects = core.tick(1.3, ready(1.3, target_segment_id=expected,
                                 at_start=initial_at_start), True)
    assert core.status.segment_id == expected and core.status.execution_id == execution
    assert not kinds(effects, 'SEND')


def test_start_target_waits_for_localization_again_after_initial_skip(approach_setup):
    _, _, core = approach_setup
    core.start(core.config['route_id'], 'task', 1.)
    core.tick(1., ready(1., target_segment_id='APPROACH_A', at_start=True), True)
    assert core.status.segment_id == 'AB'
    # A claimed ready flag cannot dispatch after localization becomes unknown.
    assert not kinds(core.tick(1.1, ready(1.1, target_segment_id='AB', at_start=None), True), 'SEND')
    effects = core.tick(1.2, ready(1.2, target_segment_id='AB', at_start=False), True)
    assert core.status.segment_id == 'APPROACH_A' and not kinds(effects, 'SEND')
    effects = core.tick(1.3, ready(1.3, target_segment_id='APPROACH_A', at_start=False), True)
    assert [e.segment_id for e in kinds(effects, 'SEND')] == ['APPROACH_A']


@pytest.mark.parametrize('server_ready, code', [(True, 'READY_TIMEOUT'), (False, 'SERVICE_TIMEOUT')])
def test_start_pose_oscillation_does_not_restart_wait_deadline(approach_setup, server_ready, code):
    _, config, core = approach_setup
    core.start(core.config['route_id'], 'task', 1.)
    boundary = 1. + config['timeouts']['ready' if server_ready else 'service']
    for index, when in enumerate((1., 1.5, 2., boundary-.1, boundary)):
        facts = replace(ready(when, target_segment_id=core.status.segment_id,
                              at_start=index % 2 == 0), ready=False)
        effects = core.tick(when, facts, server_ready)
        assert not kinds(effects, 'SEND')
        assert core._waiting_at == 1.
    assert core.status.state == S.FAILED and core.status.error_code == code


def test_approach_success_does_not_count_and_full_route_still_requires_three_segments(approach_setup):
    _, _, core = approach_setup
    approach_running(core)
    effects = event(core, 'RESULT', 1.2, status=4, success=True,
                    navigation_stopped=True, hold_confirmed=True)
    assert core.status.state == S.WAITING_READY and core.status.segment_id == 'AB'
    assert core.status.segment_index == 1 and core.status.completed_segments == 0
    assert core.status.segment_count == 3 and not kinds(effects, 'SEND')
    assert not kinds(core.tick(1.21, ready(1.21, target_segment_id='APPROACH_A'), True), 'SEND')
    effects = core.tick(1.3, ready(1.3, target_segment_id='AB', at_start=True), True)
    sent = [e.segment_id for e in kinds(effects, 'SEND')]
    for index in range(3):
        event(core, 'ACCEPTED', 1.4+index, accepted=True)
        effects = success(core, 1.5+index)
        sent.extend(e.segment_id for e in kinds(effects, 'SEND'))
    assert sent == ['AB', 'BC', 'CD']
    assert core.status.state == S.SUCCEEDED and core.status.completed_segments == 3


@pytest.mark.parametrize('accepted_before_cancel', [False, True])
def test_approach_cancel_handles_late_acceptance_and_never_starts_main(approach_setup, accepted_before_cancel):
    _, _, core = approach_setup
    execution_id = approach_dispatch(core)
    if accepted_before_cancel:
        event(core, 'ACCEPTED', 1.01, accepted=True)
    assert core.cancel(1.1)
    effects = core.tick(1.1)
    if not accepted_before_cancel:
        assert not kinds(effects, 'CANCEL')
        effects = event(core, 'ACCEPTED', 1.11, accepted=True)
    assert len(kinds(effects, 'CANCEL')) == 1
    assert core.status.state == S.CANCELING
    effects = event(core, 'RESULT', 1.2, status=5, success=False,
                    navigation_stopped=True, hold_confirmed=True)
    assert core.status.state == S.CANCELED and core.status.completed_segments == 0
    assert core.status.execution_id == execution_id and core.status.segment_index == 0
    assert not kinds(effects, 'SEND')


@pytest.mark.parametrize('status,success_flag,stopped,hold,code', [
    (6, False, True, True, 'APPROACH_PLAN_FAILED'),
    (4, True, True, False, ''),
    (4, True, False, True, ''),
    (6, True, True, True, ''),
])
def test_approach_result_must_confirm_success_and_release(approach_setup, status, success_flag, stopped, hold, code):
    _, _, core = approach_setup
    approach_running(core)
    effects = event(core, 'RESULT', 1.2, status=status, success=success_flag,
                    navigation_stopped=stopped, hold_confirmed=hold,
                    error_code=code)
    assert core.status.state in (S.FAILED, S.CANCELING)
    assert core.status.segment_id == 'APPROACH_A' and core.status.completed_segments == 0
    assert not kinds(effects, 'SEND')


@pytest.mark.parametrize('timeout,code', [('execution', 'EXECUTION_TIMEOUT'),
                                         ('confirmation', 'CONFIRMATION_TIMEOUT'),
                                         ('feedback', 'FEEDBACK_TIMEOUT')])
def test_approach_uses_execution_and_feedback_deadlines(approach_setup, timeout, code):
    route, config, _ = approach_setup
    config['per_segment'] = {'APPROACH_A': {'timeouts': {timeout: .2}}}
    core = MissionCore(route, config)
    approach_running(core)
    if timeout == 'confirmation':
        event(core, 'FEEDBACK', 1.02, stamp=1.02, segment_state='ARRIVAL_CONFIRM',
              phase='geometric_candidate', progress=1.)
    effects = core.tick(1.25)
    assert core.status.state == S.CANCELING and core.status.error_code == code
    assert len(kinds(effects, 'CANCEL')) == 1 and not kinds(effects, 'SEND')


def test_retired_approach_feedback_cannot_mutate_main_segment(approach_setup):
    _, _, core = approach_setup
    execution_id = approach_running(core)
    event(core, 'RESULT', 1.2, status=4, success=True,
          navigation_stopped=True, hold_confirmed=True)
    core.tick(1.3, ready(1.3, target_segment_id='AB', at_start=True), True)
    event(core, 'ACCEPTED', 1.4, accepted=True)
    original = replace(core.status)
    core.post('FEEDBACK', core.status.task_id, execution_id,
              stamp=1.5, segment_state='ARRIVAL_CONFIRM', phase='geometric_candidate', progress=1.)
    effects = core.tick(1.5, ready(1.5, target_segment_id='AB'), True)
    assert core.status == original and not kinds(effects, 'SEGMENT_TRANSITION')


@pytest.mark.parametrize('mutation', [
    lambda r, c: c.update(approach_segment='missing'),
    lambda r, c: c.update(approach_segment=''),
    lambda r, c: c.update(approach_segment=None),
    lambda r, c: c.update(approach_segment='AB'),
    lambda r, c: r['segments']['APPROACH_A'].update(kind='stair'),
    lambda r, c: r['segments']['APPROACH_A'].update(backend='preset_path'),
    lambda r, c: r['segments']['APPROACH_A'].update(end='B'),
])
def test_invalid_approach_config_is_rejected(approach_setup, mutation):
    route, config, _ = approach_setup
    mutation(route, config)
    with pytest.raises(NavError, match='approach_segment'):
        validate_mission(config, route)


def test_each_transition_keeps_its_own_snapshot_within_one_tick(setup):
    _, _, core = setup
    assert core.start(core.config['route_id'], 'task', 1.)
    effects = core.tick(1., ready(1.), True)
    transitions = kinds(effects, 'TRANSITION')
    assert [e.snapshot.state for e in transitions] == [S.VALIDATING, S.WAITING_READY, S.DISPATCHING]
    assert [e.snapshot.execution_id for e in transitions[:2]] == ['', '']
    assert transitions[2].snapshot.execution_id == core.status.execution_id
    assert all(e.snapshot is not core.status for e in transitions)
    event(core, 'ACCEPTED', 1.1, accepted=True)
    assert [e.snapshot.state for e in transitions] == [S.VALIDATING, S.WAITING_READY, S.DISPATCHING]


def test_segment_changes_capture_each_valid_feedback_and_ignore_duplicates(approach_setup):
    _, _, core = approach_setup
    approach_running(core)
    core.post('FEEDBACK', core.status.task_id, core.status.execution_id,
              stamp=1.1, segment_state='EXECUTING', phase='platform', progress=.2)
    core.post('FEEDBACK', core.status.task_id, core.status.execution_id,
              stamp=1.2, segment_state='ARRIVAL_CONFIRM', phase='geometric_candidate', progress=1.)
    effects = core.tick(1.2, ready(1.2, target_segment_id='APPROACH_A'), True)
    changes = kinds(effects, 'SEGMENT_TRANSITION')
    assert [e.snapshot.segment_state for e in changes] == ['EXECUTING', 'ARRIVAL_CONFIRM']
    assert [e.snapshot.segment_progress for e in changes] == [.2, 1.]
    assert all(e.snapshot.state == S.APPROACHING_START for e in changes)
    assert all(e.data['event'] == 'SEGMENT_FEEDBACK' for e in changes)
    effects = event(core, 'FEEDBACK', 1.3, stamp=1.2, segment_state='EXECUTING', phase='late')
    assert not kinds(effects, 'SEGMENT_TRANSITION')
    assert core.status.segment_state == 'ARRIVAL_CONFIRM'
    effects = event(core, 'FEEDBACK', 1.4, stamp=1.4, segment_state='ARRIVAL_CONFIRM',
                    phase='confirmed_stable', progress=1.)
    assert len(kinds(effects, 'SEGMENT_TRANSITION')) == 1
    assert changes[0].snapshot.phase == 'platform'


def test_feedback_before_acceptance_preserves_checking_and_planning_order(approach_setup):
    _, _, core = approach_setup
    approach_dispatch(core)
    for stamp, state, phase in ((1.01, 'CHECKING', 'checking'),
                                (1.02, 'PLANNING', 'planning')):
        core.post('FEEDBACK', core.status.task_id, core.status.execution_id,
                  stamp=stamp, segment_state=state, phase=phase)
    core.post('ACCEPTED', core.status.task_id, core.status.execution_id, accepted=True)
    effects = core.tick(1.1, ready(1.1, target_segment_id='APPROACH_A'), True)
    changes = kinds(effects, 'SEGMENT_TRANSITION')
    assert [e.data['new'] for e in changes] == ['CHECKING', 'PLANNING']
    assert [e.snapshot.state for e in changes] == [S.DISPATCHING, S.DISPATCHING]
    assert changes[0].snapshot.phase == 'checking'
    assert core.status.state == S.APPROACHING_START
    assert core.status.segment_state == 'PLANNING'


def test_early_feedback_neither_accepts_goal_nor_extends_acceptance_deadline(approach_setup):
    _, config, core = approach_setup
    approach_dispatch(core)
    boundary = 1.+config['timeouts']['acceptance']
    effects = event(core, 'FEEDBACK', boundary-.1, stamp=boundary-.1,
                    segment_state='CHECKING', phase='checking')
    assert len(kinds(effects, 'SEGMENT_TRANSITION')) == 1
    assert core.status.state == S.DISPATCHING
    effects = core.tick(boundary)
    assert core.status.state == S.CANCELING and core.status.error_code == 'GOAL_ACCEPT_TIMEOUT'
    assert not kinds(effects, 'SEND')


def test_canceled_feedback_is_reported_without_completing_task_before_result(setup):
    _, _, core = setup
    running(core)
    core.cancel(1.1)
    effects = event(core, 'FEEDBACK', 1.2, stamp=1.2,
                    segment_state='CANCELED', phase='platform', progress=.4)
    changes = kinds(effects, 'SEGMENT_TRANSITION')
    assert [e.data['new'] for e in changes] == ['CANCELED']
    assert changes[0].snapshot.state == S.CANCELING
    assert core.status.state == S.CANCELING and not core.status.downstream_terminal
    assert core.status.completed_segments == 0 and not kinds(effects, 'TERMINAL')
    effects = event(core, 'RESULT', 1.3, status=5, success=False,
                    navigation_stopped=True, hold_confirmed=True)
    assert core.status.state == S.CANCELED and len(kinds(effects, 'TERMINAL')) == 1
    assert not kinds(effects, 'SEGMENT_TRANSITION')  # feedback already reported it


@pytest.mark.parametrize('status,success_flag,segment_state,task_state', [
    (4, True, 'SUCCEEDED', S.WAITING_READY),
    (6, False, 'FAILED', S.FAILED),
])
def test_result_reports_segment_terminal_before_advancing_and_late_feedback_is_ignored(
        setup, status, success_flag, segment_state, task_state):
    _, _, core = setup
    execution_id = running(core)
    event(core, 'FEEDBACK', 1.1, stamp=1.1,
          segment_state='EXECUTING', phase='stair', progress=.5)
    effects = event(core, 'RESULT', 1.2, status=status, success=success_flag,
                    navigation_stopped=True, hold_confirmed=True, error_code='')
    changes = kinds(effects, 'SEGMENT_TRANSITION')
    assert [e.data['new'] for e in changes] == [segment_state]
    assert changes[0].data['event'] == 'SEGMENT_RESULT'
    assert changes[0].snapshot.segment_id == 'AB'
    assert changes[0].snapshot.execution_id == execution_id
    assert changes[0].snapshot.segment_state == segment_state
    assert changes[0].snapshot.downstream_terminal
    transition_index = effects.index(changes[0])
    assert all(e.kind != 'TRANSITION' for e in effects[:transition_index])
    assert core.status.state == task_state
    original = replace(core.status)
    core.post('FEEDBACK', core.status.task_id, execution_id, stamp=1.3,
              segment_state='EXECUTING', phase='late', progress=.2)
    effects = core.tick(1.3)
    assert core.status == original and not kinds(effects, 'SEGMENT_TRANSITION')


@pytest.mark.parametrize('stage,error', [
    ('response', 'CANCEL_RESPONSE_TIMEOUT'),
    ('terminal', 'CANCEL_TERMINAL_TIMEOUT'),
    ('cleanup', 'CLEANUP_TIMEOUT'),
])
def test_cancel_feedback_does_not_extend_deadlines_or_replace_failure_cause(setup, stage, error):
    route, config, _ = setup
    config['timeouts'].update(execution=.05, cancel_response=.4, cancel_terminal=.4, cleanup=.4)
    core = MissionCore(route, config)
    running(core)
    core.tick(1.1)  # execution deadline initiates cancellation
    assert core.status.state == S.CANCELING and core.status.error_code == 'EXECUTION_TIMEOUT'
    last_feedback = core._last_feedback
    if stage == 'terminal':
        event(core, 'CANCEL_RESPONSE', 1.2, accepted=True)
    elif stage == 'cleanup':
        core.post('RESULT', core.status.task_id, core.status.execution_id, status=5, success=False,
                  navigation_stopped=True, hold_confirmed=False)
        core.tick(1.2)
    for now in (1.3, 1.4):
        core.post('FEEDBACK', core.status.task_id, core.status.execution_id,
                  stamp=now, segment_state='ARRIVAL_CONFIRM', phase='geometric_candidate',
                  progress=1., error_code='DOWNSTREAM_DETAIL', error_message='display only')
        effects = core.tick(now)
        assert core.status.state == S.CANCELING and core.status.completed_segments == 0
        assert core.status.error_code == 'EXECUTION_TIMEOUT'
        assert core._last_feedback == last_feedback and core._confirmation_at is None
        assert not kinds(effects, 'TERMINAL')
        if stage == 'cleanup':
            assert core.status.segment_state == 'CANCELED'
            assert not kinds(effects, 'SEGMENT_TRANSITION')
    deadline = (1.1 if stage == 'response' else 1.2) + .4
    core.tick(deadline+.001)
    assert core.status.state == S.FAULT_BLOCKED and core.status.error_code == error
