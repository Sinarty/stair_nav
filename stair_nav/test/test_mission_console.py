"""Console summaries must preserve lifecycle semantics while throttling progress."""
from types import SimpleNamespace

import pytest

from stair_nav.mission_console import MissionReporter


def feedback(**overrides):
    values = dict(task_id='task-1', route_id='inspection_demo',
                  execution_id='exec-1', segment_id='AB', segment_index=1,
                  segment_count=3, completed_segments=0, state='RUNNING',
                  segment_state='EXECUTING', phase='platform',
                  segment_progress=.1, error_code='', error_message='', blocked=False)
    values.update(overrides)
    return SimpleNamespace(**values)


def result(**overrides):
    values = dict(task_id='task-1', route_id='inspection_demo', state='SUCCEEDED',
                  success=True, error_code='', error_message='',
                  navigation_stopped=True, hold_confirmed=True)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_progress_throttles_without_suppressing_changes():
    reporter = MissionReporter()
    assert any('10.0%' in line for line in reporter.feedback(feedback(), 10.))
    assert reporter.feedback(feedback(segment_progress=.2), 10.2) == []
    lines = reporter.feedback(feedback(segment_progress=.3, phase='stair'), 10.3)
    assert any('楼梯' in line for line in lines)
    assert not any('本段路径进度' in line for line in lines)
    assert any('30.0%' in line for line in reporter.feedback(
        feedback(segment_progress=.3, phase='stair'), 11.))
    assert reporter.feedback(feedback(segment_progress=.3, phase='stair'), 12.) == []


@pytest.mark.parametrize('segment_id, execution_id', [('BC', 'exec-2'), ('AB', 'retry-2')])
def test_new_execution_shows_progress_immediately(segment_id, execution_id):
    reporter = MissionReporter()
    reporter.feedback(feedback(), 1.)
    lines = reporter.feedback(feedback(segment_id=segment_id, execution_id=execution_id), 1.1)
    assert any(execution_id in line for line in lines)
    assert any('10.0%' in line for line in lines)


def test_accepted_segment_does_not_claim_motion_started():
    lines = MissionReporter().feedback(feedback(segment_state='PLANNING', phase='planning'), 1.)
    text = '\n'.join(lines)
    assert '导航段已受理' in text
    assert '规划本段路径' in text
    assert '开始运动' not in text
    assert '路径跟踪' not in text
    assert '本段路径进度' not in text


def test_approach_does_not_display_zero_as_main_segment_ordinal():
    lines = MissionReporter().feedback(feedback(
        segment_id='APPROACH_A', segment_index=0, state='APPROACHING_START'), 1.)
    text = '\n'.join(lines)
    assert '接近起点段，不计入主路段' in text
    assert '第 0/3' not in text


def test_error_updates_are_immediate_and_keep_original_detail():
    reporter = MissionReporter(progress_interval=10.)
    reporter.feedback(feedback(), 1.)
    lines = reporter.feedback(feedback(error_code='POSE_STALE',
                                       error_message='localization stamp expired'), 1.1)
    text = '\n'.join(lines)
    assert 'POSE_STALE' in text and '定位数据超时' in text
    assert 'localization stamp expired' in text
    lines = reporter.feedback(feedback(
        error_code='FUTURE_VENDOR_FAULT', error_message='vendor detail'), 1.2)
    text = '\n'.join(lines)
    assert 'FUTURE_VENDOR_FAULT' in text and 'vendor detail' in text


def test_unknown_state_and_phase_are_retained():
    text = '\n'.join(MissionReporter().feedback(
        feedback(state='FUTURE_STATE', segment_state='NEW_SEGMENT', phase='new_phase'), 1.))
    assert 'FUTURE_STATE' in text and 'NEW_SEGMENT' in text and 'new_phase' in text


def test_completion_and_error_are_not_hidden_by_progress_interval():
    reporter = MissionReporter(progress_interval=10.)
    reporter.feedback(feedback(), 1.)
    lines = reporter.feedback(feedback(completed_segments=1, segment_state='SUCCEEDED'), 1.1)
    assert any('已完成 1/3' in line for line in lines)
    assert any('本段执行成功' in line for line in lines)


def test_blocked_result_does_not_claim_physical_stop():
    lines = MissionReporter().result(result(
        state='FAULT_BLOCKED', success=False, error_code='CLEANUP_TIMEOUT',
        error_message='release unknown', navigation_stopped=False,
        hold_confirmed=False), 6)
    text = '\n'.join(lines)
    assert '任务阻塞' in text and 'release unknown' in text
    assert '导航指令：撤销未确认' in text and '运控保持：未确认' in text
    assert '已停稳' not in text and '巡检成功' not in text


@pytest.mark.parametrize('protocol', [5, 6])
def test_business_cancellation_can_have_canceled_or_aborted_protocol(protocol):
    text = '\n'.join(MissionReporter().result(
        result(state='CANCELED', success=False, error_code='CANCELED'), protocol))
    assert '巡检已取消' in text
    assert '结果异常' not in text


def test_success_requires_consistent_action_result():
    reporter = MissionReporter()
    reporter.feedback(feedback(completed_segments=3, state='SUCCEEDED',
                               segment_state='SUCCEEDED', segment_progress=1.), 1.)
    assert any('巡检成功，已完成 3/3' in line for line in reporter.result(result(), 4))
    assert any('结果异常' in line for line in reporter.result(result(), 6))
    assert any('结果异常' in line for line in reporter.result(result(success=False), 4))


def test_success_result_can_arrive_before_last_completion_feedback():
    reporter = MissionReporter()
    reporter.feedback(feedback(segment_id='CD', segment_index=3, completed_segments=2), 1.)
    text = '\n'.join(reporter.result(result(), 4))
    assert '巡检成功，已完成 3/3' in text
    assert '2/3' not in text
    assert 'SUCCEEDED' in text


@pytest.mark.parametrize('revoked, held', [(False, True), (True, False), (False, False)])
def test_success_without_stop_and_hold_contract_is_reported_as_inconsistent(revoked, held):
    text = '\n'.join(MissionReporter().result(result(
        navigation_stopped=revoked, hold_confirmed=held), 4))
    assert '结果异常' in text and 'SUCCEEDED' in text
    assert '[任务完成]' not in text


@pytest.mark.parametrize('successful, protocol', [(True, 5), (True, 6), (False, 4), (False, 0)])
def test_inconsistent_cancellation_result_is_not_reported_as_normal_cancellation(
        successful, protocol):
    text = '\n'.join(MissionReporter().result(
        result(state='CANCELED', success=successful), protocol))
    assert '结果异常' in text and 'CANCELED' in text
    assert '[任务取消]' not in text


@pytest.mark.parametrize('state', ['SUCCEEDED', 'FAILED', 'CANCELED', 'FAULT_BLOCKED'])
def test_final_business_state_is_visible_without_terminal_feedback(state):
    successful = state == 'SUCCEEDED'
    protocol = 4 if successful else 6
    text = '\n'.join(MissionReporter().result(result(state=state, success=successful), protocol))
    assert state in text


@pytest.mark.parametrize('interval', [0., -1., float('nan'), float('inf')])
def test_invalid_progress_interval_is_rejected(interval):
    with pytest.raises(ValueError):
        MissionReporter(interval)
