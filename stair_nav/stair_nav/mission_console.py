"""Human-readable mission feedback, independent of ROS and terminal I/O."""
import math


MISSION_STATES = {
    'IDLE': '空闲',
    'VALIDATING': '检查任务',
    'WAITING_READY': '等待定位、运控和控制释放就绪',
    'DISPATCHING': '正在派发导航段',
    'RUNNING': '导航段已受理，正在执行段流程',
    'APPROACHING_START': '接近起点段已受理',
    'CANCELING': '正在取消，等待执行结束及保持、释放确认',
    'SUCCEEDED': '巡检任务成功',
    'FAILED': '巡检任务失败',
    'CANCELED': '巡检任务已取消',
    'FAULT_BLOCKED': '故障阻塞，暂不接受新任务',
}
SEGMENT_STATES = {
    'IDLE': '空闲',
    'CHECKING': '检查本段输入',
    'PLANNING': '规划本段路径',
    'WAIT_READY': '等待目标运动模式和运控就绪',
    'EXECUTING': '路径跟踪',
    'ARRIVAL_CONFIRM': '几何到达，等待保持及配置要求的确认',
    'SUCCEEDED': '本段执行成功',
    'FAILED': '本段执行失败',
    'CANCELED': '本段已取消',
}
PHASES = {
    'checking': '输入检查',
    'planning': '路径规划',
    'waiting_ready': '等待运控',
    'platform': '平台',
    'entry': '进梯',
    'stair': '楼梯',
    'exit': '出梯',
    'geometric_candidate': '到达确认',
}
ERRORS = {
    'BUSY': '已有活动任务或处于故障阻塞',
    'CONFIG_INVALID': '路线配置无效',
    'MISSION_CONFIG_INVALID': '任务配置无效',
    'POSE_MISSING': '缺少可用定位',
    'INVALID_POSE': '定位数据无效',
    'POSE_STALE': '定位数据超时',
    'MOTION_STALE': '当前执行的运控反馈超时',
    'MOTION_FAULT': '运控报告故障',
    'MOTION_NOT_READY': '运控就绪条件丢失',
    'CAPABILITY_MISSING': '缺少本段要求的运动能力',
    'MODE_UNSUPPORTED': '不支持本段要求的运动模式',
    'MODE_MISMATCH': '实际运动模式与本段要求不一致',
    'SERVICE_TIMEOUT': '等待导航服务超时',
    'READY_TIMEOUT': '等待定位、运控或控制释放就绪超时',
    'PLANNING_TIMEOUT': '路径规划超时',
    'EXECUTION_TIMEOUT': '导航段执行超时',
    'FEEDBACK_TIMEOUT': '导航段反馈超时',
    'ARRIVAL_CONFIRM_TIMEOUT': '本段到达确认超时',
    'CONFIRMATION_TIMEOUT': '任务等待到达确认超时',
    'GOAL_ACCEPT_TIMEOUT': '导航段接收应答超时',
    'GOAL_REJECTED': '导航段请求被拒绝',
    'ACTION_TRANSPORT': '任务与导航段之间的通信异常',
    'CANCEL_REJECTED': '下游拒绝取消，执行状态待核实',
    'CANCEL_TRANSPORT': '取消请求通信异常',
    'CANCEL_RESPONSE_TIMEOUT': '等待取消应答超时',
    'CANCEL_TERMINAL_TIMEOUT': '取消后等待执行终态超时',
    'CLEANUP_TIMEOUT': '等待保持、稳定或控制释放超时',
    'ACCEPTANCE_UNRESOLVED': '导航段是否已受理仍不明确',
    'CHANNEL_BLOCKED': '通道被阻塞',
    'PATH_DEVIATION': '横向路径偏差过大',
    'HEIGHT_DEVIATION': '支撑高度偏差过大',
    'TRACKING_UNREACHABLE': '当前位姿无法按允许方式跟踪路径',
    'FINAL_HEADING_UNREACHABLE': '无法完成目标朝向对齐',
    'TRACKING_COLLISION': '跟踪指令超出已验证的可通行范围',
    'START_MOVED': '规划期间机器人离开允许的起点范围',
    'CLOCK_JUMP': '时钟异常或发生回跳',
    'MAP_IO': '地图或路径文件读取失败',
    'INVALID_GRID': '栅格地图数据无效',
    'INVALID_PATH': '预设路径数据无效',
    'NO_PATH': '未找到可通行路径',
    'JOIN_UNVERIFIED': '预设路径的起点连接通道尚未确认',
    'JOIN_OUT_OF_RANGE': '定位超出预设路径入口范围',
    'RESULT_CONTRACT': '导航段结果缺少必要的停止或保持确认',
    'INTERNAL_ERROR': '导航段内部异常',
    'MISSION_WRAPPER_ERROR': '任务接口内部异常',
    'CANCELED': '任务取消',
}
ACTION_STATES = {4: '成功', 5: '取消', 6: '中止'}


def _text(value):
    return str(getattr(value, 'value', value))


def _label(value, labels):
    value = _text(value)
    return f'{labels[value]}（{value}）' if value in labels else value or '未知'


def _reason(code, message):
    """Retain the original code and detail even if no translation is known."""
    if not code and not message:
        return []
    description = ERRORS.get(code, '未收录的错误代码') if code else '任务说明'
    line = f'[原因] {code}：{description}' if code else f'[原因] {description}'
    if message:
        line += f'；原始说明：{message}'
    return [line]


class MissionReporter:
    """Format changes immediately and rate-limit only the path-progress line."""

    def __init__(self, progress_interval=1.0):
        self.progress_interval = float(progress_interval)
        if not math.isfinite(self.progress_interval) or self.progress_interval <= 0:
            raise ValueError('progress_interval must be finite and positive')
        self._task = None
        self._reset()

    def _reset(self):
        self._state = self._segment = self._phase = self._completed = None
        self._error = ('', '')
        self._blocked = False
        self._progress = None
        self._progress_at = -math.inf
        self._counts = (0, 0)

    def _task_lines(self, value):
        task = (value.task_id, value.route_id)
        if task == self._task:
            return []
        self._task = task
        self._reset()
        return [f'[任务信息] 任务：{task[0] or "未分配"}，路线：{task[1] or "未指定"}']

    def feedback(self, status, now):
        lines = self._task_lines(status)
        state = _text(status.state)
        if state != self._state:
            lines.append(f'[任务状态] {_label(state, MISSION_STATES)}')
            self._state = state

        segment = (status.segment_id, status.execution_id,
                   status.segment_index, status.segment_count)
        if segment != self._segment:
            self._segment = segment
            self._phase = None
            self._progress = None
            self._progress_at = -math.inf
            if status.segment_id:
                ordinal = ('接近起点段，不计入主路段' if status.segment_index == 0 else
                           f'第 {status.segment_index}/{status.segment_count} 个主路段')
                execution = status.execution_id or '待派发'
                lines.append(f'[当前段] {status.segment_id}，{ordinal}，执行 ID：{execution}')

        phase = (_text(status.segment_state), _text(status.phase))
        if phase != self._phase:
            self._phase = phase
            if phase[0] or phase[1]:
                detail = _label(phase[0], SEGMENT_STATES) if phase[0] else '等待段状态'
                if phase[1]:
                    detail += f'，阶段：{_label(phase[1], PHASES)}'
                lines.append(f'[段状态] {detail}')

        counts = (status.completed_segments, status.segment_count)
        if counts != self._completed:
            if counts[0]:
                lines.append(f'[完成进度] 已完成 {counts[0]}/{counts[1]} 个主路段')
            self._completed = counts
        self._counts = counts

        error = (status.error_code, status.error_message)
        if error != self._error:
            lines += _reason(*error)
            self._error = error
        if status.blocked and not self._blocked:
            lines.append('[故障阻塞] 请核实当前执行及保持、控制释放状态后再复位')
        self._blocked = bool(status.blocked)

        progress = status.segment_progress
        if (status.segment_id and phase[0] in ('EXECUTING', 'ARRIVAL_CONFIRM', 'SUCCEEDED')
                and isinstance(progress, (int, float)) and math.isfinite(progress)):
            percentage = round(min(1.0, max(0.0, progress)) * 100, 1)
            if percentage != self._progress and now - self._progress_at >= self.progress_interval:
                lines.append(f'[本段路径进度] {status.segment_id}：{percentage:.1f}%')
                self._progress, self._progress_at = percentage, now
        return lines

    def result(self, result, action_status):
        lines = self._task_lines(result)
        state = _text(result.state)
        protocol = ACTION_STATES.get(action_status, f'未知状态 {action_status}')
        if (state == 'SUCCEEDED' and result.success and action_status == 4 and
                result.navigation_stopped and result.hold_confirmed):
            # Result and feedback travel independently. A successful mission
            # completes every configured main segment, even if final feedback
            # has not arrived and the last displayed count is still lower.
            _, total = self._counts
            detail = f'，已完成 {total}/{total} 个主路段' if total else ''
            lines.append(f'[任务完成] 巡检成功{detail}（SUCCEEDED）')
        elif state == 'SUCCEEDED':
            lines.append('[结果异常] 任务报告 SUCCEEDED，但成功标志、停止/保持确认或 Action 终态不一致')
        elif state == 'CANCELED' and not result.success and action_status in (5, 6):
            lines.append(f'[任务取消] 巡检已取消（CANCELED，Action {protocol}）')
        elif state == 'CANCELED':
            lines.append('[结果异常] 任务报告 CANCELED，但 success 标志或 Action 终态不一致')
        elif state == 'FAULT_BLOCKED':
            lines.append('[任务阻塞] 执行或控制释放尚未确认，需核实后显式复位（FAULT_BLOCKED）')
        elif state == 'FAILED':
            lines.append('[任务失败] 本次巡检未成功完成（FAILED）')
        else:
            lines.append(f'[任务结束] 状态：{state or "未知"}，success={result.success}，'
                         f'Action 终态：{protocol}')
        lines += _reason(result.error_code, result.error_message)
        revoked = '已撤销' if result.navigation_stopped else '撤销未确认'
        hold = '已确认' if result.hold_confirmed else '未确认'
        lines.append(f'[确认记录] 导航指令：{revoked}；运控保持：{hold}（任务结果记录）')
        return lines
