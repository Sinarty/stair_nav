"""Chinese command-line client for one ExecuteMission goal."""
import argparse
import math
import signal
import sys
import time

from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from stair_nav_interfaces.action import ExecuteMission

from .mission_console import MissionReporter


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('请输入大于 0 的有限秒数')
    return number


def argument_parser():
    parser = argparse.ArgumentParser(
        description='启动一次巡检任务，以中文显示状态、进度和最终结果。')
    parser.add_argument('--route-id', default='',
                        help='任务路线标识；默认空值表示使用服务端已加载任务')
    parser.add_argument('--action', default='/execute_mission',
                        help='任务 Action 名称，默认 /execute_mission')
    parser.add_argument('--server-timeout', type=positive_seconds, default=10.,
                        help='等待 Action 服务可用的最长时间，默认 10 秒')
    parser.add_argument('--progress-interval', type=positive_seconds, default=1.,
                        help='进度输出的最小间隔，默认 1 秒；状态变化立即显示')
    parser.add_argument('--cancel-timeout', type=positive_seconds, default=15.,
                        help='Ctrl+C 后等待最终结果的最长时间，默认 15 秒')
    return parser


class MissionClient(Node):
    """Own exactly one goal; never retry a send or cancel through a service."""

    def __init__(self, options, context=None):
        super().__init__('mission_client', context=context)
        self.options = options
        self._mission_executor = SingleThreadedExecutor(context=self.context)
        self._mission_executor.add_node(self)
        self.action = ActionClient(
            self, ExecuteMission, self.resolve_topic_name(options.action))
        self.reporter = MissionReporter(options.progress_interval)
        self.interrupts = 0
        self.sent = False
        self.finished = False
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None
        self.cancel_response_seen = False
        self.cancel_deadline = None
        self.pending_feedback = []

    @staticmethod
    def say(message):
        print(message, flush=True)

    def on_interrupt(self, signum, frame):
        # Keep the ROS context alive so cancellation can be sent and confirmed.
        self.interrupts += 1

    def on_feedback(self, message):
        if self.goal_handle is None:
            self.pending_feedback.append(message.feedback.status)
            return
        self.show_feedback(message.feedback.status)

    def show_feedback(self, status):
        for line in self.reporter.feedback(status, time.monotonic()):
            self.say(line)

    def uncertain(self, detail):
        self.say(f'[结果未确认] {detail}')
        self.say('[请核实] 请查看 /mission/status 和实际运控状态；'
                 '退出客户端不代表机器人已停止，请勿直接重复发送任务。')
        return 1

    def run(self):
        self.say(f'[等待服务] {self.options.action}')
        deadline = time.monotonic() + self.options.server_timeout
        while self.context.ok() and not self.action.server_is_ready():
            if self.interrupts:
                self.say('[启动取消] 尚未发送任务请求。')
                return 130
            if time.monotonic() >= deadline:
                self.say('[启动失败] 等待 Action 服务超时，未发送任务。')
                return 1
            self._mission_executor.spin_once(timeout_sec=.1)
        if self.interrupts:
            self.say('[启动取消] 尚未发送任务请求。')
            return 130
        if not self.context.ok():
            self.say('[启动失败] ROS 通信已关闭，未发送任务。')
            return 1

        route = self.options.route_id or '服务端已加载任务'
        self.say(f'[发送任务] 路线：{route}；仅发送一次请求。')
        self.sent = True
        self.goal_future = self.action.send_goal_async(
            ExecuteMission.Goal(route_id=self.options.route_id),
            feedback_callback=self.on_feedback)

        while self.context.ok():
            self._mission_executor.spin_once(timeout_sec=.1)
            if self.goal_handle is None and self.goal_future.done():
                handle = self.goal_future.result()
                if not handle.accepted:
                    self.finished = True
                    self.say('[任务拒绝] 请求未被接受。请检查路线标识、'
                             '已有任务及故障阻塞状态；不会自动重发。')
                    return 1
                self.goal_handle = handle
                self.say('[任务受理] 请求已接受，仍需通过就绪检查；'
                         f'目标 ID：{bytes(handle.goal_id.uuid).hex()}')
                self.result_future = handle.get_result_async()
                for status in self.pending_feedback:
                    self.show_feedback(status)
                self.pending_feedback.clear()

            # A final result can win a race with an interrupt/cancel response.
            if self.result_future is not None and self.result_future.done():
                response = self.result_future.result()
                self.finished = True
                for line in self.reporter.result(response.result, response.status):
                    self.say(line)
                result = response.result
                if (response.status == 4 and result.success and result.state == 'SUCCEEDED'
                        and result.navigation_stopped and result.hold_confirmed):
                    return 0
                if (response.status in (5, 6) and not result.success and
                        result.state == 'CANCELED'):
                    return 130
                return 1

            if self.interrupts and self.cancel_deadline is None:
                self.cancel_deadline = time.monotonic() + self.options.cancel_timeout
                self.say('[请求取消] Ctrl+C 已触发取消；等待目标受理与最终结果。'
                         '再次按 Ctrl+C 仅退出客户端。')
            if self.cancel_deadline is not None:
                if self.goal_handle is not None and self.cancel_future is None:
                    self.cancel_future = self.goal_handle.cancel_goal_async()
                if (self.cancel_future is not None and self.cancel_future.done() and
                        not self.cancel_response_seen):
                    self.cancel_response_seen = True
                    try:
                        response = self.cancel_future.result()
                        target = bytes(self.goal_handle.goal_id.uuid)
                        accepted = any(bytes(item.goal_id.uuid) == target
                                       for item in response.goals_canceling)
                        if accepted:
                            self.say('[取消受理] 正在等待任务结束及停止清理结果。')
                        else:
                            self.say('[取消未获接受] 继续等待原任务的最终结果。')
                    except Exception as exc:
                        self.say(f'[取消通信异常] {exc}；继续等待原任务的最终结果。')
                if self.interrupts >= 2:
                    self.uncertain('再次收到 Ctrl+C，结束客户端等待，任务终态尚未确认。')
                    return 130
                if time.monotonic() >= self.cancel_deadline:
                    return self.uncertain('取消等待超时，无法确认任务已经结束。')
        return self.uncertain('ROS 通信已关闭，无法取得任务最终结果。')


def main(args=None):
    argv = sys.argv if args is None else ['mission_client', *args]
    options = argument_parser().parse_args(remove_ros_args(args=argv)[1:])
    node = None
    context = Context()
    previous_sigint = signal.getsignal(signal.SIGINT)
    try:
        # Context.init is common to Galactic and Humble and does not install
        # Humble's process-wide shutdown-on-SIGINT handler. Keep cancellation
        # on this private context until its result is known.
        context.init(args=argv)
        node = MissionClient(options, context=context)
        signal.signal(signal.SIGINT, node.on_interrupt)
        return node.run()
    except KeyboardInterrupt:
        if node is not None and node.sent and not node.finished:
            return node.uncertain('客户端被中断，未取得任务终态。')
        print('[启动取消] 尚未发送任务请求。', flush=True)
        return 130
    except Exception as exc:
        if node is not None and node.sent and not node.finished:
            return node.uncertain(f'客户端异常：{type(exc).__name__}: {exc}')
        print(f'[客户端错误] {type(exc).__name__}: {exc}', flush=True)
        return 1
    finally:
        try:
            if node is not None:
                node._mission_executor.shutdown()
                node.action.destroy()
                node.destroy_node()
        finally:
            try:
                if context.ok():
                    context.shutdown()
            finally:
                signal.signal(signal.SIGINT, previous_sigint)


if __name__ == '__main__':
    raise SystemExit(main())
