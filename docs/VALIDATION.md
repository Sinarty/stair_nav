# 实际验证记录

验证日期：2026-10-28（Asia/Shanghai）。环境为 Ubuntu 22.04、Python 3.10.12、ROS 2 Humble；当前目录最初无业务代码，未修改旁边的其他 ROS 包。构建产物放在本项目 `.colcon/`，未写入已有工作区 build/install。

## 执行结果

| 验证 | 实际结果 | 原始记录 |
|---|---|---|
| colcon build | 两个包成功，首次构建约 3 分 1 秒 | [接口包](build_interfaces.log)、[Python 包](build_python.log) |
| 最终 Python 包复建 | 加入 TF 依赖后复建成功 | [最终构建](final_build.log) |
| colcon test / test-result | 70 tests，0 errors，0 failures，0 skipped | [colcon 测试日志](colcon_test.log) |
| ROS 无关核心测试 | 70 项通过 | [JUnit](core_tests.xml) |
| 确定性三段闭环 | AB、BC、CD 全成功；24.6 秒模拟时间；439 次有效控制采样 | [完整状态与末位姿](headless_demo.json) |
| ROS Action/话题/服务 | 正常三段、未就绪、定位过期、执行取消、缺少出梯确认、通道阻塞，6 个场景通过 | [JSON](ros_smoke_results.json)、[日志](ros_logs/) |
| 导航 Action 服务端失联 | 中途终止测试自己的段导航进程；任务检测超时、报告停止状态未知、拒绝新任务 | [JSON](transport_smoke_results.json)、[日志](ros_logs/transport_timeout.log) |
| 目标接受后立即取消 | 即时 Action 取消通过；返回 CANCELED，导航已撤销、保持尚未确认 | [JSON](immediate_cancel_results.json)、[日志](ros_logs/immediate_cancel.log) |
| ROS 回调竞态与入口校验 | 13/13 通过，含执行回调尚未开始时的取消，以及终态结果与迟到取消的竞态 | [回调测试](ros_callback_tests.log) |
| RViz 实际演示 | 已启动 RViz 并显示节点、路径、轨迹、前视点、楼梯/平台区域及合成栅格 | [截图](rviz_demo.png)、[日志](rviz_smoke.log) |

核心测试覆盖：A* 绕障/不可达/禁止对角穿角/未知格/膨胀/简化后碰撞检查、错误楼层、空路径及非有限/重复/跳变点、入口接入拒绝、三维重叠路径的高度与历史进度匹配、正常限速及变化率、能力约束、规划取消与超时、旧规划 Future 迟到、旧任务反馈/结果、忙碌拒绝、定位/反馈过期、执行异常及超时、通道阻塞、几何到达但缺少整机出梯确认。

模拟适配器测试还覆盖：没有有效指令时不移动、TTL 到期撤销、无效速度/时间/任务 ID/模式拒绝、同时间戳撤销优先、旧执行指令拒绝、HOLD 确认延迟。正常闭环的 CD **实际积分轨迹**逐段对膨胀栅格进行了碰撞复核。此检查只针对这份合成场景，不代表运行时具备实时障碍感知或真实楼梯通行认证。

确定性闭环末位姿为 `(8.9864, 0.0171, 2.0)` m，yaw 为 `-0.1680` rad。XY/yaw 来自控制指令积分，高度来自配置剖面采样；没有将机器人推送到目标点。

## 复现

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base .colcon/log build --base-paths stair_nav_interfaces stair_nav \
  --build-base .colcon/build --install-base .colcon/install --symlink-install
source .colcon/install/setup.bash
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
python3 tools/test_ros_callbacks.py
python3 tools/ros_action_smoke.py --output-dir /tmp/stair_nav_ros_smoke
python3 tools/ros_action_smoke.py --cases transport_timeout \
  --domain-base 112 --output-dir /tmp/stair_nav_transport_smoke
python3 tools/ros_action_smoke.py --cases immediate_cancel \
  --domain-base 113 --output-dir /tmp/stair_nav_immediate_cancel_smoke
```

`tools/ros_action_smoke.py` 每个场景使用新模拟进程和独立 ROS domain，场景间不重置正在运行的机器人位姿。正常用例检查三个导航段都进入执行，且任务仅在 CD 结果成功后完成。取消用例还注入其他 execution_id 的迟到错误反馈，检查其不会影响当前任务。

纯核心演示的规划调度器在测试中同步计算以保证确定性；ROS 节点使用真实后台线程池。规划进行期间的取消/超时及迟到结果由受控 Future 测试独立验证。

本环境第一次在限制网络的沙箱内启动 Fast DDS 时出现 `Operation not permitted`；完整六场景复验使用允许本机 DDS 通信的环境通过。默认节点只连接本项目模拟端。测试不涉及真实机器人或真实厂区地图。

## 已修复的联调问题

初版任务管理器曾把 AB 的 `SUCCEEDED` 反馈透传成整任务成功；真实 ROS 测试检出了这一点。现在单段终态反馈是 `SEGMENT_SUCCEEDED/SEGMENT_FAILED/SEGMENT_CANCELED`，整任务只根据对应执行的 Action 结果推进，最后一段成功才发布任务 `SUCCEEDED`。已增加迟到反馈/结果、取消前后竞态回归测试。

另补齐了目标已被接受但执行回调尚未开始时的取消处理，以及模拟端动态 TF。TF 直接采用积分后的导航参考点，不添加机体高度。

## 未验证部分与联调清单

- 真实点云、楼层平台分割及栅格生成未接入；需要建图团队提供分平台的栅格、原点、分辨率、高度、参考点和未知区域约定。
- 预设楼梯通道及入口区域仍需现场测绘和可通行性确认；合成数据中的 `entry_corridor_confirmed` 不能复制到现场后视为认证。
- 运控协议、整机出梯/稳定判据、模式切换、HOLD/退出策略与物理停稳确认需要真实适配器实现。
- 所有跟踪参数为演示值。真实楼梯中的转向、原地转向、倒退及水平速度到策略速度的换算，需要按实际能力和策略响应标定。
- 模拟不包含台阶接触、打滑、动力学、机体倾斜或定位协方差。真实系统的碰撞监控、紧急停机和安全控制链仍由对应系统承担。
- Action 通信超时后保留任务忙碌，避免未知执行结果下竞争任务；恢复时需先由运控/操作方确认执行与保持状态，再受控重启或重建任务。这是当前首版的保守恢复边界。
