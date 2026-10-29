# 实际验证记录

本文保留验证结论、环境、限制与复现命令。构建/运行日志、JUnit XML 和场景结果 JSON 是本地产物，不由 Git 跟踪；原有本地副本保留。新克隆的仓库可运行下列命令重新生成结果，输出放在 `/tmp` 或工程根目录已忽略的 `artifacts/`。RViz 截图作为文档插图继续保留。

## 原规划/跟踪版本记录

以下至“任务状态机增量验证”之前为原规划/跟踪版本的历史记录，验证日期：2026-10-28（Asia/Shanghai）。环境为 Ubuntu 22.04、Python 3.10.12、ROS 2 Humble；当时构建产物放在本项目 `.colcon/`，未写入已有工作区 build/install。任务状态机增量采用工作区根目录构建，新的执行结果单独记录，旧结果不能视为新增代码已验证。

## 执行结果

| 验证 | 实际结果 |
| --- | --- |
| colcon build | 两个包成功，首次构建约 3 分 1 秒 |
| 最终 Python 包复建 | 加入 TF 依赖后复建成功 |
| colcon test / test-result | 70 tests，0 errors，0 failures，0 skipped |
| ROS 无关核心测试 | 70 项通过 |
| 确定性三段闭环 | AB、BC、CD 全成功；24.6 秒模拟时间；439 次有效控制采样 |
| ROS Action/话题/服务 | 正常三段、未就绪、定位过期、执行取消、缺少出梯确认、通道阻塞，6 个场景通过 |
| 导航 Action 服务端失联 | 中途终止测试自己的段导航进程；任务检测超时、报告停止状态未知、拒绝新任务 |
| 目标接受后立即取消 | 即时 Action 取消通过；返回 CANCELED，导航已撤销、保持尚未确认 |
| ROS 回调竞态与入口校验 | 13/13 通过，含执行回调尚未开始时的取消，以及终态结果与迟到取消的竞态 |
| RViz 实际演示 | 已启动 RViz 并显示节点、路径、轨迹、前视点、楼梯/平台区域及合成栅格；[文档截图](rviz_demo.png) |

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

## 任务状态机增量验证

本次验证从 `/home/syf/ros2_ws` 根目录构建 `stair_nav_interfaces` 与 `stair_nav`，产物位于 `/tmp/stair_nav_mission_build`、`/tmp/stair_nav_mission_install`，构建日志位于 `/tmp/stair_nav_mission_logs`。复现命令、配置和状态契约见 [MISSION.md](MISSION.md)。

验证日期：2026-10-28（Asia/Shanghai）。历史 70 项/13 项结果仅对应原版本；本次实际执行如下。

| 验证 | 实际结果 |
| --- | --- |
| 工作区根目录构建 | 两包成功，3 分 30 秒；未覆盖相邻包或原工作区安装 |
| 纯核心 pytest | 117/117 通过，其中新增任务状态机 47 项 |
| colcon test / test-result | 117 tests，0 errors，0 failures，0 skipped |
| ROS 回调与就绪边界 | 26/26 通过；无 ROS 图，真实回调 + 可控 Future |
| 真实任务 Action 联调 | 8/8 场景通过，原规划/跟踪/模拟链路 |
| 原 Launch/Trigger 正常链路 | AB、BC、CD 执行且最终 CD/SUCCEEDED |
| 默认 demo 自动启动 | 单调度者、单执行链、AB→BC→CD 成功后定时 SIGINT 退出 |
| 原接口故障与立即取消 | 失联阻塞/新任务拒绝、单段即时取消，2/2 通过 |

任务 Action 八场景：顺序成功（26.171 秒，三个不同执行 ID，每段仅一次；忙碌请求拒绝；晚订阅取得终态）、运行取消（等待 hold/stable 后完成）、派发前早取消（未创建导航段，hold=false 符合无自有运动契约）、未就绪（READY_TIMEOUT，无派发）、定位过期（POSE_STALE，确认清理）、缺少出梯（ARRIVAL_CONFIRM_TIMEOUT，AB 几何候选不推进）、执行器失联（FAULT_BLOCKED/CANCEL_RESPONSE_TIMEOUT，停止与保持未知，新任务和 reset 均拒绝）、仅重启任务节点（IDLE、空任务 ID、不续跑旧段）。

核心和回调测试补齐真实图不便稳定制造的竞态：接受响应前取消、接受超时后的迟到句柄、取消响应拒绝/丢失、已接受取消但终态超时、清理确认过期或错误执行 ID、取消后的迟到成功、旧任务/重复结果、结果注册异常、早取消的 ROS CANCELING 协议时序、时钟回退、配置引用不一致、执行/确认/反馈超时、故障保持与带守卫复位。测试使用受控时钟/事件，无长 sleep。

本机 ROS 联调设置 ROS_LOCALHOST_ONLY=1，使用独立 Domain 和全新模拟进程；Fast DDS 在受限环境打印 UDP 权限诊断，但同机共享内存通信完成了实际 Action/Service/Topic 验证。未验证跨机器 DDS 通信、真实设备、真实地图和任意当前位置到 A 的自动接近。

复现本次构建和测试：

```bash
cd /home/syf/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/stair_nav_mission_logs build \
  --base-paths src/stair_nav/stair_nav_interfaces src/stair_nav/stair_nav \
  --packages-up-to stair_nav --build-base /tmp/stair_nav_mission_build \
  --install-base /tmp/stair_nav_mission_install --symlink-install
source /tmp/stair_nav_mission_install/setup.bash
colcon --log-base /tmp/stair_nav_mission_test_logs test \
  --base-paths src/stair_nav/stair_nav_interfaces src/stair_nav/stair_nav \
  --packages-select stair_nav --build-base /tmp/stair_nav_mission_build \
  --install-base /tmp/stair_nav_mission_install --event-handlers console_direct+
colcon --log-base /tmp/stair_nav_mission_test_logs test-result \
  --test-result-base /tmp/stair_nav_mission_build --verbose
cd /home/syf/ros2_ws/src/stair_nav
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
python3 tools/test_ros_callbacks.py
ROS_LOCALHOST_ONLY=1 python3 tools/mission_smoke.py \
  --domain-base 171 --output-dir /tmp/stair_nav_mission_ros
ROS_LOCALHOST_ONLY=1 python3 tools/ros_action_smoke.py \
  --cases normal --domain-base 201 --output-dir /tmp/stair_nav_mission_legacy
ROS_LOCALHOST_ONLY=1 python3 tools/ros_action_smoke.py \
  --cases transport_timeout,immediate_cancel --domain-base 181 \
  --output-dir /tmp/stair_nav_legacy_mission_smoke
```

联调中修复了两个回调边界：任务早取消必须等 ROS 目标进入 CANCELING 后才调用 canceled；已接受的段目标即使 get_result_async 注册异常，也必须保留已知接受状态并取消。另确保旧/重复结果不会更新新段的同步时间屏障。

## 就绪守卫修复验证（2026-10-29）

修复两项审查发现：最新运控反馈否定旧释放快照后仍可能派发，以及成功结果切换导航段时复用上一段的就绪条件。

- 派发同时要求新鲜、匹配的释放证据；失去稳定、保持或执行/段标识不匹配时继续等待。初始无执行时，空执行标识且稳定、尚无保持确认的既有启动契约保留。
- 纯核心 `Readiness` 增加 `target_segment_id`，派发守卫同时核对目标段。结果回调选择下一段后，旧段事实不再授权新段；重新评估新段能力和模式后才可派发。
- 该字段仅属内部纯 Python 数据模型，任务/段 ROS Action、Message、Service 定义未改变。

新增 6 项核心测试和 3 项回调测试，并加强已有释放检查。确定性回归覆盖：更新的反馈否定旧快照时无 SEND，反馈恢复后仅 SEND 一次；BC 结果与新快照在定时器前到达时，缺少 CD 的 turn_in_place 能力不会派发 CD；补齐后仅派发一次；持续缺少前置条件时由 READY_TIMEOUT 结束。

| 验证 | 结果 |
| --- | --- |
| 工作区根目录增量构建 | 受影响的 stair_nav 包成功，2 分钟；复用未变化的接口包 |
| 全部纯核心测试 | 123/123 通过（任务核心 53 项） |
| 实际 ROS 回调及就绪测试 | 29/29 通过 |
| 原规划/跟踪/模拟端任务 Action 联调 | 8/8 通过；顺序、取消、就绪、过期、缺少出梯、失联阻塞与重启行为正常 |

正常任务实际耗时 25.702 秒，AB/BC/CD 三个执行 ID 各出现一次；忙碌拒绝和晚订阅终态检查通过。取消、失联阻塞及重启不续跑行为未受本次守卫变更影响。

复现新增检查（先按前文从工作区根目录构建并 source 相同安装空间）：

```bash
cd /home/syf/ros2_ws/src/stair_nav
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
python3 tools/test_ros_callbacks.py
ROS_LOCALHOST_ONLY=1 python3 tools/mission_smoke.py \
  --domain-base 211 --output-dir /tmp/stair_nav_readiness_fix_ros
```
