# 任务状态机与联调

本次在已有 `stair_nav` Python 包内完善 `task_manager`，没有增加第二个调度者。`core/mission.py` 拥有任务状态及转换规则；`task_node.py` 负责 ROS Action、订阅、定时器和副作用执行。规划、实时路径匹配、速度计算、入口对准、出梯和平台到达确认仍由原导航层负责。任务节点不发布速度，也不调用机器人 SDK。

## 现有接口核对与增量

| 项目 | 审查结果 / 本次处理 |
|---|---|
| 包 | `stair_nav` 为 ament_python 包，`stair_nav_interfaces` 为共享接口包；沿用它们 |
| 段服务 | `segment_navigation` 提供 `/navigate_segment`，类型 `stair_nav_interfaces/action/NavigateSegment` |
| 段请求 | `task_id`、唯一的 `execution_id`、`segment_id` |
| 段反馈 | `SegmentStatus`：Header、三个 ID、phase/state、progress、横向/朝向误差、错误、geometry_reached、navigation_stopped、hold_confirmed |
| 段结果 | success、error_code、error_message、navigation_stopped、hold_confirmed |
| 段取消 | ROS Action CancelGoal；取消响应、取消终态和整机保持是不同事件 |
| 段完成责任 | `SegmentExecutor`：几何到达后进入 ARRIVAL_CONFIRM；检查当前 execution_id 的新鲜出梯/稳定/保持/模式反馈及持续确认时间，才返回成功 |
| 已有输入 | `/localization/pose`、`/motion/status`、`/navigation/channel_event`；模拟端已有五种故障开关 |
| 已有任务入口 | `~/start`、`~/cancel` Trigger 和 `/task/status`；保留兼容，替换其内部最小调度逻辑 |
| 新接口 | `/execute_mission` 任务 Action、`/mission/status`、`~/reset`，以及只读 `/navigation/get_state` 同步服务 |
| 未更改接口 | 原 NavigateSegment、MotionStatus、ControlCommand、SegmentStatus 的字段和段成功契约不变 |

段 Action 的 success 不是几何候选。任务层不会再等待一套独立“平台成功”信号，也不会重复规划、对准或下发动作。段反馈的 `geometric_candidate` 只是进度，不能推进任务。任务层仅接受当前目标的成功结果，并核对结果状态及导航撤销/保持字段。

`/navigation/get_state` 提供当前段节点实例 ID、忙碌/预留状态、控制权释放和段快照。控制权释放需要当前运控反馈新鲜、保持且稳定；初始无执行时允许空 execution_id 的稳定静止状态。任务节点启动后先同步，不能仅以 Action 服务“在线”推断可派发。

## 配置与路线

任务配置为 [inspection_mission.yaml](../stair_nav/config/inspection_mission.yaml)。`route_file` 引用已有 [demo_route.yaml](../stair_nav/config/demo_route.yaml)，不复制 A/B/C/D 坐标或路径点；每段的 mode、allowed_modes、required_capabilities、目标节点和到达条件均从导航配置读取。

配置包含 `route_id`、有序 `sequence`、就绪新鲜度与分类超时。序列要存在于导航配置并相互连接；未知导航段、不连接序列、非法超时或不一致输入应在配置校验时拒绝。`per_segment.<id>.timeouts` 可覆盖特定段的超时，其余继承全局值。全部时间单位为秒。

| 超时 | 默认值 | 含义 |
|---|---:|---|
| service | 5 | 当前段 Action 服务可用等待 |
| ready | 5 | 定位、运控、能力及下游释放就绪等待 |
| acceptance | 3 | 已发送目标的接受响应等待；超时不能假定目标未执行 |
| execution | 55 | 单段接受后的总执行期限 |
| confirmation | 8 | 下层报告几何候选后的额外到达确认期限；段内仍有自身 arrival 超时 |
| feedback | 2 | 运行中段 Action 反馈失联期限 |
| cancel_response | 3 | 取消请求响应期限 |
| cancel_terminal | 5 | 取消后等段终态期限 |
| cleanup | 5 | 段终态后等新鲜保持/稳定与控制释放期限 |

定位和运控最大年龄默认 0.5 秒，下游同步快照默认 1 秒。预检要求定位有限且在同一 frame，运控 ready、无故障、支持本段所需能力和模式，下游无活动/预留目标且允许释放。当前模式不必已经是 stair：具体模式握手由段执行器的 HOLD 请求触发，段执行器确认后才运动。段间重新检查这些条件，并从实时定位规划下一段。

派发就绪同时要求最新运控反馈确认控制权释放。即使缓存的下游快照仍显示已释放，更新的反馈若失去稳定、保持，或执行/段 ID 不匹配，也会阻止派发。纯核心的 `Readiness.target_segment_id` 标识本次检查的目标段；它不同于关联上一执行释放证据的 `execution_id`。成功结果切换段后，旧段的就绪信息不能授权新段，必须重新检查新段的能力和模式。缺少条件时保持 WAITING_READY，条件补齐后派发一次，等待超时才结束任务。

首版任务启动要求机器人已经位于首段起点 A 的位置/高度/朝向容差内；该容差引用导航配置。模拟器从 A 起步。现有工程没有“当前位置→A”的独立导航段或站点地图，本次没有虚构一条接近路线。真实设备若从其他位置启动，应先由已有定位/导航系统到达 A，再发本任务；自动接近 A 需提供经过验证的接近段配置，不能用修改模拟位姿代替。

## 状态转换表

核心是串行裁决的轻量枚举状态机，所有时钟输入可注入，纯核心不导入 ROS。ROS 回调把事件连同 task_id/execution_id 入队；定时器处理事件、更新状态、执行 SEND/CANCEL/终态等副作用。转换日志含时间、任务/段/目标标识、原状态、事件、新状态、错误和原因。

| 状态 | 进入动作 | 允许事件、条件与下一状态 | 超时处理 | 退出动作 |
|---|---|---|---|---|
| IDLE | 发布空闲快照，无自动续跑 | 接受合法路线且无预留/活动任务 → VALIDATING | 无 | 建立唯一任务 ID，上层 Action 立即预留 |
| VALIDATING | 校验配置/路线和任务上下文 | 有效 → WAITING_READY；无效 → FAILED；取消 → CANCELED | 校验同步完成 | 初始化首段计数及期限 |
| WAITING_READY | 检查当前段实时定位/运控与下游同步状态 | 条件满足且服务可用 → DISPATCHING；取消 → CANCELED | service/ready → FAILED，无段目标则无需伪造取消 | 生成唯一 execution_id，只发一次段目标 |
| DISPATCHING | 异步 send_goal，记录目标 UUID、接受期限 | 接受 → RUNNING；明确拒绝 → FAILED；取消/通信异常 → CANCELING | acceptance → CANCELING，保持执行状态未知 | 记录接受结果；迟到接受仍登记并清理 |
| RUNNING | 等段结果，转发 phase/state/progress | 当前目标确认成功 → 下一段 WAITING_READY 或整任务 SUCCEEDED；拒绝/失败/取消/异常 → CANCELING | execution/feedback/confirmation → CANCELING | 冻结推进意图，失败不重试 |
| CANCELING | 锁存取消或失败原因；有句柄则发取消，无句柄等接受响应 | 取消响应只记录 accepted；段终态且当前执行的新鲜保持/稳定与释放确认 → CANCELED 或 FAILED；迟到成功也不能发下一段 | cancel_response/cancel_terminal/cleanup → FAULT_BLOCKED | 仅清理确认后释放活动任务 |
| SUCCEEDED | 全部配置段已确认成功，发布一次任务结果 | 新请求可建立新任务；迟到事件不改变终态 | 无 | 新任务使用新 ID，重新检查起点和就绪 |
| FAILED | 导航失败且清理已确认，或尚未派发即失败；发布一次结果与原因 | 新请求可建立新任务，必须重新满足预检 | 无 | 无自动重试、倒退或暂停恢复 |
| CANCELED | 已派发目标完成清理，或尚未派发即取消；发布一次结果 | 新请求可建立新任务，必须重新满足预检 | 无 | 旧 ID 回调仅允许清理，无状态复活 |
| FAULT_BLOCKED | 发布任务失败结果，保留未知执行/控制权阻塞 | 新任务拒绝；迟到句柄继续取消；仅明确 reset 且原目标已解析、终态已知、新鲜快照无忙碌并已释放 → IDLE | 不自动解除 | 记录人工复位原因并清除已确认故障 |

没有独立 VERIFYING 状态：完整段到达确认已经由段执行器承担。任务的 confirmation 看门狗用于约束下层确认阶段的等待，不建立第二个到达判定者。

## 异步隔离与取消契约

每个任务生成新的 task_id，每次段执行生成新的 execution_id；ROS 层另记录任务 Action UUID 和段 Action UUID。所有回调核对关联标识。旧反馈、旧结果、重复成功都不能修改新任务或重复派发下一段；终态结果只发一次。旧接受回调若携带已接受句柄，仍会请求取消并等待清理，不能仅忽略。

任务节点使用多线程 Executor、可重入回调组和锁保护。回调不调用阻塞 wait_for_server、sleep、嵌套 spin 或同步等结果；长时间任务执行协程等待 Future，定时器和取消回调继续运行。状态转换经事件队列串行裁决。默认只有一个任务，不提供队列、抢占或暂停恢复。

取消流程明确区分：

1. 接收任务取消意图，并禁止后续段派发。
2. 有目标句柄时发送下游取消；尚未收到接受响应时保留意图，后续接受后继续取消。
3. 收到取消响应，记录下游是否接受取消；这不等于执行已结束。
4. 收到下游 Action 终态，确认导航已撤销；这仍不等于机器人已完成保持/退出。
5. 等当前执行 ID 的新鲜 MotionStatus 同时报告 hold_confirmed 与 stable，并经下游同步确认控制释放，才结束任务清理。

上述下游清理适用于已经派发段目标的任务。若取消发生在 VALIDATING/WAITING_READY、尚未派发任何目标，任务没有取得运动控制权，可以直接 CANCELED；此时不伪造保持确认，hold_confirmed 可以为 false。

取消意图先被任务管理器接受时，之后迟到的段成功不能推进下一段。取消被拒绝、取消响应丢失或终态/保持确认超时进入 FAULT_BLOCKED；不将零速度、无效指令、Action CANCELED 或单个 hold 标记单独当作完成。

失败也进入约定清理流程，通过段 Action 取消及现有 HOLD/适配器策略撤销执行。首版失败后结束任务，不自动重试楼梯或后退。

## 启动、查看、取消与重启

所有包在 `/home/syf/ros2_ws` 根目录构建。这里把产物写入 `/tmp`，以免覆盖已有工作区安装：

```bash
cd /home/syf/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/stair_nav_mission_log build \
  --base-paths src/stair_nav/stair_nav_interfaces src/stair_nav/stair_nav \
  --packages-up-to stair_nav --build-base /tmp/stair_nav_mission_build \
  --install-base /tmp/stair_nav_mission_install --symlink-install
source /tmp/stair_nav_mission_install/setup.bash
ros2 launch stair_nav demo.launch.py rviz:=false autostart:=false
```

默认 demo 启动一个模拟器、一个段执行器和一个任务管理器；只有 demo 显式 autostart 才自动发一个新任务。普通 `task_manager` 和真实导航 launch 均不自动开始。避免同时启动旧的手工段客户端或额外调度器。

新终端 source 同一安装空间后：

```bash
ros2 action send_goal /execute_mission stair_nav_interfaces/action/ExecuteMission \
  '{route_id: inspection_demo}' --feedback
ros2 topic echo /mission/status --qos-durability transient_local
ros2 service call /task_manager/cancel std_srvs/srv/Trigger '{}'
```

Action 客户端应优先使用自身目标句柄的 cancel_goal_async；取消握手成功且清理完成后，ROS 目标协议状态为 CANCELED。兼容旧客户端的 `/task_manager/start` 和 `/task_manager/cancel` 服务仍存在；若通过旧 cancel 服务取消 Action 创建的任务，任务语义结果仍为 `result.state=CANCELED`，但没有客户端 Action 取消握手，ROS 目标协议状态使用 ABORTED，调用方须读取结果字段。上面的服务命令适合 CLI 运维取消。

`/task/status` 仍是 SegmentStatus 的兼容快照，完整任务进度应改订阅 MissionStatus。所有入口共用一个预留/活动任务检查，忙碌或阻塞时拒绝新请求。

任务完成后机器人仍在当前位置，不自动回到 A。重新执行整条路线要先让真实设备回到 A 并满足预检；合成测试可结束整套 demo，再启动全新模拟进程。仅重启任务管理器不会恢复旧任务；新任务派发前会查询下游忙碌与控制权，不能靠重启清掉执行不明状态。

故障解除步骤：先检查 `/mission/status` 的 error_code、取消阶段及关联 execution_id，核实段执行器、运动适配器和机器人真实状态。原目标必须已明确拒绝或返回终态，当前数据新鲜、无忙碌且保持/稳定与控制释放确认后，再调用：

```bash
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
ros2 service call /task_manager/reset std_srvs/srv/Trigger '{}'
```

reset 是带守卫的重新同步，不是强制清空。目标接受或结果仍不明时应拒绝复位；修复通信并取得原目标结局，或在操作方确认设备状态后受控重建整个执行链。不要重复发送任务来探测是否恢复。

## 状态字段、时间与 QoS

`ExecuteMission.Goal` 指定 route_id。反馈中的 MissionStatus 包含 task_id、route_id、execution_id、当前段 ID/序号/总数、完成段数、任务 state、下层 phase/segment_state/segment_progress、错误、停止/保持、取消阶段和 blocked。结果给出任务 ID、route_id、终态、成功、错误及停止/保持信息。

`segment_index` 从 1 开始；`completed_segments` 只在对应段确认成功后增加。`segment_progress` 原样反映下层三维路径弧长匹配进度，不能解释为整个任务距离或耗时百分比。需要显示总体进度时可显示“已完成 n/N 段 + 当前段进度”，不要把不同长度段简单称为距离百分比。

| 接口 | QoS / 时间含义 |
|---|---|
| `/mission/status` MissionStatus | Reliable、TransientLocal、KeepLast(1)；Header 是任务快照发布时刻，valid_until 是活性有效期；晚启动监控可取得当前快照，但仍应检查有效期 |
| `/task/status`、`/navigation/status` SegmentStatus | Reliable、TransientLocal、KeepLast(1)；兼容状态与段状态持续更新，不能作为新的运动授权 |
| `/execute_mission` | ROS Action 默认服务/反馈 QoS；任务结果仅处理一次；长时间结果可另存日志，不依赖反馈可靠保留 |
| `/localization/pose`、`/motion/status` | Reliable、Volatile、KeepLast(5)；header.stamp 为观测时刻，与 route frame 及 ROS 时钟一致，不能收包时重盖章 |
| `/navigation/get_state` | 只读 ROS Service；服务端时戳及实例标识用于同步，快照只在配置的新鲜期限内使用 |

未来时间偏差最多容忍 0.1 秒；过期、非有限、错误 frame 或错误 execution_id 的数据不能满足就绪/清理。取消清理确认还必须来自清理开始后的新观测。MissionStatus 的终态 navigation_stopped/hold_confirmed 是任务结束时锁存的结果证据；Header/valid_until 只续租任务快照发布者的活性，不续租物理保持证据。机器人当前状态须查看 /motion/status 的观测时戳。每次重新派发和复位都会独立检查新鲜运控与下游释放反馈。多机部署需统一 ROS 时间源并验证时间同步；时钟后退按故障处理。

## 故障注入和验证

以下命令各启动一套隔离的模拟流程，不应与同 Domain 的另一套 demo 同时运行：

```bash
ros2 launch stair_nav demo.launch.py rviz:=false not_ready:=true
ros2 launch stair_nav demo.launch.py rviz:=false missing_exit:=true
ros2 launch stair_nav demo.launch.py rviz:=false unstable:=true
```

运行中也可注入定位丢失或执行异常：

```bash
ros2 service call /sim_executor/localization_dropout std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim_executor/execution_fault std_srvs/srv/SetBool '{data: true}'
```

纯核心测试使用受控时钟和事件，无需 ROS 图或长 sleep。真实 ROS 场景使用独立 Domain、新进程和原有规划/跟踪/模拟端；测试替身不能与真实 `/navigate_segment` 同名竞争。运行方法及实际结果由 [验证记录](VALIDATION.md) 的任务状态机部分列出。

```bash
cd /home/syf/ros2_ws/src/stair_nav
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
source /tmp/stair_nav_mission_install/setup.bash
python3 tools/test_ros_callbacks.py
python3 tools/mission_smoke.py --output-dir /tmp/stair_nav_mission_smoke
```

任务 ROS 脚本覆盖正常顺序、取消/早取消、未就绪、定位过期、缺少出梯、失联阻塞及任务节点重启不续跑；每个用例使用新模拟进程。`transport_timeout` 只终止脚本自己启动的段节点。

## 真实运控接入仍需确认

- A 的接近路线、可通行区域及接近过程由谁执行；无配置时只接受已经到 A 的任务。
- 真实平台出梯、稳定、保持/退出的传感器判据，以及不同楼梯阶段允许的处理动作。
- MotionStatus 中当前执行 ID、模式、能力、时间戳与故障的真实含义；反馈频率和丢包容忍值。
- 控制权释放是否包含策略退出、机器人 SDK 清理及外部控制权仲裁；不能只观察发布速度为零。
- 通信故障、段节点/任务节点重启时的独立 TTL 看门狗，以及如何获得原目标结局并执行人工重新同步。
- 合成坐标、楼梯剖面与跟踪参数替换为实测配置，确认参考点、楼层、平台栅格及模式转换。

未实现任务队列、抢占、暂停恢复、自动重试、自动后退、断电续跑或真实 SDK。模拟 HOLD/稳定只是软件联调契约，不能证明实体机器人已安全停稳。
