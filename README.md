# 分段路径规划与跟踪：ROS 2 Humble 首版

本工程实现固定巡检路线 **A→B→C→D**，通过一个可复用的段导航 Action 依次执行 AB、BC、CD。它包含实际可运行的规划、实时位姿闭环跟踪、到达确认、任务调度、结构化运控接口和模拟执行端。算法核心不依赖 ROS；不依赖 Nav2 或行为树。

**默认数据全部为合成场景。** AB、BC 是两段带平台的人工三维楼梯参考路径；CD 在二层平台占据栅格上绕障规划。示例没有证明任何真实点云地图上的可通行性。默认启动不会加载真实机器人 SDK。

实际执行的构建、测试、节点联调结果见 [docs/VALIDATION.md](docs/VALIDATION.md)；可复现的纯核心演示记录见 [docs/headless_demo.json](docs/headless_demo.json)。

已实际通过：两个 ROS 包构建、70 项核心测试、13 项 ROS 回调/入口测试，以及三段正常任务和七种取消/故障场景。RViz 实拍见 [演示截图](docs/rviz_demo.png)。

## 1. 工程结构与职责

```text
stair_nav/                          # ament_python 包
  stair_nav/core/
    models.py                       # 位姿、路径、指令、反馈、错误等纯数据
    config.py                       # YAML 读取、继承合并与校验
    planning.py                     # 统一后端、preset_path、grid_astar
    tracking.py                     # 三维匹配 + Pure Pursuit + 指令限制
    engine.py                       # 同一套段执行状态机、规划工作线程
    simulation.py                   # 速度积分与模拟运控反馈
  stair_nav/segment_node.py          # /navigate_segment Action 服务端
  stair_nav/task_node.py             # 串行任务客户端、任务取消与超时监视
  stair_nav/sim_node.py              # 模拟定位/运控 ROS 节点
  stair_nav/demo.py                  # 无 ROS、确定时钟闭环演示
  config/                           # 路线、带阶段路径、单平台合成栅格
  launch/{demo,navigation}.launch.py
  rviz/demo.rviz
  test/                             # 核心与生命周期回归测试
stair_nav_interfaces/                # 独立 msg/action 接口包
  action/NavigateSegment.action
  msg/{ControlCommand,MotionStatus,SegmentStatus,ChannelEvent}.msg
tools/ros_action_smoke.py            # 真正经 ROS Action/topic/service 的场景测试
docs/VALIDATION.md                   # 实际验证记录与边界
```

`task_manager` 只负责按 `sequence` 调度和报告任务；`segment_navigation` 使用同一个 `SegmentExecutor` 执行每个导航段；`sim_executor` 是可被真实适配器替换的执行端。规划计算在独立线程池运行，ROS 定时器继续处理定位、跟踪和取消。ROS 段节点使用多线程 Executor、可重入回调组及锁，核心方法由调用者串行访问。

任务管理器每次生成新的 `task_id`，每段生成新的 `execution_id`。Action 目标接收时即预留执行器，同一时间只允许一个段请求；忙碌时拒绝新请求，不抢占。取消后，等待旧目标终态再发新目标。客户端通过任务代次和执行 ID 过滤迟到反馈/结果；段执行器只消费当前规划 Future，迟到旧规划无法覆盖新路径。执行 ID 在该服务进程生命周期内不得复用。

## 2. 构建与启动

### 依赖

目标环境为 Ubuntu 22.04 / ROS 2 Humble / Python 3.10。包依赖已写入两个 `package.xml`；额外 Python 依赖是 PyYAML 和 pytest。使用已有 Humble 环境时可以运行：

```bash
source /opt/ros/humble/setup.bash
cd /home/syf/ros2_ws/src/stair_nav
rosdep install --from-paths stair_nav_interfaces stair_nav --ignore-src -r -y
```

`rosdep` 安装依赖需要系统安装权限及网络。本工程的核心测试无需启动 ROS 图。

### 在当前工程内构建（不改动相邻工程的构建目录）

```bash
source /opt/ros/humble/setup.bash
cd /home/syf/ros2_ws/src/stair_nav
colcon --log-base .colcon/log build \
  --base-paths stair_nav_interfaces stair_nav \
  --packages-select stair_nav_interfaces stair_nav \
  --build-base .colcon/build --install-base .colcon/install --symlink-install
source .colcon/install/setup.bash
ros2 launch stair_nav demo.launch.py
```

这条演示启动命令会启动三个节点及 RViz，自动执行 AB→BC→CD。无图形界面时使用：

```bash
ros2 launch stair_nav demo.launch.py rviz:=false
```

也可以使用常规工作区构建，两个方案选一个：

```bash
cd /home/syf/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select stair_nav_interfaces stair_nav --symlink-install
source install/setup.bash
ros2 launch stair_nav demo.launch.py
```

每个新终端都要 source 相同的安装空间。只有日志目录不可写时才需要设置，例如 `export ROS_LOG_DIR=/tmp/stair_nav_ros_logs`。同一 ROS Domain 内请只启动一套该工程；绝对 topic 名称使同域多套实例会共享消息。

### 手工开始、取消与单段调用

```bash
ros2 launch stair_nav demo.launch.py autostart:=false
```

在另一个已 source 的终端：

```bash
ros2 service call /task_manager/start std_srvs/srv/Trigger '{}'
ros2 topic echo /task/status --qos-durability transient_local
ros2 service call /task_manager/cancel std_srvs/srv/Trigger '{}'
```

段执行器也可单独被其他任务调度器调用；下面的执行 ID 只可使用一次，重试时换新 ID：

```bash
ros2 action send_goal /navigate_segment stair_nav_interfaces/action/NavigateSegment \
  '{task_id: manual-inspection-1, execution_id: manual-ab-attempt-1, segment_id: AB}' --feedback
```

任务完成或故障后不会把机器人复位到 A。再次请求整条路线时仍从实时定位接入 AB；若已在 D 或超出 AB 入口范围，会明确拒绝。重做合成测试请结束并重新启动演示进程，每个新的模拟进程才从配置的初始点开始。

### 无 ROS 闭环与回归测试

在工程根目录执行：

```bash
PYTHONPATH=stair_nav python3 -m stair_nav.demo --output /tmp/stair_nav_headless.json
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
```

演示调用生产使用的核心规划、跟踪、到达判断及执行状态机，通过候选速度积分得到定位。测试调度器同步完成规划，以获得确定的时间和结果；ROS 节点使用异步规划线程。头部 JSON 中 `success=true` 才表示整条路线成功；故障演示返回非零退出码。

构建后可运行 ROS 场景验证脚本，每个场景启动一套全新模拟节点，结果和节点日志保存到输出目录：

```bash
source .colcon/install/setup.bash
python3 tools/test_ros_callbacks.py
python3 tools/ros_action_smoke.py --output-dir /tmp/stair_nav_ros_smoke
```

`--cases immediate_cancel` 单独检查目标刚接受时取消；`--cases transport_timeout` 会中止脚本自己启动的段导航进程，检查任务通信超时与忙碌保护。它们均只操作测试自己的模拟节点。

## 3. 坐标、参考点与速度定义

所有路径和定位均使用路线 `frame`（示例为 `map`）。首版只接收 **`/localization/pose` 的 `geometry_msgs/PoseStamped`**，不直接查询 TF 或接收 Odometry。`header.stamp` 是观测时间，四元数须有限且接近单位长度；定位必须与路径使用同一个导航参考点。

首版 `reference_point: ground_projection` 表示由团队明确约定的、落在支撑表面上的导航参考点。示例路径的 z 和模拟定位的 z 都是该点的高度。它不是 `base_link` 原点，也不是身体中心高度。真实定位若输出传感器或机体位姿，应由输入适配层完成转换：先用标定外参和完整三维旋转变换到约定的机体参考点，再按支撑/地形或接触估计求对应地面参考点。刚性外参部分应使用 `p_ref = p_body + R_map_body * offset_body`；地面投影还需要重力方向和表面信息。楼梯姿态及支撑变化时不能盲目给 z 加减一个固定机身高度。

候选指令的含义固定为：

- `vx`：水平面内，沿机器人 yaw 对应前向的速度，单位 m/s；不是沿台阶斜坡的弧长速度。
- `wz`：绕重力方向的角速度，单位 rad/s；不是倾斜机体局部 z 轴的角速度。
- `ControlCommand.header.frame_id` 标识路线对齐坐标系；`vx` 的方向由机器人 yaw 决定，不是 map 的 x 分量。
- 路径 z 只参与三维匹配、楼层区分和到达判断；跟踪器不输出向上速度。

如果真实策略输入是沿坡速度、机体坐标速度或动作 latent，须由运控适配层按策略定义转换。例如在方向与坡面一致且无侧滑等前提成立时，沿坡速度与水平速度存在坡角余弦关系；这不替代对策略接口的标定。首版从不自行把高度差转成竖直控制量。

示例楼梯中不允许原地转向或倒退，纠偏也受 `allow_stair_correction` 与运控能力声明约束。平台段可通过 `allow_turn_in_place` 和 `turn_in_place` 能力允许终点转向。示例速度、前视、转向与加速度限制都是演示值，需根据真实策略响应、通信延迟、机器人尺寸和允许姿态重新标定。

## 4. 路线、参考路径与单平台栅格

主配置为 [stair_nav/config/demo_route.yaml](stair_nav/config/demo_route.yaml)。默认值 `defaults` 与每段配置递归合并，段配置覆盖默认值。所有相对文件路径均相对主路线 YAML 所在目录解析。

| 配置区域 | 主要字段与含义 |
|---|---|
| 全局 | `frame`、`reference_point`、显式 `sequence`、`control_period`、`command_ttl`、`task_transport_timeout` |
| `nodes.A/B/C/D` | 三维 `position`、`yaw`、`arrival` 到达区域及确认要求 |
| `arrival` | `xy` 为水平圆半径，`z` 为高度绝对误差，`yaw` 为朝向误差，`progress` 为三维弧长进度下限；单位为 m/rad；`confirm_seconds` 为持续确认时间 |
| 到达整机条件 | `require_exit`、`require_stable`、`required_mode`；`require_hold` 默认 true，可显式配置 |
| 每个 `segments.<id>` | `start/end`、`kind`、`platform_id`、`backend`、`path_file/grid_file`、`mode`、`allowed_modes`、`required_capabilities` |
| `tracking` | 前视、速度/转向上限、线/角加速度限制、接近减速、最大偏离/高度差、匹配窗口、转向/倒退/楼梯纠偏权限 |
| `planning` | 接入范围、最大路径点跳变、入口已核实标志、膨胀半径、未知区域策略、规划超时 |
| `timeouts` | `ready/execution/pose/status/arrival`，单位秒 |
| `regions` | 供 RViz 显示的平台/楼梯区域；不构成碰撞或可通行性证明 |
| `simulation` | 初始节点、剖面、确认延迟、模拟能力；真实导航启动不使用模拟节点 |

示例 A=(0,0,0)、B=(3,0,1)、C=(6,0,2)、D=(9,0,2)，朝向均为 0。AB/BC 的尾部各保留平台剖面；B/C 需要几何条件、出梯、稳定、HOLD 和 `flat` 模式全部确认后才成功。

### `preset_path`

[path 示例](stair_nav/config/flight_ab.yaml) 的格式是：

```yaml
frame: map
reference_point: ground_projection
points:
  - {position: [0.0, 0.0, 0.0], phase: entry, mode: stair}
  - {position: [0.5, 0.0, 0.0], phase: entry, mode: stair}
  - {position: [1.0, 0.0, 0.25], phase: stair, mode: stair}
  # ...
  - {position: [2.5, 0.0, 1.0], phase: exit, mode: stair}
  - {position: [3.0, 0.0, 1.0], phase: platform, mode: stair}
```

后端校验空路径、非有限数值、重复点、超限跳变、无法用水平速度跟踪的纯竖直边、终点与节点不一致以及框架/参考点不一致。阶段和模式保存在内部 `Waypoint` 中，当前阶段会出现在结构化状态和指令里；标准 `nav_msgs/Path` 仅承载几何路径。

每段都以当前实时位姿规划。预设路径只接受配置的入口邻域 `join_xy/join_z` 内的接入；非零偏移还需要 `entry_corridor_confirmed: true`，表示外部已经确认该小范围入口走廊允许接入。返回的参考路径保留原始几何，不会随意补一条直线。范围超限返回 `JOIN_OUT_OF_RANGE`，缺少可行性确认返回 `JOIN_UNVERIFIED`。示例的 true 只描述合成场景；真实项目需要现场几何、机体包络和策略能力验证后设置。

### `grid_astar`

[栅格示例](stair_nav/config/upper_platform.yaml) 为显式单平台、轴对齐的 YAML：

```yaml
frame: map
platform_id: upper_platform
height_z: 2.0
width: 42
height: 30
resolution: 0.1
origin: [5.5, -1.5]
data:  # height 行，每行 width 个值；第 0 行对应最小 y
  - [100, 100, 100]  # 此行仅示意，实际数量须与 width 一致
```

`origin` 是左下角网格边界，单元中心为 `origin + (index+0.5)*resolution`。占据值 `0..100`，未知为 `-1`；默认占据阈值 50，未知不可通行。规划按机器人/安全余量配置进行保守障碍膨胀，检查起终点、地图边界和平台高度，不允许对角线从障碍夹角穿过。默认不简化路径；启用 `planning.simplify` 后每条简化边仍必须通过膨胀地图的连续线段碰撞检查。最终返回路径也检查碰撞。

当前规划接口读取 YAML 文件，不直接订阅在线地图，也不直接读取点云。真实点云转换的接入口是生成同格式的**每平台栅格**：由建图成员明确提供平台 ID、坐标系、测量高度、有效范围、分辨率、占据与未知定义，再输出与导航节点一致的文件。楼梯继续使用单独核实的三维参考路径。不要把整栋建筑点云压到一张二维图，也不要在数据不足时猜楼层切片高度或净空参数。倾斜/非平的平台需要后续扩展模型。

## 5. 跟踪与到达判断

Pure Pursuit 在当前段的路径上进行三维线段投影，匹配评分包含高度，并限制历史进度附近的前后搜索窗口。进度按三维弧长维护且单调不减，避免同一 XY 的不同高度或回折路径被随意切换。前视点按路径弧长采样；横向误差以局部平面切线给出符号，朝向误差按 yaw 计算。

跟踪器根据参考点计算 `vx/wz`，限制速度、转向及变化率，在终点前减速，超出横向或高度阈值时返回明确错误。常规跟踪指令受变化率限制；取消、异常和进入确认阶段时直接撤销有效性，不用渐变的有效速度掩盖撤销。执行频率默认 20 Hz，默认候选指令 TTL 为 0.2 秒。

几何到达同时检查目标水平距离、高度、朝向和路径进度，之后进入 `ARRIVAL_CONFIRM`，此时状态只表示 `geometric_candidate`。仍需等待当前执行 ID 的出梯、稳定、模式和 HOLD 反馈，并持续满足配置的确认时间。缺失整机反馈会等待并最终报 `ARRIVAL_CONFIRM_TIMEOUT`，不会提前宣告 B/C 成功。确认中几何条件失效会重新进入执行；下一段重新使用实时定位，不假定恰好停在标注节点。

## 6. 状态机与停止语义

| 当前状态 | 触发/检查 | 下一状态或动作 |
|---|---|---|
| `IDLE` 或终态 | 新目标合法且空闲 | `CHECKING`，分配新执行 ID 上下文 |
| `CHECKING` | 新鲜、有限、同坐标系定位就绪 | 提交异步规划 → `PLANNING` |
| `CHECKING` | 无定位超过期限/定位无效 | `FAILED` |
| `PLANNING` | 当前规划完成，机器人仍在已核实规划起点范围 | `WAIT_READY` |
| `PLANNING` | 规划错误、超时或起点移动超限 | `FAILED`，忽略迟到规划结果 |
| `WAIT_READY` | 当前 ID 的新鲜反馈 ready、能力及模式满足 | `EXECUTING` |
| `WAIT_READY` | 就绪/模式未确认超过期限 | `FAILED/READY_TIMEOUT` |
| `EXECUTING` | 尚未到达 | 周期发布带 TTL 的有效 `TRACK` 指令 |
| `EXECUTING` | 几何到达候选 | `ARRIVAL_CONFIRM`，撤销速度并请求 HOLD/目标模式 |
| `ARRIVAL_CONFIRM` | 全部几何及整机条件持续满足 | `SUCCEEDED` |
| `ARRIVAL_CONFIRM` | 几何条件丢失 | `EXECUTING` |
| `ARRIVAL_CONFIRM` | 整机确认超时 | `FAILED/ARRIVAL_CONFIRM_TIMEOUT` |
| 任意活动状态 | 取消 | `CANCELED`，立即发布无效指令及 HOLD 请求 |
| 任意活动状态 | 定位过期、阻塞、时钟异常等 | `FAILED`，撤销指令 |
| 执行/确认中 | 反馈过期、故障、ready 丢失、超时/偏离等 | `FAILED`，撤销指令 |

任务管理器按配置串行发送三个 Action；只在最终 CD 的结果成功后发布整个任务 `SUCCEEDED`。段终态反馈在任务话题中显示为 `SEGMENT_SUCCEEDED/FAILED/CANCELED`，避免把第一段完成误读成整条路线完成。`progress` 在执行中是当前段进度，不是整条任务的距离比例。

`navigation_stopped=true` 只意味着导航已停止产生有效速度指令；`hold_confirmed=true` 才是适配器当前反馈的保持确认。Action 取消/失败可以在保持确认之前返回，结果中的两个字段必须分别读取；之后 `/navigation/status` 会继续反映当前反馈。零速度或无效指令本身都不能证明机器人已在楼梯上安全停稳。真实适配器收到 HOLD 或 TTL 过期后，需要执行与当前运动阶段相符的保持/退出策略，并依据真实传感器和策略状态报告结果。

任务 Action 接受/反馈失联超过 `task_transport_timeout` 后，任务进入 `TRANSPORT_TIMEOUT`，请求取消并保持 BUSY；没有终态确认时不接受新任务，也不把导航停止或整机保持标为已确认。执行端本身仍须独立实现 TTL 看门狗。恢复时应先核实运控实际状态，再处理节点/任务恢复。

## 7. ROS 接口、时间戳与 QoS

接口定义在 `stair_nav_interfaces` 包，所有数值使用 m、s、rad。以下 topic 使用绝对名称：

| 接口 | 类型 | QoS / 用途 |
|---|---|---|
| `/navigate_segment` | `NavigateSegment` Action | 请求含 task/execution/segment ID；反馈为 `SegmentStatus`；结果含成功、错误码、停止及保持确认；ROS Action 默认服务/反馈 QoS，结果保留 60 秒 |
| `/localization/pose` | `geometry_msgs/PoseStamped` | Reliable、Volatile、KeepLast(5)，观测时间与路线 frame |
| `/motion/status` | `MotionStatus` | Reliable、Volatile、KeepLast(5)，持续发布当前 ID、ready、模式、能力、出梯/稳定/保持及故障 |
| `/navigation/command` | `ControlCommand` | Reliable、Volatile、KeepLast(1)，候选速度/模式/阶段、valid、valid_until、TRACK/HOLD |
| `/navigation/channel_event` | `ChannelEvent` | Reliable、Volatile、KeepLast(5)，外部通道 blocked/clear 事件 |
| `/navigation/status` | `SegmentStatus` | Reliable、TransientLocal、KeepLast(1)，当前段执行快照，含误差、进度和错误 |
| `/task/status` | `SegmentStatus` | Reliable、TransientLocal、KeepLast(1)，当前任务及当前段状态 |
| `/navigation/reference_path` | `nav_msgs/Path` | Reliable、TransientLocal、KeepLast(1)，当前几何参考路径；新目标开始时先清空旧路径 |
| `/navigation/reference_point` | `geometry_msgs/PoseStamped` | Reliable、TransientLocal、KeepLast(1)，当前前视参考点，供可视化 |
| `/navigation/scene` | `visualization_msgs/MarkerArray` | Reliable、TransientLocal、KeepLast(1)，节点与平台/楼梯区域 |
| `/simulation/trajectory` | `nav_msgs/Path` | Reliable、TransientLocal、KeepLast(1)，模拟实际轨迹，最多 4000 个采样点 |
| `/simulation/platform_grid` | `nav_msgs/OccupancyGrid` | Reliable、TransientLocal、KeepLast(1)，合成平台原始栅格可视化 |
| `/tf` | `tf2_msgs/TFMessage` | 仅模拟端使用标准动态 TF QoS 广播 `map → simulation_nav_reference`；来自同一积分位姿，仅供显示/调试，不是首版定位输入 |
| `/task_manager/start`、`/task_manager/cancel` | `std_srvs/Trigger` | 显式任务启动/取消 |
| `/sim_executor/<fault>` | `std_srvs/SetBool` | 仅模拟端的故障开关 |

`SegmentStatus` 至少包含 `task_id/execution_id/segment_id/phase/state/progress/lateral_error/heading_error/error_code/error_message`，并给出几何候选、导航停止及 HOLD 确认。运动反馈须回显收到的当前 `execution_id/segment_id`；旧 ID 或同 ID 的乱序反馈不会覆盖最新状态。

时间统一使用 ROS 时钟。定位与运控反馈的时间戳必须是采集/观察时间，不能把历史数据收到时重新盖章掩盖过期。默认定位/运控新鲜度均为 0.5 秒，最多容忍 0.1 秒未来时间偏差；候选指令超过 `valid_until` 后失效。真实适配器必须校验帧、有限值、ID、时间范围、能力和速度限制，并保证节点断联时也会过期处理。模拟端已独立实现这些基础校验。首版默认墙钟运行，模拟 ROS 时钟跳变或多机时间同步需要实际联调。

晚启动 RViz/监控节点以匹配的 TransientLocal QoS 订阅状态、路径和场景，即可取得当前快照；状态节点还持续重复发布。控制与运动状态使用 Volatile，晚启动执行端不能回放旧有效命令，只等待新的带 TTL 命令并握手。`reference_point` 与路径是显示数据，终态时仍可能保留最后参考点，不能当作运动授权。

通道事件接收时要求 frame 一致且年龄不超过 1 秒，确认阻塞后在当前段节点内保持，直到显式 clear；事件源重启/导航节点重启后的状态重放由外部事件源负责，首版不持久化阻塞状态。

`ChannelEvent.segment_id` 为空或 `*` 时表示所有导航段；指定段 ID 时仅影响对应段。

## 8. 模拟、可视化与故障注入

模拟端只积分收到且通过校验、未过期的实际 `vx/wz`；不把 XY/yaw 推到目标点。高度由当前段的已配置合成剖面，按积分后的 XY 和历史位置采样，平台使用显式高度。剖面是地形约束模型，不能用于证明真实定位高度误差、台阶接触、打滑、爬楼动力学或保持安全性。

模拟 ready、模式、出梯、稳定和延迟 HOLD 确认仅用于软件协议联调。`missing_exit` 让几何到达仍无法成功，`unstable` 则阻断稳定确认。退出确认使用配置的剖面末端距离，不代表真实机体包络已出梯。

RViz 中显示绿色任务节点、半透明平台/楼梯区域、青色当前参考路径、黄色实际轨迹、红色参考点、绿色实时定位，以及二层平台占据栅格。Fixed Frame 为 `map`；若使用其他路线 frame，需相应修改 RViz 配置。上层栅格的高度来自 `height_z`，不是把所有楼层合成一张图。

故障开关可通过 SetBool 随时修改，false 表示恢复该输入；故障导致任务终态后不会自动续跑：

```bash
# 未就绪：应在手工 start 之前设置，等待 READY_TIMEOUT。
ros2 service call /sim_executor/not_ready std_srvs/srv/SetBool '{data: true}'
# 执行过程中中断定位：超过定位新鲜度后撤销有效指令。
ros2 service call /sim_executor/localization_dropout std_srvs/srv/SetBool '{data: true}'
# 运控执行异常：立即由结构化故障反馈触发失败。
ros2 service call /sim_executor/execution_fault std_srvs/srv/SetBool '{data: true}'
# 到达几何候选仍缺少出梯确认。
ros2 service call /sim_executor/missing_exit std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim_executor/unstable std_srvs/srv/SetBool '{data: true}'
```

阻塞事件需要**当前时间戳**。以下脚本等待订阅者发现后发布 AB 阻塞；解封时将 `blocked=True` 改成 `False`，重新运行：

```bash
python3 - <<'PY'
import time
import rclpy
from stair_nav_interfaces.msg import ChannelEvent
from stair_nav.ros_utils import live_qos
rclpy.init()
node = rclpy.create_node('inject_channel_event')
pub = node.create_publisher(ChannelEvent, '/navigation/channel_event', live_qos(5))
deadline = time.monotonic() + 3.0
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
msg = ChannelEvent(segment_id='AB', blocked=True, reason='test channel closed')
msg.header.frame_id = 'map'
msg.header.stamp = node.get_clock().now().to_msg()
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

也可无 ROS 快速复现故障：

```bash
PYTHONPATH=stair_nav python3 -m stair_nav.demo --fault not_ready
PYTHONPATH=stair_nav python3 -m stair_nav.demo --fault localization_dropout --fault-at 2
PYTHONPATH=stair_nav python3 -m stair_nav.demo --fault missing_exit
PYTHONPATH=stair_nav python3 -m stair_nav.demo --cancel-at 2
```

这些命令预期返回失败/取消及非零退出码；断言式验证使用 pytest 或 ROS 场景脚本。

## 9. 接入真实地图、定位与运控

真实系统采用独立启动文件，必须传入已经核实的路线文件；它不启动模拟端，也不自动发任务：

```bash
ros2 launch stair_nav navigation.launch.py route_file:=/absolute/path/site_route.yaml
```

接入顺序建议如下：

1. **建图接口**：提供 map 坐标定义和每个平台的有效栅格，单独标注每段楼梯入口/出口/平台、模式与可接入邻域。核实机体包络、楼梯净空及路线可通行性；保留未知区域。
2. **定位适配器**：把已有 TF/Odometry/Pose 转成约定 `map` 中的同参考点 `PoseStamped`，保留真实时间戳，处理三维姿态、外参和地面参考，不由导航盲目加高度。
3. **运控适配器**：订阅 `ControlCommand`，回显当前执行 ID，转换水平 yaw 速度至策略接口；独立检查 TTL/能力/限幅；对 HOLD、失联和故障选择真实保持或退出策略。持续发布 `MotionStatus`，只有真实条件满足才设置出梯、稳定和保持标志。
4. **外部事件源**：把通道封锁、人工禁行等转为带时间戳的 `ChannelEvent`，并负责状态恢复与重放。
5. **联调确认**：先验证停止/保持反馈、时钟与模式握手，再标定速度、前视、变化率、误差及超时，最后在受控条件下运行完整任务。

首版没有建图、SLAM、楼梯识别、RL 训练、真实 SDK、在线重规划、在线局部避障、完整机体包络碰撞/动力学验证、自动楼层切片或断电恢复。静态 A* 路径碰撞检查与配置入口声明不能代替真实机器人整机安全评估。实际已验证内容和剩余联调项以 [验证记录](docs/VALIDATION.md) 为准。
