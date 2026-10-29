# longjing-robot

面向龙净干式超净装置的机器人自主巡检项目，涵盖建图定位、楼梯导航、仿真验证与实机部署。

当前实现面向 ROS 2 Humble：任务状态机串行调度导航段，段状态机完成规划、定位闭环跟踪和到达确认。算法核心不依赖 ROS、Nav2 或行为树。演示场景与正式导航代码分开，保留 `stair_nav`、`stair_nav_interfaces` 两个 ROS 包。

## 目录与职责

```text
stair_nav/                              # ament_python 包
  stair_nav/
    core/
      models.py                         # 位姿、路径、指令、反馈
      config.py                         # 路线加载与校验
      planning.py                       # 预设路径、平台 A*、高度栅格 A*
      tracking.py                       # 三维路径匹配与 Pure Pursuit
      engine.py                         # 段执行状态机
      mission.py                        # 任务调度、取消与故障恢复
    segment_node.py                     # /navigate_segment Action 服务端
    task_node.py                        # /execute_mission 与前端状态
    ros_utils.py                        # ROS 消息与 QoS 转换
    demo/
      __init__.py、__main__.py
      headless.py、simulation.py         # 无 ROS 闭环、合成运控反馈
      sim_node.py、generate.py           # ROS 模拟端、地图生成器
      config/
        route.yaml、mission.yaml
        maps/{approach,ab,bc,cd}.yaml
      launch/demo.launch.py
      rviz/demo.rviz
  launch/navigation.launch.py            # 正式导航入口，显式提供配置
  test/
    fixtures/legacy/                     # 旧场景，只用于回归，不安装
    ros/                                # ROS 回调、可视化与场景检查
stair_nav_interfaces/                    # msg/action/srv 接口包
```

`task_manager` 负责任务预检、逐段派发、状态上报和取消清理，不发布速度；`segment_navigation` 负责规划、跟踪、速度输出和段完成判断；demo 的 `sim_executor` 积分实际指令并提供模拟定位与运控反馈。真实部署由机器人适配器承担模拟端的接口职责。

## 构建与启动 demo

环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。依赖写在两个包的 `package.xml` 中；核心还使用 PyYAML、pytest。首次安装依赖可在仓库根目录运行 `rosdep install --from-paths stair_nav_interfaces stair_nav --ignore-src -r -y`。

如果已构建过旧目录版本，迁移时先在工作区根目录执行一次 `rm -rf build/stair_nav install/stair_nav`，清除该包的旧文件链接。这两个目录是可重新生成的构建、安装产物；源码目录不受影响。

在 ROS 工作区根目录构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/syf/ros2_ws
colcon build --packages-up-to stair_nav --symlink-install
source install/setup.bash
ros2 launch stair_nav demo.launch.py
```

该命令启动任务节点、段导航节点、模拟节点和 RViz，默认 `autostart:=true`，自动执行完整流程。每个新终端都需 source 同一工作区的 `install/setup.bash`；同一个 ROS Domain 内只启动一套 demo。

统一在 `/home/syf/ros2_ws` 构建，不在仓库目录 `/home/syf/ros2_ws/src/stair_nav` 内另建安装空间。如果 `ros2 pkg prefix stair_nav` 指向 `src/stair_nav/install/stair_nav`，先备份移走仓库内误生成的 `build/install/log`，再新开终端 source 工作区安装。正确输出为 `/home/syf/ros2_ws/install/stair_nav`；反复 source 不保证调整已加载安装空间的优先级。

本次移动了 Python 入口和配置、launch、RViz 资源，**需要重新构建安装**。后续使用 `--symlink-install` 时，修改 Python 或已有链接资源通常只需重启节点；新增资源、修改安装列表或接口定义后需重新构建。

无图形界面时：

```bash
ros2 launch stair_nav demo.launch.py rviz:=false
```

手动触发时先启动：

```bash
ros2 launch stair_nav demo.launch.py autostart:=false
```

再在另一个已 source 的终端中，任选一种启动方式：

```bash
# 方式一：启动当前已加载的任务。
ros2 service call /task_manager/start std_srvs/srv/Trigger '{}'
```

```bash
# 方式二：通过 Action 启动并查看反馈。
ros2 action send_goal /execute_mission stair_nav_interfaces/action/ExecuteMission \
  '{route_id: inspection_demo}' --feedback
```

`route_id` 校验当前已加载任务的标识，不会根据名称切换配置文件。忙碌或故障阻塞时拒绝新任务；每次启动分配新的 `task_id`，每段分配新的 `execution_id`。

## 演示路线与地图

默认模拟位置为 **O=(0,0,0)**，执行 **O→A→B→C→D**，全程上楼或走平台，没有下楼段。

| 段 | 路线与高度 | 规划方式 |
|---|---|---|
| O→A | O→(-4,-0.5,0)，约 4m | 平台 `grid_astar` |
| AB | 向北 4m，到 B=(-4,3.5,1.2) | 楼梯 `height_grid_astar` |
| BC | 向东 6m，到 C=(2,3.5,1.2) | 平台 `grid_astar` |
| CD | 向北 3m，到 D=(2,6.5,2.4) | 楼梯 `height_grid_astar` |

接近 A 时任务状态为 `APPROACHING_START`，段 ID 为 `APPROACH_A`，不计入三段主路线数量。若启动时已经处于 A 的位置、高度和朝向容差内，则跳过接近段。每段重新使用实时定位规划，楼梯沿中轴直行，在平台上完成转角；任务结束不自动返回 O。

AB、CD 各有六级台阶，每级上升 0.2m，栅格分辨率为 0.1m。平台地图包含占据矩阵 `data`；楼梯还包含逐格高度 `heights` 和阶段 `phases`。高度栅格 A* 限制相邻格的步高并返回 XYZ 路径。这是每格一个支撑高度的 **2.5D 合成地图**，不直接读取点云，也不表示完整三维机体碰撞或楼梯动力学。

默认六份配置集中在 [demo/config](stair_nav/stair_nav/demo/config)：

| 文件 | 修改内容 |
|---|---|
| [route.yaml](stair_nav/stair_nav/demo/config/route.yaml) | O/A/B/C/D、各段后端、速度、到达容差、模拟初始位置 |
| [mission.yaml](stair_nav/stair_nav/demo/config/mission.yaml) | `route_id`、主段顺序、接近段、任务超时与就绪条件 |
| [maps/approach.yaml](stair_nav/stair_nav/demo/config/maps/approach.yaml) | O→A 平台范围与占据 |
| [maps/ab.yaml](stair_nav/stair_nav/demo/config/maps/ab.yaml) | 第一段楼梯的占据、高度与阶段 |
| [maps/bc.yaml](stair_nav/stair_nav/demo/config/maps/bc.yaml) | 中间平台通道 |
| [maps/cd.yaml](stair_nav/stair_nav/demo/config/maps/cd.yaml) | 第二段楼梯的占据、高度与阶段 |

从仓库根目录生成一套自定义尺寸配置，再同时传入路线与任务文件：

```bash
PYTHONPATH=stair_nav python3 -m stair_nav.demo.generate --out /tmp/custom_demo
PYTHONPATH=stair_nav python3 -m stair_nav.demo.generate --help
ros2 launch stair_nav demo.launch.py \
  route_file:=/tmp/custom_demo/route.yaml mission_file:=/tmp/custom_demo/mission.yaml
```

生成器支持楼梯/通道长度、高差、台阶数量和分辨率。也可用 `initial_pose:='[x, y, z, yaw]'` 覆盖初始位姿，位置须落在接近地图可通行范围内；空数组使用路线中的默认初始位置。

## 状态、取消与 RViz

任务状态包括 `IDLE`、`VALIDATING`、`WAITING_READY`、`DISPATCHING`、`APPROACHING_START`、`RUNNING`、`CANCELING`，以及终态 `SUCCEEDED`、`FAILED`、`CANCELED`、`FAULT_BLOCKED`。

每段依次经过 `CHECKING → PLANNING → WAIT_READY → EXECUTING → ARRIVAL_CONFIRM → SUCCEEDED`；异常或取消进入相应终态。几何到达检查 XY、高度、朝向与路径进度，随后还需当前执行的保持、稳定、模式及所配置的出梯确认持续成立，才允许下一段开始。

前端订阅 `/mission/status` 获取任务/当前段快照，订阅 `/mission/events` 获取逐次状态变化及当时快照；段状态另见 `/navigation/status`。状态与显示数据使用 Reliable、TransientLocal QoS，晚启动订阅者可取得最近快照。

```bash
ros2 topic echo /mission/status --qos-durability transient_local
ros2 service call /task_manager/cancel std_srvs/srv/Trigger '{}'
```

`CANCELING` 表示停止派发并等待下游目标终态、保持、稳定和控制释放；`CANCELED` 表示取消完成；`FAILED` 表示失败且清理完成或无需清理。接受/释放状态不明或清理超时则进入 `FAULT_BLOCKED`，拒绝新任务；核实下游执行与真实保持状态后，才可通过 `/task_manager/reset` 请求守卫复位。Action 客户端优先使用目标句柄取消；若通过上述兼容服务取消 Action 任务，业务结果为 `CANCELED`，ROS 协议终态为 `ABORTED`。

RViz 显示橙色楼梯、蓝色平台、青色参考路径、黄色实际轨迹、红色前视点和绿色定位。深红占据边界块仅在显示中隐藏，规划地图仍保留占据边界；`demo.rviz` 是视图配置，地图来自上述 YAML。

默认选择 `Move Camera` 和 `Orbit` 三维透视视角：左键拖动旋转，中键或 `Shift+左键` 拖动平移，滚轮或右键拖动缩放，`Shift+右键` 拖动升降。`Views` 面板可修改 Pitch、Yaw、Distance、Focal Point；按 `M` 可切回相机工具。

`/navigation/reference_point` 的位置是当前前视目标，朝向是跟踪器期望的平面朝向，到点对齐时为目标朝向；终态可能保留最后参考点，不能据此判断仍在运动。

## 正式导航与接口契约

正式入口与 demo 分开，必须显式提供已核实的路线和任务文件；它不启动模拟端或 RViz，也不自动开始任务：

```bash
ros2 launch stair_nav navigation.launch.py \
  route_file:=/absolute/path/site_route.yaml \
  mission_file:=/absolute/path/site_mission.yaml
```

真实定位和运控适配器需另行接入：

| 接口 | 类型与约定 |
|---|---|
| `/localization/pose` | `geometry_msgs/PoseStamped`；同一路线 frame、真实观测时间、单位四元数 |
| `/navigation/command` | `ControlCommand`；ID、模式、阶段、`vx/wz`、`TRACK/HOLD`、`valid/valid_until` |
| `/motion/status` | `MotionStatus`；回显当前 `execution_id/segment_id`，持续提供 ready、模式、能力、故障、出梯/稳定/保持确认 |
| `/navigate_segment` | `NavigateSegment` Action；任务层用于派发单段，不与活动任务并行争用 |
| `/navigation/get_state` | 下游忙碌、控制释放及当前段快照 |

定位与路径共同使用 `reference_point: ground_projection`，表示约定的支撑表面参考点。机体或传感器位姿应由输入适配层结合三维外参、重力方向和表面信息转换；楼梯上不能简单减去固定机身高度。

`vx` 是水平面内沿机器人 yaw 前向的速度（m/s），`wz` 是绕全局重力方向的 yaw 角速度（rad/s）；`vx` 不是 map 的 X 分量或沿坡速度，`wz` 不是倾斜机体局部 Z 轴角速度。路径 Z 用于匹配与到达判断，导航不输出竖直速度。

默认 20Hz 发布指令，TTL 为 0.2s；接收端仍须独立校验有效期并处理断联。`HOLD` 撤销有效导航速度，请求与当前阶段相符的保持/退出策略；`navigation_stopped=true` 仅表示没有有效导航速度，`hold_confirmed=true` 才是运控反馈的保持确认。零速度不能替代真实停稳、出梯或安全保持判断。定位和反馈使用观测时间，不得重盖时间戳掩盖陈旧数据。

当前无真实机器人 SDK、在线局部避障、完整机体碰撞/动力学验证或点云建图集成。合成地图与模拟确认只用于软件联调，真实路线、净空、模式能力与控制参数需现场核实。

## 检查与回归

在仓库根目录运行无 ROS 闭环与核心测试：

```bash
cd /home/syf/ros2_ws/src/stair_nav
PYTHONPATH=stair_nav python3 -m stair_nav.demo --output /tmp/stair_nav_headless.json
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
```

构建并 source 工作区后，可运行 ROS 消息/回调检查及真实 Action 场景。场景脚本自行启动模拟节点，默认使用独立 ROS Domain；输出日志与 JSON 放在 `/tmp`：

```bash
source /home/syf/ros2_ws/install/setup.bash
python3 stair_nav/test/ros/check_callbacks.py
python3 stair_nav/test/ros/check_visualization.py
python3 stair_nav/test/ros/ros_action_smoke.py --output-dir /tmp/stair_nav_ros_smoke
python3 stair_nav/test/ros/mission_smoke.py --output-dir /tmp/stair_nav_mission_smoke
```

旧直线配置仅保留在 `test/fixtures/legacy` 供回归，运行默认 demo 使用新的 O→A→B→C→D 场景。`docs/` 与运行产物不纳入 Git；以上说明与命令不依赖本地历史文档。
