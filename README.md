# stair_nav：机器人巡检状态机与规划层

本项目用于按指定路线执行分段巡检：任务状态机依次派发导航段，段状态机完成路径规划、定位闭环跟踪、到达保持和结果上报。支持平台栅格 A*、楼梯高度栅格 A* 和预设路径。

本文介绍从编译、运行 demo 到真实机器人部署的操作方法。实机部分以**外部 NVIDIA Orin + 云深处 M20 PRO** 为例；其他机器人可以按文末接口约定编写适配器。

**当前已提供软件模拟闭环和 M20 原厂 ROS 接口适配，尚未连接实机完成验证。** 本项目不负责点云建图，不包含在线局部避障、完整机体碰撞或楼梯动力学验证。已有地图仍需整理为项目配置；实机楼梯还需要支撑参考定位和保持/出梯监督输入。

## 1. 先了解运行组成

```text
前端 / 命令行：发送一次任务启动请求
  → task_manager：检查就绪条件、依次派发导航段
    → segment_navigation：规划路径、根据定位生成运动指令
      → 运动适配器：调用机器人原厂接口，返回真实运动与保持状态

机器人定位 → 定位适配器 → /localization/pose → 两个状态机
```

“适配器”是本项目与机器人原厂接口之间的转换程序。它负责消息转换、指令有效期检查和反馈同步；机器人原厂控制器仍负责实际运动。

| 启动入口 | 包含内容 | 是否自动开始任务 |
|---|---|---|
| `demo.launch.py` | 两个状态机、模拟定位/运控、可选 RViz | 默认是 |
| `m20_orin.launch.py` | 两个状态机、M20 定位与运控适配器 | 否；默认只读、运控适配器未启用 |
| `navigation.launch.py` | 仅两个状态机，供其他机器人接入 | 否 |

demo 和实机不能在同一个 ROS Domain 内同时发布同名定位、运控反馈。以下按“隔离 demo → 实机只读 → 平地单段 → 平地多段 → 楼梯 → 全路线”逐步验证。

## 2. 环境准备与编译

### 2.1 环境与目录

当前迁移目标为 **Ubuntu 20.04、ROS 2 Galactic、Python 3.8**，同时保留 Humble 使用的公共接口。代码已针对 Python 3.8 和 Galactic 的 ROS API 做兼容处理；**Galactic 目标机的实际构建、运行验收仍需按第 9 节完成**，不能将此前 Humble 下的软件验证等同于 Galactic 或实机验证。

下面假设 Galactic 已安装，使用独立工作空间 `~/galactic_ws`，仓库放在：

```text
~/galactic_ws/
  src/
    stair_nav/                 # 本仓库
      README.md
      stair_nav/               # Python 导航包
      stair_nav_interfaces/    # 消息、Action、服务接口包
```

使用其他路径时，替换命令中的工作空间路径。**在工作空间根目录编译，不在仓库内另建 build/install。** 从 Humble 迁移时，将源码放入新的 Galactic 工作空间，不复制旧 `build/install/log`，也不要在同一终端混合加载两个 ROS 发行版。Orin 还需核对其 Ubuntu/JetPack 版本及厂家对 Galactic 与当前固件组合的支持。

### 2.2 安装依赖

安装构建和依赖管理工具：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

如果这台主机从未初始化过 rosdep，先执行一次 `sudo rosdep init`。然后安装项目声明的依赖：

```bash
source /opt/ros/galactic/setup.bash
rosdep update --include-eol-distros
cd ~/galactic_ws
rosdep install --from-paths src/stair_nav/stair_nav_interfaces src/stair_nav/stair_nav \
  --ignore-src -r -y --rosdistro galactic
```

基础 demo 不需要 M20 消息包。**实机还需单独取得并构建与固件匹配的 `drdds` 包，rosdep 不会替你取得厂商消息定义**，详见第 5 节。PyYAML 是运行依赖，pytest 是测试依赖。

Galactic 已结束官方维护，rosdep 更新时需包含历史发行版；ROS 本体及依赖的安装源也需可用。不要通过混入 Humble 安装包解决 Galactic 依赖缺失。

### 2.3 编译并加载工作空间

```bash
cd ~/galactic_ws
colcon build --packages-up-to stair_nav --symlink-install
source install/setup.bash
ros2 pkg prefix stair_nav
```

`--packages-up-to stair_nav` 会同时构建依赖的 `stair_nav_interfaces`。最后一条命令应指向当前工作空间的安装目录。

**每个新终端都需要加载环境：**

```bash
source /opt/ros/galactic/setup.bash
# 实机若使用独立厂商工作空间，在此 source 其 install/setup.bash。
source ~/galactic_ws/install/setup.bash
```

新增消息、launch 或配置资源后需要重新构建；修改已链接的 Python 文件后通常重启节点即可。旧目录布局升级后若仍加载旧文件，检查 `ros2 pkg prefix stair_nav`，新开终端加载正确安装空间，必要时重新生成该包的构建与安装产物。

### 2.4 本次迁移的兼容范围

- 可空类型注解使用 Python 3.8 支持的 `Optional`；规划线程池关闭不使用 3.9 才加入的 `cancel_futures`，仍保留排队任务取消和运行中规划的协作取消。
- 中文客户端采用兼容的 ROS 上下文和执行器管理，避免依赖 Galactic 缺少的信号配置接口；第一次 Ctrl+C 仍请求取消本次目标并等待结果，不能直接关闭通信上下文来代替取消。
- `msg/action/srv` 字段、任务 ID、控制指令有效期及保持/释放确认约定不变。接口包和厂商消息包都需在 Galactic 下重新生成和构建。
- ROS 的 Action 接收/取消、反馈顺序、TF 晚到、定位失效和故障清理仍需在目标机回归；兼容修改不生成原厂消息、支撑参考定位或楼梯出梯证据。

继续使用 Humble 时，请使用独立的 Humble 工作空间和对应 ROS 环境，不加载本节 Galactic 的安装空间。

## 3. 使用 demo 验证软件整流程

demo 使用合成地图和模拟执行器，不连接机器人，也不验证真实步态和制动能力。

### 3.1 自动运行

在已加载工作空间的终端中，用独立 Domain 和本机通信隔离实机：

```bash
export ROS_DOMAIN_ID=190
export ROS_LOCALHOST_ONLY=1
ros2 launch stair_nav demo.launch.py
```

默认启动 RViz，并自动执行一次任务。无桌面环境的 Orin 可关闭 RViz：

```bash
ros2 launch stair_nav demo.launch.py rviz:=false
```

上述两种命令任选一种，同一 Domain 内只运行一套 demo。查看或控制 demo 的其他终端也需设置相同的 `ROS_DOMAIN_ID=190` 和 `ROS_LOCALHOST_ONLY=1`。

默认路线为 **O→A→B→C→D**：

| 段 | 场景 | 规划方式 |
|---|---|---|
| O→A | 接近首个巡检点的平台 | `grid_astar` |
| AB | 第一段楼梯，高差 1.2 m | `height_grid_astar` |
| BC | 同层平台通道 | `grid_astar` |
| CD | 第二段楼梯，再升高 1.2 m | `height_grid_astar` |

O→A 为接近段，不计入三段主路线数量；已在 A 点的位置和朝向容差内时会跳过。demo 没有下楼段，任务结束不自动返回起点。

RViz 中可查看场景、参考路径、实际轨迹、定位和前视目标。鼠标左键旋转、中键平移、滚轮缩放；显示配置位于 [demo.rviz](stair_nav/stair_nav/demo/rviz/demo.rviz)。

### 3.2 手动启动与查看结果

希望模拟前端启动流程时，先运行：

```bash
ros2 launch stair_nav demo.launch.py autostart:=false rviz:=false
```

在另一个已加载环境、使用相同 Domain 的终端中，发送**一次**任务请求：

```bash
ros2 action send_goal /execute_mission stair_nav_interfaces/action/ExecuteMission \
  '{route_id: ""}' --feedback
```

空 `route_id` 表示使用当前已加载任务，也可填写 demo 的 `inspection_demo`。该字段只校验任务标识，**不会按名称加载其他配置**。不要同时调用其他启动接口。

`accepted` 只表示任务被接收。正常执行会经过检查、等待就绪、派发和运行，最终应返回 `success: true`、`state: SUCCEEDED`。也可以订阅任务快照：

```bash
ros2 topic echo /mission/status --qos-durability transient_local
```

### 3.3 使用中文任务客户端

希望终端显示中文摘要时，可用 `mission_client` 替代上面的 `ros2 action send_goal`。它会实际发送一次相同的任务请求，两种启动方式任选一种。先确保 demo 使用 `autostart:=false`，或实机已经按第 5 节准备就绪。

新增入口首次使用前，在工作空间根目录重新构建并加载环境：

```bash
cd ~/galactic_ws
colcon build --packages-up-to stair_nav --symlink-install
source install/setup.bash
ros2 run stair_nav mission_client --route-id inspection_demo
```

实机可省略 `--route-id` 使用服务端已加载任务；填写标识不会切换路线文件。各终端仍需使用相同的 ROS Domain，模拟测试保持本机隔离。

客户端会显示任务受理、任务 ID、当前段、规划/跟踪/到达确认、完成段数、错误和最终结果。示例：

```text
[任务受理] 请求已接受，仍需通过就绪检查；目标 ID：…
[任务状态] 等待定位、运控和控制释放就绪（WAITING_READY）
[当前段] AB，第 1/3 个主路段，执行 ID：…
[段状态] 路径跟踪（EXECUTING），阶段：楼梯（stair）
[本段路径进度] AB：42.0%
[段状态] 几何到达，等待保持及配置要求的确认（ARRIVAL_CONFIRM）
[任务完成] 巡检成功，已完成 3/3 个主路段
```

任务 `RUNNING` 显示为“导航段已受理”，只有段 `EXECUTING` 才显示“路径跟踪”。本段路径进度不是整条任务的百分比。英文状态码、错误码和原始说明保留，已有 Action、消息定义和云端接口不变。

状态、路段和错误变化立即显示；同一执行段的进度默认最多每秒输出一次，相同进度不刷屏。可调整：

```bash
ros2 run stair_nav mission_client --route-id inspection_demo --progress-interval 2
ros2 run stair_nav mission_client --help
```

按一次 **Ctrl+C** 会通过本次 Action 目标请求取消，并等待最终结果；即使接受回执尚未到达，也会在收到后取消对应目标，不会重发启动请求。取消受理不等于已经停稳。默认等待 15 秒，可通过 `--cancel-timeout` 调整；等待超时会明确报告结果未确认。再次按 Ctrl+C 仅退出客户端，不能据此认定任务已停止。

`--action` 可指定其他任务 Action 名称，`--server-timeout` 控制等待服务的时间。退出码为：0 表示成功且停止/保持确认齐全，130 表示任务取消或用户中断，1 表示失败或结果未确认，2 表示命令参数错误。末尾的停止/保持信息是任务结果记录，实时运控仍需查看 `/motion/status`。

### 3.4 修改演示场景

演示配置位于 [demo/config](stair_nav/stair_nav/demo/config)：

| 文件 | 内容 |
|---|---|
| [route.yaml](stair_nav/stair_nav/demo/config/route.yaml) | 节点、各段后端、跟踪参数、到达容差 |
| [mission.yaml](stair_nav/stair_nav/demo/config/mission.yaml) | 主段顺序、接近段、就绪与超时条件 |
| `maps/*.yaml` | 平台占据栅格、楼梯高度及阶段 |

加载已编译的工作空间后，可以生成另一套**合成场景**：

```bash
python3 -m stair_nav.demo.generate --help
python3 -m stair_nav.demo.generate --out /tmp/stair_nav_custom_demo
ros2 launch stair_nav demo.launch.py \
  route_file:=/tmp/stair_nav_custom_demo/route.yaml \
  mission_file:=/tmp/stair_nav_custom_demo/mission.yaml
```

生成器支持楼梯/通道长度、高差、台阶数量和分辨率，不是实测地图转换器。`initial_pose:='[x, y, z, yaw]'` 只修改模拟初始位姿，不能用于设置真实定位。

## 4. 准备现场地图、路线和任务

实机入口必须提供现场文件，没有可直接用于机器人的默认路线。建议将现场配置独立保存，例如：

仓库已创建 [stair_site/](stair_site) 现场工作目录，已归档 `~/map_2d` 的原始地图并转换到 `maps/floor_1.yaml`，路线默认引用该栅格。坐标系、平台支撑高度、节点、路段和任务序列等待确认项仍为空。请先按其中说明补齐实测信息，再复制到下述部署目录；当前配置不能直接运行。

```text
~/stair_site/
  route.yaml
  mission.yaml
  m20_orin.yaml
  maps/                       # 转换后的平台/楼梯地图
  paths/                      # 使用预设路径时保存路径文件
```

### 4.1 已有地图如何使用

当前**不直接读取点云 PCD，也不直接读取 Nav2 的 image + PGM/PNG 地图，不订阅 /map 作为规划输入**。需要转换为项目栅格格式，或配置经过现场核实的预设路径；仓库尚未提供现成地图转换器。

| 后端 | 适用场景 | 输入要求 |
|---|---|---|
| `grid_astar` | 同一平台的平地规划 | 项目占据栅格，单一平台高度 |
| `height_grid_astar` | 已明确支撑表面的楼梯规划 | 占据栅格、逐格高度 `heights`，可选阶段 `phases` |
| `preset_path` | 沿已核实的固定路线跟踪 | 含 XYZ、阶段和模式的路径点 |

平台栅格的格式示意如下，**数值仅用于说明结构，不能作为现场地图**：

```yaml
frame: map
platform_id: floor_1
height_z: 0.0
width: 3
height: 2
resolution: 0.1
origin: [0.0, 0.0]
data:
  - [0, 0, 100]
  - [0, -1, 100]
```

`data[y][x]` 表示栅格，示例中 0 为空闲、100 为占据、-1 为未知；默认将未知区作为障碍。图像转换须核对 Y 方向、原点、分辨率和占据判定。当前 `origin` 仅包含 XY，源地图有旋转时需对齐或重采样。

楼梯高度栅格是每个 XY 位置一个支撑高度的 **2.5D 地图**，不是完整三维碰撞模型；重叠楼层需分段使用不同地图。`preset_path` 不执行栅格碰撞校验，使用前需核实路径和入口连接通道的净空。

预设路径的实时起点还需满足 `planning.join_xy/join_z`。与首路径点存在非零位置偏移时，须先核实局部连接通道，再显式设置该段的 `planning.entry_corridor_confirmed: true`；否则会返回 `JOIN_UNVERIFIED`，不能仅为通过检查而置位。

### 4.2 路线与任务如何配置

可参考 demo 的文件结构，逐项替换为实测信息：

| 文件 | 必须核对的内容 |
|---|---|
| `route.yaml` | `frame`、`reference_point: ground_projection`、节点 XYZ/yaw、到达容差、各段起终点、后端、地图/路径文件、模式、能力和跟踪参数 |
| `mission.yaml` | `route_id`、指向同一现场路线的 `route_file`、`sequence`、可选 `approach_segment`、就绪与超时条件 |
| `m20_orin.yaml` | 原厂话题、支持模式、已验证能力、速度上限和反馈超时 |

路线中的相对地图/路径文件名相对 `route.yaml` 所在目录解析；任务中的相对 `route_file` 相对 `mission.yaml` 所在目录解析。路线、地图、定位必须使用一致坐标系和支撑高度基准。

**第一次实机测试只配置平地直线 A→B：**

1. 将主序列改为这一段，并使用真实地图坐标。
2. 没有合格接近段时，从任务配置删除 `approach_segment`，人工将机器人放在 A 点的位置、高度和朝向容差内。接近段仅支持平台 `grid_astar`。
3. 平地使用 `flat`，关闭未使用的楼梯修正等选项；原地转向等能力只在实测确认后声明。
4. 按机器人和负载尺寸、定位误差设置障碍膨胀与通道净空，不沿用 demo 的膨胀半径。
5. 让路线速度上限与适配器一致，并根据实际响应标定最小速度、减速距离和到达容差。

demo 的坐标、严格朝向容差、速度和模拟确认均不是 M20 实机参数。模板默认只支持 `flat`、能力列表为空；直接套用 demo 的楼梯或原地转向要求会导致预检不通过。

## 5. 在外部 Orin 上接入 M20 PRO

### 5.1 安装固件匹配的消息包

从厂家取得这台机器人对应的 `drdds` 消息包，按厂商说明构建。若其源码放在同一工作空间的 `src` 下，可显式构建：

```bash
cd ~/galactic_ws
colcon build --packages-up-to drdds stair_nav --symlink-install
source install/setup.bash
ros2 interface show drdds/msg/NavCmd
ros2 interface show drdds/msg/MotionInfo
ros2 interface show drdds/msg/LocationStatus
```

若使用独立厂商工作空间，每个终端先 source 厂商安装空间，再 source 本项目。M20 节点运行时加载消息类型；基础包编译成功不代表厂商消息已安装。

已核对的公开消息版本没有 `LocationStatus`，不能仅下载公开仓库就认为接口齐全，也不能根据网页自制同名类型代替实际 DDS 定义。

### 5.2 原厂系统准备和网络检查

保持原厂控制流程：

```text
机器人开机
→ 控制器或原厂命令站立
→ 原厂 basic_server 自动推进到 RL 控制（motion_state=17）
→ 原厂系统与接口就绪
→ 启用本项目运控适配器
→ 前端发送一次巡检启动请求
```

本项目不发送站立/RL 状态切换命令，不替换原厂 RL 策略。执行前还需确认**导航使用模式**已启用、冲突的原厂 `planner` 已停止、`basic_server` 保持运行且没有自主充电任务。`motion_state=17` 与“导航使用模式”是不同条件。

按厂家流程加载现场定位地图并完成初始化或重定位，确认原厂定位正常。定位地图与项目路线必须采用一致的现场坐标基准；仅将地图文件放进本项目目录不会启动原厂定位。

按实际机器人设置所有实机终端的通信环境：

Galactic 默认 RMW 为 Cyclone DDS；本项目 M20 原厂接入使用 Fast DDS。请确认 Galactic 的 `rmw_fastrtps_cpp` 已安装，并显式设置下列变量，不依赖默认 RMW：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0       # 官方示例值；以机器人实际配置为准
export ROS_LOCALHOST_ONLY=0  # 从 demo 切换时也要修改
ros2 topic info /ODOM --verbose
ros2 topic hz /MOTION_INFO
ros2 topic echo /LOCATION_STATUS --once --qos-reliability best_effort
```

确认 Orin 与机器人时间同步，并且确实收到下列输入；发现话题名不等于收到数据：

| 原厂接口 | 用途 |
|---|---|
| `/ODOM`，`nav_msgs/msg/Odometry` | 原厂机体定位，通常 10 Hz |
| `/LOCATION_STATUS`，`drdds/msg/LocationStatus` | 定位质量，通常 2 Hz |
| `/MOTION_INFO`，`drdds/msg/MotionInfo` | 真实运动状态、步态和速度，通常 20 Hz |
| `/IMU`，`sensor_msgs/msg/Imu` | 角速度与稳定性检查 |
| `/HES_STATUS`、`/CHARGE_STATUS` | 急停、充电冲突与反馈超时检查 |

指南将 `/CHARGE_STATUS` 标为仅 33 网段可见，Orin 必须具备实际接收条件；缺失时不会放行。定位正常码在官方不同章节中存在 0/1 的矛盾，需根据本机固件和实际健康状态确认，不能同时接受两者。

### 5.3 接入支撑参考定位

原厂 `/ODOM` 是机体 `base_link` 位姿，本项目路线使用支撑表面参考点 `ground_projection`。需由标定或支撑参考估计程序提供按观测时刻有效的 TF：

```text
base_link → navigation_ground_reference
```

检查 TF 是否存在：

```bash
ros2 run tf2_ros tf2_echo base_link navigation_ground_reference
```

TF 可查到还需验证转换结果与现场坐标、高度一致。平地关系需实测标定，楼梯通常需要动态转换；不能通过改 `frame_id`、固定减身高或直接使用 `MotionInfo.height` 代替支撑参考估计。

**项目已有 TF 转换和晚到重试接口，尚未实现支撑面估计器。** 缺少有效转换时，定位适配器不会发布可用的 `/localization/pose`。

### 5.4 先启动只读联调

复制运控配置模板：

```bash
export STAIR_SITE_DIR="$HOME/stair_site"
mkdir -p "$STAIR_SITE_DIR"
cp "$(ros2 pkg prefix stair_nav)/share/stair_nav/config/m20_orin.yaml" \
  "$STAIR_SITE_DIR/m20_orin.yaml"
```

该命令仅用于首次准备模板，已有现场配置时不要重复覆盖。按第 4 节准备好 `route.yaml`、`mission.yaml` 和地图/路径文件。

设置定位正常码，再启动。**下方 -1 是阻断值，必须在核实后改成 0 或 1**：

```bash
export M20_LOCATION_OK=-1
ros2 launch stair_nav m20_orin.launch.py \
  route_file:="$STAIR_SITE_DIR/route.yaml" \
  mission_file:="$STAIR_SITE_DIR/mission.yaml" \
  adapter_config:="$STAIR_SITE_DIR/m20_orin.yaml" \
  normal_location_status:="$M20_LOCATION_OK"
```

默认 `dry_run=true`，不向原厂 `/NAV_CMD` 和 `/GAIT` 写入。检查：

```bash
ros2 topic echo /m20/localization_diagnostic
ros2 topic echo /m20/motion_diagnostic
ros2 topic echo /localization/pose
```

上面每条 echo 持续运行，分别在终端查看或用 Ctrl+C 切换。其他终端同样需要加载 ROS/厂商/本项目环境及正确 Domain。

验收：定位持续更新，转换后位置、朝向和 Z 高度与地图和路线一致；原厂输入新鲜，诊断能解释当前未就绪原因。`dry_run` 下运动反馈不可执行是预期行为，不能在机器人不动时验收真实任务成功。

### 5.5 启用实机运动并发送任务

先停止只读启动。核实现场条件后，以相同配置重新启动：

```bash
ros2 launch stair_nav m20_orin.launch.py \
  route_file:="$STAIR_SITE_DIR/route.yaml" \
  mission_file:="$STAIR_SITE_DIR/mission.yaml" \
  adapter_config:="$STAIR_SITE_DIR/m20_orin.yaml" \
  normal_location_status:="$M20_LOCATION_OK" \
  dry_run:=false navigation_mode_confirmed:=true planner_stopped:=true
```

`navigation_mode_confirmed` 和 `planner_stopped` 只是对人工核实结果的声明，**不会自动切换使用模式或停止原厂服务**。这些 launch 参数及 `normal_location_status` 会覆盖 YAML 中同名设置，单改 YAML 不够。配置参数仅在启动时生效。

适配器每次启动仍为 disabled。机器人处于状态 17、定位和安全反馈正常、新鲜且连续稳定后，在另一终端启用：

```bash
ros2 service call /m20_motion_adapter/enable std_srvs/srv/SetBool '{data: true}'
```

确认返回 `success: true`。该服务只允许适配器执行导航命令，不启动巡检。再检查导航端空闲、控制已释放：

```bash
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
```

预期 `busy: false`、`control_released: true`。随后只发送一次：

```bash
ros2 action send_goal /execute_mission stair_nav_interfaces/action/ExecuteMission \
  '{route_id: ""}' --feedback
```

任务接收后仍需通过定位、运控、能力和起点等就绪检查。段执行器先请求 HOLD、必要时切换目标步态，收到真实反馈并确认稳定后才开始跟踪。

实际运动采用 **ROS 话题接口**：`/NAV_CMD` 输出 `x_vel=vx、y_vel=0、yaw_vel=wz`，`/GAIT` 请求目标步态；不是从 Orin 直接调用 basic_server 协议。原厂 basic_server 继续工作。

### 5.6 分阶段验收

| 阶段 | 通过标准 |
|---|---|
| 软件 demo | 完整模拟任务最终 `SUCCEEDED` |
| 实机只读 | 原厂反馈、定位质量、支撑参考 TF 和地图坐标一致 |
| 平地单段 | 实际到点并保持，任务返回 `success: true / SUCCEEDED`，控制已释放 |
| 平地多段 | 每段重新规划、段间保持和释放正常，任务顺序正确 |
| 楼梯单段 | 已补齐支撑定位和保持/出梯监督，真实步态切换与到达确认通过 |
| 完整巡检 | 各段串行完成，取消与异常停止验证通过，完整日志可追溯 |

先在模拟环境验证输入中断等故障，再在平地验证取消和停止，最后进入楼梯。建议记录 `/localization/pose`、`/motion/status`、`/navigation/command`、`/navigation/status`、`/mission/status` 和原厂反馈，便于区分定位、控制和状态机问题。

M20 模板默认速度上限为 `vx=0.3 m/s、wz=0.6 rad/s`。官方给出的部分固件非零速度死区约为平地 `|vx|≥0.15 m/s、|wz|≥0.35 rad/s`，楼梯 `|wz|≥0.4 rad/s`；实际以本机响应为准。适配器只诊断死区，不自动提升速度。不要将 0.05 m/s 等小值直接当作可执行慢速，也不要照搬 demo 的减速与到达参数。

### 5.7 完整楼梯任务还需要什么

模板默认仅声明平地能力。接入楼梯前，按实测结果配置 `supported_modes` 和 `stair_motion`、`stair_yaw_correction` 等能力，并与路线要求一致。

官方反馈没有可直接映射的“安全保持”和“全机体完全出梯”确认位。当前楼梯流程需要外部监督器持续向 `/m20/confirmation` 发布 `stair_nav_interfaces/msg/MotionStatus`，包含匹配的执行/段 ID、路线 frame、真实观测时间和有证据的 `stable/hold_confirmed`；楼梯转平台还需 `exit_confirmed`。

**该监督识别器尚未实现。** 平地保持可由 HOLD 后真实速度/IMU 连续稳定推导；楼梯不能只凭中心到点、零速度、固定等待或手工发布 true 就确认完成。地图准备完毕不代表完整实机楼梯任务已具备全部输入。

## 6. 任务监控、取消和恢复

### 6.1 常用接口

| 接口 | 类型 | 用户用途 |
|---|---|---|
| `/execute_mission` | `stair_nav_interfaces/action/ExecuteMission` | 启动一次任务并等待反馈、结果 |
| `/task_manager/start` | `std_srvs/srv/Trigger` | 启动当前任务的兼容入口，与 Action 二选一 |
| `/task_manager/cancel` | `std_srvs/srv/Trigger` | 请求取消当前任务 |
| `/task_manager/reset` | `std_srvs/srv/Trigger` | 满足条件后解除任务故障阻塞 |
| `/mission/status`、`/mission/events` | `MissionStatus`、`MissionEvent` | 任务快照和有序状态变化 |
| `/navigation/status` | `SegmentStatus` | 当前段进度、误差和保持状态 |
| `/m20_motion_adapter/enable` | `std_srvs/srv/SetBool` | 启用或撤销 M20 运控适配器 |

任务主要过程为 `VALIDATING → WAITING_READY → DISPATCHING → APPROACHING_START/RUNNING`；每段经历 `CHECKING → PLANNING → WAIT_READY → EXECUTING → ARRIVAL_CONFIRM`。几何到达后仍需保持、稳定和所配置的出梯确认，才会完成该段。

状态话题使用 Reliable、TransientLocal QoS，晚加入的客户端也能读取最近快照，但仍须检查时间和有效期。任务层的 `segment_progress` 是当前段进度，不是整条路线的距离百分比。

### 6.2 取消与撤销

请求取消：

```bash
ros2 service call /task_manager/cancel std_srvs/srv/Trigger '{}'
```

服务接受请求后，状态进入 `CANCELING`；只有下游执行终止、实际保持和控制释放确认后，才算取消完成。Action 前端优先使用目标句柄取消；通过兼容服务取消 Action 任务时，业务状态为 `CANCELED`，ROS Action 协议终态为 `ABORTED`。

撤销适配器执行权限：

```bash
ros2 service call /m20_motion_adapter/enable std_srvs/srv/SetBool '{data: false}'
```

该操作请求零速度并等待真实停稳，不等同于硬急停、趴下或断电。步态请求尚未确认时，不会跳过握手直接报告释放。

### 6.3 故障恢复

`FAULT_BLOCKED` 表示停止/释放状态不明或清理失败，会拒绝新任务。先核实原厂状态、实际保持和仍存活的执行，再恢复新鲜输入、重新启用适配器，满足条件后请求：

```bash
ros2 service call /task_manager/reset std_srvs/srv/Trigger '{}'
```

reset 仅解除任务守卫，不复位本体，不强行释放控制，也不自动恢复旧任务。步态切换超时仍保持闭锁；需人工核实后处理，不能靠反复发任务绕过。

## 7. 接入其他机器人

使用仅含状态机的正式入口，并自行启动定位与运动适配器：

```bash
ros2 launch stair_nav navigation.launch.py \
  route_file:=/absolute/path/site_route.yaml \
  mission_file:=/absolute/path/site_mission.yaml
```

| 适配接口 | 必须满足的约定 |
|---|---|
| `/localization/pose`，`geometry_msgs/msg/PoseStamped` | 与路线同 frame、支撑参考点、真实观测时间、单位四元数 |
| `/navigation/command`，`stair_nav_interfaces/msg/ControlCommand` | 接收执行 ID、模式、阶段、TRACK/HOLD、vx/wz 和有效期 |
| `/motion/status`，`stair_nav_interfaces/msg/MotionStatus` | 回显当前执行/段 ID，持续提供实际 ready、模式、能力、故障和保持/出梯状态 |

`vx` 为水平面内沿机器人 yaw 前向的速度，单位 m/s；`wz` 为绕全局重力方向的 yaw 角速度，单位 rad/s。路径 Z 用于匹配和到达判断，导航不输出竖直速度。

默认每 0.05 s 生成指令，TTL 为 0.2 s。适配器必须独立处理过期、断联和 HOLD，并依据真实反馈确认保持；不能将发过零速度等同于已经停稳。定位与反馈不得重盖时间戳掩盖旧观测。

`navigation.launch.py` 支持通过 launch 参数修改定位/反馈/指令话题、Action、服务和监控话题名称；`pose_qos_reliability` 可选 `reliable` 或 `best_effort`。改名不改变消息类型，原厂 Odometry 仍需转换。

## 8. 常见问题

| 现象 | 优先检查 |
|---|---|
| 找不到包、launch 或仍运行旧代码 | 是否在工作空间根目录构建、是否 source 正确安装空间、`ros2 pkg prefix stair_nav` 是否正确 |
| 编译成功但 M20 节点报缺消息类型 | 固件匹配的 drdds 是否构建并 source，是否包含 LocationStatus |
| 能发现话题却收不到数据 | Domain、Fast DDS、网络/网段、QoS，以及 demo 遗留的 ROS_LOCALHOST_ONLY=1 |
| 没有 /localization/pose | 定位正常码、源时间、原始 frame、支撑参考 TF 和定位诊断 |
| enable 失败 | dry_run、导航使用模式声明、planner 声明、状态 17、定位/IMU/安全反馈、连续稳定和未完成步态握手 |
| 任务 accepted 后一直等待或失败 | /mission/status 的错误信息、起点容差、反馈新鲜度、能力和控制释放状态 |
| CAPABILITY_MISSING 或模式不支持 | 路线是否继承了 demo 的楼梯/原地转向要求，适配器是否仅声明 flat |
| 快到目标时停滞或朝向不收敛 | 原厂速度死区、减速参数、到达容差及实际定位误差 |
| 楼梯已到终点但不完成 | 当前执行的保持、稳定和完全出梯证据是否真实、新鲜且匹配 ID |

## 9. 开发检查与代码位置

以下命令用于 **Galactic 目标机**上的迁移验收。先新开终端加载正确环境，确认 `ROS_DISTRO` 为 `galactic`、Python 为 3.8，再在仓库根目录运行：

```bash
source /opt/ros/galactic/setup.bash
source ~/galactic_ws/install/setup.bash
printenv ROS_DISTRO
python3 --version
cd ~/galactic_ws/src/stair_nav
PYTHONPATH=stair_nav python3 -m stair_nav.demo --output /tmp/stair_nav_headless.json
PYTHONPATH=stair_nav python3 -m pytest -q stair_nav/test
```

构建并 source 工作空间后，可以运行 ROS 接口检查与隔离模拟场景：

```bash
python3 stair_nav/test/ros/check_callbacks.py
python3 stair_nav/test/ros/check_visualization.py
python3 stair_nav/test/ros/check_deployment_interfaces.py
python3 stair_nav/test/ros/check_m20_localization.py
python3 stair_nav/test/ros/check_m20_motion.py
python3 stair_nav/test/ros/check_mission_client.py
python3 stair_nav/test/ros/ros_action_smoke.py --output-dir /tmp/stair_nav_ros_smoke
python3 stair_nav/test/ros/mission_smoke.py --output-dir /tmp/stair_nav_mission_smoke
```

M20 检查使用 ROS 基础类型和离线字段夹具，不连接机器人；场景脚本使用模拟节点和独立 Domain。软件回归不替代现场步态、制动和出梯识别验证。

迁移验收至少包含：完整模拟任务成功、接受应答前/执行中取消、取消超时不误报停稳、定位失效、缺少出梯确认以及执行端重启不恢复旧任务。通过这些软件检查后，再按第 5 节从只读接入、平地单段开始验证实机。

主要代码位置：

| 路径 | 职责 |
|---|---|
| [core/mission.py](stair_nav/stair_nav/core/mission.py)、[task_node.py](stair_nav/stair_nav/task_node.py) | 任务状态机与前端接口 |
| [core/engine.py](stair_nav/stair_nav/core/engine.py)、[segment_node.py](stair_nav/stair_nav/segment_node.py) | 段状态机与 Action |
| [core/planning.py](stair_nav/stair_nav/core/planning.py)、[core/tracking.py](stair_nav/stair_nav/core/tracking.py) | 路径规划与跟踪 |
| [m20_localization_node.py](stair_nav/stair_nav/m20_localization_node.py) | 原厂定位质量检查、参考点转换 |
| [m20_motion_node.py](stair_nav/stair_nav/m20_motion_node.py) | M20 原厂 ROS 运控适配 |
| [mission_client.py](stair_nav/stair_nav/mission_client.py)、[mission_console.py](stair_nav/stair_nav/mission_console.py) | 中文任务 Action 客户端与反馈摘要 |
| [stair_nav_interfaces](stair_nav_interfaces) | 消息、Action、服务定义 |

## 10. M20 接口依据

接入说明依据[《山猫 M20 开发指南》](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/mExel2BLV54bA9pbHvlrl10XWgk9rpMq)、[运动控制章节](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/KGZLxjv9VG3OAkQOS6ALnkKGV6EDybno)、[定位章节](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/Qnp9zOoBVBZwADmwhPMDzZPlV1DK0g6l)、[话题速查](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/oP0MALyR8k73gzN3iYagqwbK83bzYmDO)、[DDS 说明](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/o14dA3GK8g5y4QGySnyegPppV9ekBD76)和[已核对的官方消息源码版本](https://github.com/DeepRoboticsLab/deep-robotics-msg/tree/aa646c1782aa982f384b7e46469723f31696f778)。

文档与源码存在差异时，以该台机器人实际固件、已生成消息定义和现场反馈为准。适配器支持原厂 header 的 `stamp.sec/nanosec` 与文档中的 `timestamp.sec/nsec` 两种布局；原厂数字 `header.frame_id` 是序号，不能作为 TF 坐标系名称。
