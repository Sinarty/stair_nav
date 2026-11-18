# APPROACH_A 实机测试启动流程

本流程在外部 Orin 上执行，只运行“当前有效定位 → A 点”的平地任务。准备三个终端：A 运行导航节点，B 检查、启用和发送任务，C 记录 rosbag。

当前设置：A 点为地图坐标 `[2.0, 0.0, 0.05]`，最终朝向 `yaw=0.0`（地图 +X）；定位输入为 `/localization/odometry`；原厂定位状态码检查已关闭，无需填写正常码。测试地面参考点位于机身正下方，地图 Z 固定为 `0.05 m`，TF 由定位适配器自动发布。

## 1. 每个终端加载环境

在终端 A、B、C，以及之后新开的操作终端中分别执行：

```bash
source /opt/ros/galactic/setup.bash
source "$HOME/m20_interfaces_ws/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export STAIR_SITE_DIR="$HOME/ros2_ws/src/stair_nav/stair_site"
export STAIR_TEST_LOG_ROOT="$HOME/stair_test_logs"
mkdir -p "$STAIR_TEST_LOG_ROOT"

ros2 pkg prefix drdds
ros2 pkg prefix stair_nav
```

各终端 Domain 必须与机器狗一致：上面保留已有 `ROS_DOMAIN_ID`，未设置时使用 `0`；沿用已配置的 DDS 网络和 XML。部署目录不同时修改 `STAIR_SITE_DIR`。

更新源码后需要重新构建时，在终端 B 执行一次：

```bash
cd "$HOME/ros2_ws"
colcon build --packages-up-to stair_nav --symlink-install
source "$HOME/ros2_ws/install/setup.bash"
```

下文采样使用 `timeout` 自动结束，返回码 `124` 表示到达采样时限。当前 Galactic 的 `ros2 topic echo` 不使用 `--once`。

## 2. 启动并检查修正定位

按现有定位工程的启动方式，加载现场地图并完成初始化，使 `/localization/odometry` 持续发布。本仓库不包含该定位程序的启动入口，后面的导航 launch 不会启动它。

终端 B：

```bash
ros2 topic type /localization/odometry
ros2 topic info /localization/odometry --verbose
timeout 3s ros2 topic echo /localization/odometry --qos-reliability best_effort
```

继续前确认：

- 类型为 `nav_msgs/msg/Odometry`，`header.frame_id=map`，`child_frame_id` 为正确且非空的机身坐标系。
- 源时间持续更新，与 Orin 时间一致；位姿年龄不超过 `0.5 s`，位置和四元数有效。
- 定位与地图坐标一致，地图 `(2.0, 0.0)` 对应现场 A 点，起点到 A 的实际通道与地图一致。
- 当前障碍膨胀半径为 `0.4 m`，应与机体、负载和现场净空相符。

## 3. 只读启动导航

终端 A 执行并保持运行：

```bash
ros2 launch stair_nav m20_orin.launch.py \
  route_file:="$STAIR_SITE_DIR/route.yaml" \
  mission_file:="$STAIR_SITE_DIR/mission.yaml" \
  adapter_config:="$STAIR_SITE_DIR/m20_orin.yaml" \
  dry_run:=true
```

必须显式传入这里的 `adapter_config`；默认安装模板与当前现场配置不同。此时不发送任务。

终端 B 核对生效参数：

```bash
ros2 param get /m20_localization odom_topic
ros2 param get /m20_localization require_location_status
ros2 param get /m20_localization test_flat_ground_projection
ros2 param get /m20_localization test_ground_z
ros2 param get /m20_motion_adapter require_location_status
ros2 param get /m20_motion_adapter dry_run
```

预期依次为 `/localization/odometry`、`false`、`true`、`0.05`、`false`、`true`。

```bash
timeout 3s ros2 topic echo /localization/pose
timeout 3s ros2 topic echo /m20/localization_diagnostic
timeout 3s ros2 topic echo /m20/motion_diagnostic
```

通过标准：输出位姿持续更新，地图 X/Y 与修正定位一致，Z 为测试值 `0.05`，朝向为机身 yaw；定位诊断为 `OK`。只读期间运动端显示 `dry_run`、未启用或不可执行属于预期。

若输入的 `child_frame_id=base_link`，可检查自动发布的 TF：

```bash
timeout 3s ros2 run tf2_ros tf2_echo base navigation_ground_reference
```

机身 frame 不同时，将 `base_link` 换成输入消息的实际名称。这个 TF 使用固定地面测试假设，Z 一致不代表已验证真实地面高度。

## 4. 准备原厂运控与反馈

按原厂操作流程站立、进入导航使用模式，使 `motion_state=17`；停止冲突的原厂 planner，保持 basic_server 运行，并结束充电任务。导航程序不会代替这些原厂操作。

终端 B：

```bash
timeout 3s ros2 topic echo /MOTION_INFO --qos-reliability best_effort
timeout 3s ros2 topic echo /IMU --field header --qos-reliability best_effort
timeout 3s ros2 topic echo /HES_STATUS \
  --qos-reliability reliable --qos-durability transient_local
timeout 3s ros2 topic echo /CHARGE_STATUS --qos-reliability best_effort
```

执行前核对：

- 真实状态、步态和速度可解析，运动状态为 `17`，源时间有效；目标平地步态配置为 `12290`，原地转向能力应与本机一致。
- 运动和 IMU 观测年龄均不超过 `0.5 s`，机器人连续稳定至少 `0.35 s`。
- 急停未触发；充电反馈 `state=0`、`error_code=0`。
- 急停和充电反馈均满足 `3 s` 接收有效期。急停保留历史状态的 QoS 不会自动续期，一次历史状态接收不能维持整次任务的有效期；发布节奏需与此策略匹配。
- 当前最大线速度为 `0.25 m/s`、最大角速度为 `0.5 rad/s`。`min_vx=0.1` 及末端小角速度的实际响应需结合本机验证。

## 5. 切换实机模式并启用

先在终端 A 按 `Ctrl+C` 停止只读 launch。确认第 4 步准备完成后，在同一终端执行：

```bash
ros2 launch stair_nav m20_orin.launch.py \
  route_file:="$STAIR_SITE_DIR/route.yaml" \
  mission_file:="$STAIR_SITE_DIR/mission.yaml" \
  adapter_config:="$STAIR_SITE_DIR/m20_orin.yaml" \
  dry_run:=false \
  navigation_mode_confirmed:=true \
  planner_stopped:=true
```

require_fresh_estop:=false require_fresh_charge:=false
先忽略原厂充电信息与急停信息

这三个参数覆盖 YAML 中的同名值；后两个参数声明原厂操作已经完成，不会自动切换模式或停止 planner。launch 不会自动发任务，适配器仍未启用。

终端 B 再检查重启后的输入：

```bash
timeout 3s ros2 topic echo /m20/localization_diagnostic
timeout 3s ros2 topic echo /m20/motion_diagnostic
```

显式启用适配器：

```bash
ros2 service call /m20_motion_adapter/enable std_srvs/srv/SetBool '{data: true}'
```

必须返回 `success: true`；失败时按返回原因处理，再进入任务步骤。

```bash
timeout 3s ros2 topic echo /motion/status \
  --qos-reliability reliable --qos-durability transient_local
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
```

运动反馈应为 `ready: true`、`stable: true`，`fault_code` 为空；导航服务应返回 `busy: false`、`control_released: true`。

## 6. 开始记录

终端 C（已加载第 1 步环境）执行并保持记录：

```bash
ros2 bag record -s sqlite3 \
  -o "$STAIR_TEST_LOG_ROOT/approach_a_$(date +%Y%m%d_%H%M%S)" \
  /localization/odometry /localization/pose \
  /tf /tf_static \
  /navigation/reference_path /navigation/command /navigation/status \
  /motion/status /mission/status /mission/events \
  /m20/localization_diagnostic /m20/motion_diagnostic \
  /MOTION_INFO /IMU /HES_STATUS /CHARGE_STATUS /NAV_CMD /GAIT
```

确认 recorder 已运行并订阅现有话题，再发送任务。后续出现的路径和指令话题会被自动发现。每次完整测试或取消测试使用单独的记录目录。

## 7. 发送一次任务

终端 B 执行并保持客户端运行：

```bash
ros2 run stair_nav mission_client --route-id site_approach_a
```

客户端显示中文状态和进度，并等待最终结果。仅发送一次；受理后仍需进行规划、HOLD 和步态握手，再开始跟踪。

需要额外监控时，新开终端、加载第 1 步环境，再执行：

```bash
ros2 topic echo /mission/status \
  --qos-reliability reliable --qos-durability transient_local
```

## 8. 验收到达结果

客户端应报告本次任务 `success: true`、`state: SUCCEEDED`，并确认停止、保持完成。任务结束后，终端 B 检查：

```bash
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
timeout 3s ros2 topic echo /mission/status \
  --qos-reliability reliable --qos-durability transient_local
timeout 3s ros2 topic echo /localization/pose
```

以本次 `task_id` 对应的新鲜状态为准：

| 项目 | 当前通过条件 |
|---|---|
| 实际位置 | 到地图 A 点 `(2.0, 0.0)`，水平误差不超过 `0.15 m` |
| 最终朝向 | 地图 +X，yaw 误差不超过 `10°` |
| 高度 | 参考点与 A 点均为测试平面 Z=`0.05`，当前投影模式下该条件自动满足 |
| 路径进度 | 至少 `0.97`，与位置条件同时满足 |
| 保持 | 几何到达、实际稳定和保持条件连续满足 `0.5 s` |
| 任务终态 | `SUCCEEDED`，`navigation_stopped: true`，`hold_confirmed: true` |
| 控制释放 | 导航服务返回 `busy: false`、`control_released: true` |

## 9. 单独进行中途取消测试

如果本次只做完整到达测试，直接进入第 10 节收尾。取消测试另起一次：先按第 10 节结束上一次测试，再准备一个到 A 点仍有可运行行程的起点，重新执行第 1～7 节，使用新的录包目录。

执行中，在运行 `mission_client` 的终端 B 按一次 `Ctrl+C`。客户端会请求取消并等待结果，默认等待时限为 `15 s`。等待本次任务返回 `CANCELED`，确认实际停稳、保持完成后检查：

```bash
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
timeout 3s ros2 topic echo /mission/status \
  --qos-reliability reliable --qos-durability transient_local
```

预期 `busy: false`、`control_released: true`，任务为 `CANCELED`，停止和保持标志均为 `true`。再次按 `Ctrl+C` 只退出客户端等待，不能据此认定取消完成。

若需要从其他已加载环境的终端请求取消，可执行：

```bash
ros2 service call /task_manager/cancel std_srvs/srv/Trigger '{}'
```

优先使用客户端自身的取消方式；取消服务接受请求后，仍需等待任务终态和控制释放。

## 10. 结束测试与故障恢复

正常收尾：先取得任务完成或取消结果，确认实际停稳和控制释放，再在终端 B 撤销适配器权限：

```bash
ros2 service call /m20_motion_adapter/enable std_srvs/srv/SetBool '{data: false}'
timeout 3s ros2 topic echo /m20/motion_diagnostic
```

确认实际保持停止、诊断 `stopping=false` 后，在终端 C 按 `Ctrl+C` 结束录包并等待写盘完成，最后在终端 A 按 `Ctrl+C` 关闭 launch。撤销权限后显示 `DISABLED` 属于预期。

出现问题时，先保留 recorder 和导航节点并查看：

```bash
timeout 3s ros2 topic echo /m20/localization_diagnostic
timeout 3s ros2 topic echo /m20/motion_diagnostic
timeout 3s ros2 topic echo /mission/status \
  --qos-reliability reliable --qos-durability transient_local
```

| 现象或错误 | 核对项目 |
|---|---|
| 没有 `/localization/pose` | 修正定位启动情况、输入 frame、机身 child frame、四元数、源时间及是否使用现场 YAML |
| `MOTION_STALE` / `IMU_STALE` | 对应源时间戳、时间同步、接收周期和 QoS |
| `ESTOP_STALE` / `CHARGE_STALE` | 发布端、实际接收周期与 `safety_timeout=3.0` 是否匹配 |
| `MOTION_STATE` / `GAIT_UNSUPPORTED` | 原厂状态、平地步态配置和真实回执 |
| 快到 A 时停滞或朝向不收敛 | 速度死区、减速参数、朝向修正和定位误差 |
| `FAULT_BLOCKED` | 先确认实际停稳、下游任务终止，恢复输入并重新启用适配器，再解除任务守卫 |

仅在满足上述恢复条件后，需要解除 `FAULT_BLOCKED` 时执行：

```bash
ros2 service call /task_manager/reset std_srvs/srv/Trigger '{}'
ros2 service call /navigation/get_state stair_nav_interfaces/srv/GetNavigationState '{}'
```

reset 只解除任务守卫，不自动恢复旧任务；确认可以开始新任务后，再单独发送下一次请求。


1118改动：
## 遇到的问题

| 问题 | 确认结果 |
|---|---|
| Leg-KILO 启动失败 | 先遇到 launch 命令和路径拼写错误，之后确认程序误加载 /opt/MVS 的 libusb，缺少 libusb_set_option，以退出码 127 退出。预加载系统版 libusb 后能启动。 |
| 重定位不稳定 | 曾出现 LOST 和 RELOCALIZING。日志确认 RViz 的 2D Pose Estimate 已送达，但配准分数过低或没有收敛候选。重启后曾恢复 LOCALIZED。 |
| 只读导航启动后话题和服务间歇不可见 | 同一批节点运行时，默认 Fast DDS 探针收不到里程计和诊断，参数查询 6 次全超时；仅 UDPv4 探针持续收包，参数查询 6 次全成功。共享内存路径是首要嫌疑，具体端口或锁未确认。 |
| 机身 TF 查询失败 | Leg-KILO 的实际机身坐标系是 base，现场文档里的 base_link 是示例。用 base 查询曾成功收到地面参考 TF。 |
| 里程计时效波动 | 雷达合并会跳过不完整窗口，里程计也有接收断档。同时采样时，/localization/pose 的上一帧在被替换前最老达到 1.275 s，超过 0.75 s。 |
| 实机启用先后被拦截 | /MOTION_INFO 持续上报状态 17，但源时间戳一直为 0，触发 MOTION_STALE。随后 /HES_STATUS 在 10 秒只收到一条，触发 ESTOP_STALE；/CHARGE_STATUS 多次未发现发布端。 |
| APPROACH_A 规划失败且清理超时 | 适配器启用成功，任务进入 PLANNING 后报 POSE_STALE。随后因未确认稳定、保持和控制释放而报 CLEANUP_TIMEOUT，进入 FAULT_BLOCKED。复查时 busy=false，但 control_released=false。 |
| 最近再次收不到里程计 | 本次已确认 legkilo_node 在 16:52:31 以 -6（SIGABRT）退出，DDS 图里里程计发布端为 0。退出前配准分数连续为 0.000，具体崩溃原因仍未确认。 |

## 已做的改动

- 定位时效从 0.5 s 调到 0.75 s，同步修改了定位适配器、任务就绪判断、段执行器三处。现场实测仍有超限。 适配器配置 (stair_site/m20_orin.yaml:19) · 任务配置 (stair_site/mission.yaml:10) · 路线配置 (stair_site/route.yaml:52)
- 增加 /MOTION_INFO 接收时间模式。现场启用 use_motion_receipt_time: true，订阅队列只保留最新一帧，以本机单调时钟判断是否断流。0.5 s 无新反馈仍报 MOTION_STALE，通用配置默认继续使用源时间戳。 运动适配器 (stair_nav/stair_nav/m20_motion_node.py)
- 增加两个独立启动参数 require_fresh_estop 和 require_fresh_charge，默认均为 true。设为 false 可跳过对应反馈的缺失和过期检查，已收到的急停触发、充电或充电错误仍会拦截。现场实机测试使用了两个 false。 启动文件 (stair_nav/launch/m20_orin.launch.py)
- 增加回归测试，覆盖零时间戳、接收断流及两个时效开关。最后一轮 100 项核心测试、13 项 ROS 接口测试通过。本机已确认是符号链接安装，这些改动重启导航即可加载，无需重新编译。
- 运行环境调整包括用系统版 libusb 启动 Leg-KILO，以及让定位、导航和查询终端统一使用 UDPv4 测试配置。UDP XML 目前位于 /tmp，尚未固化到部署目录。 UDP 测试配置 (/tmp/stair-dds-udp.xml)
