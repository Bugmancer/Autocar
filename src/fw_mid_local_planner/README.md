# fw_mid_local_planner

局部导航执行层：接收目标，调用 A*，处理路径，维护点云障碍记忆，并在车身与制动碰撞检查后发布 `/cmd_vel`。整车构建、地图、网络、CAN 和停机操作见[工作区说明](../../README.md)。

## 选择策略

`path_follower_node.py`、`path_follower_node_v1.py` 与 `path_follower_node_v3.py` 是三个独立可执行入口，组合共用的导航运行逻辑和各自的跟踪策略，不通过节点继承替换策略。三者保留 ROS 节点名 `/path_follower_node`，接口和公共安全检查一致，每次只能运行一个。

```bash
# 默认 PID / Pure Pursuit 策略
bash scripts/ros1.sh roslaunch fw_mid_bringup navigation.launch follower_variant:=classic

# 人工势场策略
bash scripts/ros1.sh roslaunch fw_mid_bringup navigation.launch follower_variant:=apf

# 自适应前视、几何斥力与短轨迹选择策略
bash scripts/ros1.sh roslaunch fw_mid_bringup navigation.launch follower_variant:=v3
```

这些命令默认不启用 CAN；初值、地图和硬件放行步骤见根文档。切换前停止原 launch，等待退出后再启动，并用 `rostopic info /cmd_vel` 确认只有一个速度发布者。

`follower_variant` 默认 `classic`，仅接受 `classic`、`apf`、`v3`。旧 `follower_node` 参数兼容三个入口脚本名，显式设置时覆盖策略选择；新命令优先使用 `follower_variant`。初值保存与启动时加载的流程不变。

代码入口只负责启动：`follower_runtime.py` 维护公共导航与安全流程，`tracking.py`、`apf_tracking.py`、`adaptive_tracking.py` 实现各策略；公共参数和 RViz 输出分别放在 `follower_parameters.py`、`follower_visualization.py`。

独立调试需先准备 A*、点云和 TF，再启动：

```bash
bash scripts/ros1.sh roslaunch fw_mid_local_planner local_planner.launch follower_variant:=v3 require_localization:=true max_vx:=0.05 max_wz:=5.0
```

局部入口默认 `require_localization:=false`，完整导航会强制开启；独立调试示例显式开启。若其他节点已发布 `body -> base_link`，加 `publish_body_to_base_link_tf:=false`。分开启动 A* 与局部规划时，必须传入同一份 `geometry_config`。

## 策略行为

| 策略 | 行为与配置 |
| --- | --- |
| `classic` | `tracking_controller: pid`（默认）或 `pure_pursuit`；参数前缀 `pid_`、`pp_` |
| `apf` | 最多三个离散路径点的吸引力与障碍斥力合成方向；参数前缀 `apf_` |
| `v3` | 弧长自适应前视、曲率/制动限速、扇区几何斥力、绕行侧滞回、短轨迹选择；参数前缀 `v3_`，仅 V3 读取 |

`max_vx` 是直线巡航速度和纵向上限，单位 m/s；`max_wz` 是角速度上限，单位 deg/s。控制器仍保留加速度、转弯、到点与碰撞降速。最终 `/cmd_vel` 的角速度单位是 rad/s，限幅后才做碰撞预测；独立启动底盘时必须使用相同上限。

APF 从当前路径中选择近点，再按 `apf_waypoint_stride` 向前取远点和更远点；重复的终点不重复计算吸引力。合力使用向量低通滤波，停车、接受新路径或恢复段接回跟踪时清除旧滤波状态。危险区外持续合力抵消时停车，在 `astar_replan` 模式下重新请求 A*；不保证消除所有局部极小值。

障碍斥力随真实观测年龄衰减，但车身碰撞检查、危险净空判定和 A* overlay 继续使用完整记忆。`apf_danger_clearance` 是障碍到带余量矩形车身的净空，再计入障碍体素半径，不是距车心的距离。当前 APF 在危险区按净空缩减候选速度，合力抵消时尝试纯吸引力方向；大转角也允许低速前行。最终输出仍须通过动态急停、完整车身及实测制动检查，不安全则停车。

APF 关闭通用的 `obstacle_slowdown_enabled`，但保留上述策略内降速和共用碰撞保护。全部当前数值以 [local_planner.yaml](config/local_planner.yaml) 和共享 [collision_geometry.yaml](../fw_mid_common_utils/config/collision_geometry.yaml) 为准。修改后停车并重启节点，运行中不自动重载。

### V3 算法与参数

- **路径进度与前视**：在有序路径上投影，按弧长插值取目标；投影前进范围随车辆实际位移增长，保留后方绕行段。前视随速度增加、随预瞄曲率缩短并平滑变化，急弯降低远目标权重。
- **连续限速**：联合路径/转向曲率、角速度上限、横向加速度和终点制动确定速度，并约束候选命令的 `v * abs(w)`。大航向误差转入原地转向；到点同时要求 XY 距离与剩余弧长进入容差。
- **几何斥力与绕行侧**：每个角扇区保留到带余量矩形车身净距最近的障碍，重复点不会叠加放大斥力；全局路径参与选择绕行侧，切向引导与保持时间减少反复换边。
- **短轨迹选择**：在可达速度窗内生成短弧线，按路径进展、偏离、方向、净距和速度变化排序，再依次执行完整车身扫掠与实测速度制动检查。扇区压缩只用于引导和评分，碰撞检查仍使用完整近场障碍与静态地图。检查次数或时间预算耗尽且无安全候选时停车；时间预算只在完整检查之间判断，不能保证单周期耗时上限，最终命令仍经过公共发布检查。
- **停滞恢复**：在 `dynamic_avoidance_mode: astar_replan` 下，持续缺少沿路径进展时请求 A* 重规划，并沿用公共重规划间隔。正常原地转向通过实际航向变化豁免，传感器或定位无效时不累计停滞。

关键参数集中在 [local_planner.yaml](config/local_planner.yaml)，以下只列调参入口；所有 `v3_*` 参数仅 V3 读取。距离为米、时间为秒，V3 角度参数为弧度，区别于 launch 的 `max_wz`（deg/s）。

| 分组 | 代表参数 | 用途 |
| --- | --- | --- |
| 前视 | `v3_lookahead_min`、`v3_lookahead_max`、`v3_lookahead_time`、`v3_curvature_preview` | 前视上下限、速度响应和曲率预瞄范围 |
| 速度与转向 | `v3_lateral_acceleration`、`v3_accel_v`、`v3_accel_w`、`v3_rotate_threshold` | 横向加速度、速度变化率及原地转向阈值 |
| 终点接近 | `v3_goal_approach_distance` | 终点减速范围；制动减速度沿用公共碰撞配置 |
| 障碍引导 | `v3_obstacle_influence`、`v3_obstacle_sectors`、`v3_tangent_gain`、`v3_side_hold_time` | 净距作用范围、扇区数、切向引导和绕行侧保持 |
| 候选检查 | `v3_trajectory_max_checks`、`v3_trajectory_time_budget` | 完整碰撞检查次数和软时间预算 |
| 停滞判断 | `v3_stall_time`、`v3_stall_progress` | 观察时长及期间要求的最小沿路径进展 |

源码分工：

| 文件 | 职责 |
| --- | --- |
| [path_follower_node_v3.py](fw_mid_local_planner/path_follower_node_v3.py) | 独立入口，组合公共运行层与 V3 策略 |
| [arc_path.py](fw_mid_local_planner/arc_path.py) | 弧长插值、受限投影和曲率查询 |
| [adaptive_controller.py](fw_mid_local_planner/adaptive_controller.py) | 纯计算控制器、几何势场、速度参考及候选排序 |
| [adaptive_tracking.py](fw_mid_local_planner/adaptive_tracking.py) | 运行层适配、完整候选检查、发布同步和停滞重规划 |

V3 不预测移动障碍速度，也不保证消除所有局部极小值；尚未完成实车性能验证。

## 共用安全行为

- 完整导航要求定位有效、TF 新鲜。目标在定位无效时会被拒绝；已有目标在短时输入异常恢复后可能继续执行。到点要求 XY 距离进入容差，V3 还检查剩余弧长；均不保证最终朝向。
- 障碍观测按点云时间戳转换，在 FAST-LIO 连续世界系 `lidar` 中保存三维体素，再投影到 `map` 与 `base_link`。`lidar` 在此不是雷达光心。
- 新障碍受观测范围、地面/车身和同帧邻域过滤约束；高度相对 `base_link`。低矮、细小或稀疏障碍可能被过滤，应按实车校准。
- 记忆只在 RAM 中保存，无目标、换目标和到点都保留。真实空闲射线可清除旧体素；空点云、遮挡、盲区和观测年龄不构成清除证据。记忆满时阻止运动，不自动淘汰历史障碍。
- 最终命令检查矩形车身的运动扫掠、反应与制动过程，使用静态地图和障碍记忆。未知区与地图外按占用处理。预测碰撞时可尝试更低候选速度，仍使用原实测速度检查制动；无安全候选就停车。
- 默认允许在 `dynamic_obstacle_timeout` 后的短窗口内降速继续，参数为 `dynamic_degraded_*`。前提包括已有有效观测、路径和非零命令，且静态/记忆/制动检查通过；超过窗口停车。期间无法检测新障碍，当前紧急包络检查也会因数据过期返回未触发，不能保证额外配置的紧急停车距离。
- A* 等待动态 overlay 应用确认后规划，路径后处理也检查障碍记忆。重规划期间仅在允许继续跟踪、存在旧路径且检查通过时行驶；规划失败或无法安全运动时停车。
- 起点落入保守膨胀区时，可尝试受限前行恢复：当前车身、实测制动、完整恢复扫掠及出口到目标路径均须安全。接回路径前再次复查，偏离恢复段或无安全出口时停车；不清除真实障碍、不自动倒车。

这些机制不能代替实物急停，不能感知从未观察到的侧面障碍，也不预测移动障碍的未来位置。不要放宽传感器超时或缩小几何尺寸来绕过停车。

## 接口与诊断

| 接口 | 用途 |
| --- | --- |
| `/move_base_simple/goal`、`/goal_pose` | `PoseStamped` 目标；新目标替换当前目标 |
| `/fastlio2/body_cloud` | 默认 `body` 帧点云输入 |
| `/astar_planner_node/get_plan` | `GetPlan` 全局规划服务 |
| `/astar_planner_node/map`、`/astar_planner_node/costmap` | 原始栅格与膨胀/overlay 代价图 |
| `/astar_planner_node/dynamic_overlay_stamp` | 障碍快照应用确认 |
| `/cmd_vel` | `Twist` 速度输出，m/s 与 rad/s |
| `/local_planner/global_plan`、`/local_planner/desired_direction` | 跟踪路径与方向箭头 |
| `/local_planner/dynamic_obstacle_points` | 给 A* 的完整障碍记忆快照 |
| `/dynamic_obstacles_markers` | 近期观测（绿）、历史记忆（橙）和观测范围（青） |
| `/local_planner/collision_check` | 当前车身及预测轮廓，碰撞点用红球标记 |
| `/local_planner/collision_blocked` | 碰撞/障碍输入联锁状态 |
| `/path_follower_node/clear_obstacle_memory` | `Trigger`：停车、取消目标并清空记忆及 overlay |

清除误记忆前确认现场环境：

```bash
bash scripts/ros1.sh rosservice call /path_follower_node/clear_obstacle_memory "{}"
```

清空后等待新点云并重新发目标；仍存在的障碍会重新加入记忆。单次零速消息不会取消导航，正常结束应停止 launch。

`APF:` 日志包含合力、净空与实际发布速度。`V3:` 日志字段如下：

| 字段 | 含义 |
| --- | --- |
| `state` | 控制状态，如 `tracking` 跟踪、`rotating` 转向、`trajectory_blocked` 无可用候选、`force_cancelled` 合力抵消、`invalid_input` 输入异常 |
| `progress` | 当前路径起点累计弧长（m），接受新路径后重新计算 |
| `lookahead` | 当前平滑后的前视距离（m） |
| `clearance` | 势场近场扇区中的最小车身净距（m）；`inf` 表示该范围内无障碍点，不代表全环境无障碍 |
| `side` | 绕行侧：`1` 左、`-1` 右、`0` 未保持绕行侧 |
| `accepted` | 本次命令是否被公共发布流程接受，不表示底盘已执行，也不表示输出一定非零 |

`V3 path progress stalled; requesting A* plan` 表示沿路径进展不足并已请求重规划；观察 `progress` 是否增长，同时区分正常原地转向与实际受阻。`Vehicle footprint blocked`、`Dynamic obstacle emergency stop`、`Obstacle input stale` 分别表示车身碰撞、近距离急停与数据过期。`Navigation control cycle took ...` 提示计算延迟，下游 `Twist input timed out` 是独立看门狗。排查时先确认原因，不通过重复旧非零命令掩盖超时。

## 验证

工作区根目录运行（Python 3、NumPy）：

```bash
python3 -m unittest discover -s src/fw_mid_local_planner/tests -v
```

覆盖力/速度计算、障碍投影与记忆、碰撞检查和规划/恢复/发布接口。ROS 传输使用替身，仍需 Noetic 构建、ROS 联调及受控实车验证。
