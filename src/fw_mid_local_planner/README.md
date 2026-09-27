# fw_mid_local_planner

ROS1 Noetic 局部规划包：跟踪 `fw_mid_global_planner` 的 A* 路径，将 FAST-LIO
点云写入本次会话的障碍记忆，并在发布速度前检查完整车身的运动及制动轨迹。
默认以 PID 跟踪，可切换 Pure Pursuit。

## 接口

| 接口 | 作用 |
| --- | --- |
| `/move_base_simple/goal`、`/goal_pose` | 目标输入 |
| `/fastlio2/body_cloud` | 点云输入，默认在 `body` 坐标系 |
| `/astar_planner_node/get_plan` | A* 规划服务 |
| `/astar_planner_node/map`、`/astar_planner_node/costmap` | 原始静态栅格、膨胀及 overlay 后的代价图 |
| `/astar_planner_node/dynamic_overlay_stamp` | A* 已应用障碍快照的确认 |
| `/cmd_vel` | `geometry_msgs/Twist` 速度输出，m/s 和 rad/s |
| `/local_planner/desired_direction` | RViz 当前跟踪方向箭头 |
| `/local_planner/dynamic_obstacle_points` | `PoseArray` 障碍记忆完整快照，供 A* overlay 使用 |
| `/local_planner/global_plan` | 后处理后的跟踪路径 |
| `/dynamic_obstacles_markers` | 绿色近期观测、橙色历史记忆的障碍体素 |
| `/local_planner/collision_check` | 蓝色当前车身轮廓、绿色通过检查或红色被拦截的预测轮廓 |
| `/local_planner/collision_blocked` | `Bool` 碰撞检查或障碍输入联锁状态 |
| `/path_follower_node/clear_obstacle_memory` | `Trigger` 停车、取消目标、清空障碍记忆及 overlay |

本包只发布 `/cmd_vel`。完整导航入口另行启动 `fw_mid_ctrl` 的 Twist/JSON 适配器，
并由 `enable_can` 控制是否打开 CAN；首次验证应在架空轮组或受控场地进行。

## 障碍记忆与碰撞检查

`obstacle_memory.py` 是无 ROS 依赖的三维体素记忆，默认 `0.10 m` 分辨率、
最多 `60000` 个体素。`dynamic_obstacle_layer.py` 按点云时间戳转换观测，
保存在 FAST-LIO 连续局部世界系 `lidar` 中，再投影到 `map` 和 `base_link`。
这里的 `lidar` 是 LIO 世界坐标系，不是雷达光心坐标系。

新增障碍的有效框默认相对 `base_link` 为 `-1.5 <= x <= 2.5 m`、
`abs(y) <= 1.5 m`，高度再按地面和 Z 范围过滤。参数为 `dynamic_x_min`、
`dynamic_x_max`、`dynamic_y_abs`。先裁剪候选点，再限制障碍点数；远处地面不再
进入障碍记忆或挤占近处点预算。RViz 同一 MarkerArray 的青色 `observation_box`
显示该框，原始 FAST-LIO 点云仍完整显示。框外真实回波可提供清除旧障碍的射线，
不会作为新障碍写入；已有记忆不会因进入盲区或移到框外就被删除。

地面剔除参数 `dynamic_ground_z_max` 默认由 `0.08 m` 提高到 `0.12 m`：
点云先转换到 `base_link`，再排除 `z <= 0.12 m` 的障碍候选点。
该数值相对 `base_link` 原点，不是雷达原始 Z 或未经校准的离地高度。
地面回波仍用于空闲射线更新；同高度范围内的低矮障碍也会被过滤。
若大片地板仍被误检，应检查安装外参及地面倾斜。停车重启导航后参数生效，
之前会话中误记的地面点也会随内存重置清除，原始地图不变。

记忆只在 RAM 中保存，不写入原始地图。无目标、到点或换目标都保留记忆，完整导航
重启后为空。FAST-LIO 原点重置时也应重启导航。障碍离开视野不会按时间消失；
后续真实射线经过原位置附近且收到更远回波，首次有效空闲观测即清除；
不经过重规划的约 `0.3 s` 确认。当前回波覆盖的位置继续保留。
每帧最多处理 `600` 条清除射线，射线终点默认留 `0.15 m` 保护距离。
1 秒证据窗口仅在确认帧数手动设为大于 1 时生效；空点云、遮挡或没扫描到不算消失。
清除同步更新 RViz、车身碰撞检查和 A* overlay，原始静态地图保持不变。
没有被射线覆盖的侧面盲区继续保留记忆。容量耗尽时停车，不自动淘汰旧障碍。

RViz 加载 `fw_mid_localizer/rviz/localizer.rviz`，Fixed Frame 使用 `map`。
`Observed and Remembered Obstacles` 中绿色表示最近 `0.5 s` 看到的障碍，
橙色表示历史记忆，两者都参与避障。`Vehicle Collision Prediction` 展示
包含安全余量的当前车身和预测轮廓。当前蓝框默认为 `0.72 m x 0.59 m`；
预测轮廓还覆盖运动和制动过程，碰撞检查另计入体素及栅格离散误差。

`footprint_collision.py` 独立检查矩形车身平移、转弯和制动扫过的区域，使用
原始静态栅格与障碍记忆；未知区和地图外视为占用。默认车身相对 `base_link`
前后各 `0.34 m`、半宽 `0.275 m`，四周安全余量 `0.02 m`。预测 `1.0 s`，
反应时间 `0.2 s`，制动减速度 `0.25 m/s²`、`0.5 rad/s²`，采样间距 `0.025 m`。
这些值以及地面高度、车身过滤、雷达外参必须按实际安装和停车能力校准。

新回波在写入记忆前进行同帧三维邻域过滤：默认
`dynamic_obstacle_support_radius=0.15 m`，
`dynamic_obstacle_support_min_points=3`（含自身，重复坐标只计一次）。
孤立点不新增到障碍记忆或膨胀层；保留全部满足条件的点簇。
计数发生在抽样、范围/地面/车身过滤和点数预算之后、体素合并之前；
多个回波可能合成一个 RViz 体素。稀疏或细小障碍也可能被此门槛过滤。
过滤后的原始回波仍作为清除射线的实际终点，不把孤立点被剔除当成空闲证据。
盲区旧记忆和原静态地图不受该新增障碍过滤影响；重启导航生效。

`dynamic_memory_clear_neighbor_radius=0.15 m` 扩大残留清除范围：沿真实空闲射线
检查固定半径邻域，包括斜向邻格，不依赖旧占用格作为触发条件。
相邻体素中心必须距射线不超过该半径且位于回波前方。
默认 `dynamic_memory_clear_confirmations=1`，首次空闲观测即清除，不等第二帧。
不递归扩大，不按时间删除盲区记忆；没有射线覆盖时仍无法清除。
半径不能超过体素边长的 2 倍，设为 `0` 使用原来的精确射线清除。
少量残留时先索引其周边格，避免沿每条射线反复展开大量无障碍空格；
密集场景按需缓存邻格，清除范围、单帧规则和遮挡保护不变。
候选旧障碍不超过 2400 格时，以最多 64 条射线一批的向量距离筛选跳过
无关射线；距离上界同时覆盖清理半径和格子半对角线，避免漏掉贴边相交。
筛选通过的射线仍做完整 DDA 检查，所有回波仍用于保护已有占用。
`Obstacle cloud processing took ...` 表示点云处理超过超时阈值的一半；
`Obstacle input stale: scan_age=... receipt_age=...` 表示数据过期触发停车，
用于区别计算积压与碰撞停车，不通过延长传感器超时掩盖处理延迟。
回波终点附近、当前命中及遮挡后方受保护；回波未选入清除射线预算也仍用于保护。
这是对离散残留的空间容差，附近未被探测到的细小障碍仍可能误清，需现场校准。

默认 `obstacle_slowdown_enabled=false`，取消按障碍距离渐进减速的区间，
安全命令按跟踪器原速度执行。`obstacle_slowdown_distance=0.20 m` 和
`obstacle_min_speed_scale=0.20` 仅在手动开启距离减速后生效。
底盘限幅、转弯、到点减速、最终碰撞检查和近距离急停仍生效。
重规划期间沿用正常跟踪速度，不再设置独立的线速度和角速度上限。
默认近距离急停圆 `dynamic_center_stop_radius=0.40 m`（上一版 `0.45 m`），
半径从 `base_link` 原点计，只作用于前半平面；它不是车边间距。矩形车身、
安全余量及制动检查继续生效，无可用路径时的停车不受该半径控制。
旧的 `dynamic_stop_distance` 参数已移除，近处障碍不再跳过重规划；
停车重试也不要求障碍中心发生移动。A* 起点被膨胀层占用时仍可能无路可走。
RViz 红色球标出车身碰撞触发点的 XY 投影，日志同时记录 `obstacle_map_xy`。

默认开启 `collision_speed_reduction_enabled`：候选轨迹未来碰撞时，依次尝试
原候选速度的 75%、50%、25%，同步缩放线速度和角速度，采用第一个安全结果。
每次都使用原实测速度检查当前运动的制动轨迹。当前车身已碰撞、实际运动无法
安全刹停、输入无效或全部候选不安全时仍停车。

`start_recovery_enabled=true` 时，A* 起点被保守膨胀区覆盖但矩形车身仍安全，
可尝试沿当前朝向前行，最多 `start_recovery_max_distance=1.2 m`。必须整段扫掠、
段末制动和当前实测运动制动均安全，且出口到目标有正常 A* 路径，才执行恢复。
动态重规划等待期间不再仅因起点处于膨胀区而强制停车；开启继续跟踪选项、
已有旧路径且车身检查开启时，继续通过实时碰撞检查发布跟踪命令。
无旧路径、输入失效、规划失败或真实碰撞仍停车。
恢复段速度统一使用启动参数 `max_vx`，按该速度检查全段扫掠和制动；
不在出口前额外减速，接回跟踪器时从上一条已发布速度衔接。
旧参数 `start_recovery_speed`、`dynamic_replan_slow_vx_limit`、
`dynamic_replan_slow_wz_limit` 已移除，不再生效。恢复段保留原形状并显示在跟踪路径中，
不做 shortcut/平滑；优先选择出口及 X/Y 各偏移 `0.10 m` 的九个位置均在膨胀区外
的候选，没有时允许选择出口中心在膨胀区外、整段车身及制动检查通过的候选。
`10 cm` 额外规划余量不再是硬性条件，日志 `buffered_exit=False` 标识该回退。
执行命令仍实时检查，距出口不超过 `0.05 m` 时，用最新代价图和记忆复查当前位置
到后续路径的连接、整条后续路径及实测运动制动；通过后同一周期直接接续跟踪，
不再无条件停车重规划。接续暂未通过时，在实时碰撞检查允许的前提下走到出口，
不再提前 `2 cm` 停车；抵达出口仍不能安全接续时才停车重规划。
偏离短段超过 `0.08 m` 或朝向偏差超过 `0.25 rad` 时停车重规划。
无安全前行出口时保持停车，不清除障碍、不忽略占用格、不自动倒车。

点云超过 `dynamic_obstacle_timeout=0.5 s` 未有效更新、TF/记忆投影不可用、
静态地图未收到、记忆满或预测碰撞时均禁止运动。该超时限制点云新鲜度，不限制
记忆寿命。A* 默认等待 overlay 应用确认再规划，路径平滑也复查障碍记忆。

需要清空误记忆时，在确认环境后执行：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh rosservice call /path_follower_node/clear_obstacle_memory "{}"
```

服务会取消当前目标并停车，之后等待新点云和新目标。仍存在的障碍重新被观测后会
再次加入记忆。该机制不能发现从未看见的侧面障碍，也不估计移动障碍的未来位置。

等待恢复规划或接续失败会停车，并记录 `Navigation stop: ...`；成功接续记录
`Recovery handoff: ... continuing tracking`。控制周期超过 `0.3 s`
会输出耗时告警，帮助区分碰撞停车和下游 `0.5 s` 速度输入看门狗停车。
无路径或到点后持续发布零速度；不通过放宽看门狗或重发旧非零命令处理延迟。

记忆与车身检查均不依赖 A*、PID。后续接入人工势场法时可复用这两层，将新的候选
速度继续送入统一的碰撞检查。控制和障碍参数位于 `config/local_planner.yaml`；
车身尺寸、安全余量、体素分辨率、碰撞采样间距和规划跟踪余量位于
`fw_mid_common_utils/config/collision_geometry.yaml`，由 A* 和局部规划器共用。
A* 膨胀和路径后处理计入这些误差，默认在 `0.05 m` 栅格上膨胀 `0.65 m`，
使规划路径留出车身检查需要的距离。`dynamic_static_filter_radius=0` 仅过滤原
静态占用格覆盖的点，墙边新增障碍也会进入 A* overlay。

## 启动

局部规划 launch 的 `max_vx`（m/s）、`max_wz`（deg/s）必须与底盘一致。完整导航入口会自动同步这两个参数；碰撞检查在限幅之后执行，预测与最终发送的命令保持一致。

`max_vx` 同时作为 PID / Pure Pursuit 普通直线巡航速度。局部节点向控制器显式
传入该值，覆盖旧的 `pid_max_vx` / `pp_max_vx`；不再用前视点间距乘 `k_v`
限制巡航速度。前视点仍用于转向，启动加速、转弯、到点减速和碰撞保护保留。
单独使用控制器库而不传 `cruise_speed` 时，仍兼容原来的比例速度计算。
若碰撞预测把速度降低，日志 `Collision speed reduction` 记录原因、碰撞点及前后速度。

整车通常使用 `fw_mid_bringup/navigation.launch`。单独调试时先启动 A*，再运行：

```bash
roslaunch fw_mid_local_planner local_planner.launch
```

完整导航和两个规划器 launch 均支持 `geometry_config:=/绝对路径/配置.yaml`。
分开启动时必须加载同一份几何配置；它会覆盖局部配置中的同名参数。
修改配置或代码后，先停车再重启导航生效，运行中的节点不会自动更新。

如已有其他节点发布 `body -> base_link`，关闭本 launch 的重复发布：

```bash
roslaunch fw_mid_local_planner local_planner.launch \
  publish_body_to_base_link_tf:=false
```
