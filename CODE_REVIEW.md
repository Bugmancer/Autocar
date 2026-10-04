# 待处理代码问题（2026-10-04）

本文件保留仍有处理价值的问题、复现证据和验证限制。V3 已通过弧长进度规避第 8 项，classic/APF 仍受影响；其余问题尚未修复。本次项目清理没有修改这些边界行为。

审查覆盖项目自有的导航运行层、控制器、全局规划、障碍记忆、车身碰撞、LIO/ICP 集成、底盘适配及启动脚本。第三方 CAN 库、Livox SDK 和 ikd-Tree 未逐行重审，仅检查项目调用边界。

## 已确认问题

### 1. P2：点云降级期间，额外配置的紧急包络可能失效

位置：[dynamic_obstacle_layer.py](src/fw_mid_local_planner/fw_mid_local_planner/dynamic_obstacle_layer.py)，`should_emergency_stop`；[follower_runtime.py](src/fw_mid_local_planner/fw_mid_local_planner/follower_runtime.py)，`publish_degraded_radar_cmd`。

动态层在数据过期时直接返回“未触发紧急停车”，而降级续行恰好发生在过期状态。设置中心包络半径 `0.8 m`、记忆点 `(0.75, 0)`、原速度 `0.2 m/s`，数据从新鲜变为 `0.6 s` 旧后，降级分支可发布 `0.05 m/s`；相同点在新鲜时触发紧急停车。

最终车身检查仍然有效；触发条件是额外配置的包络大于实际扫掠范围，不能据此推断默认半径 `0.25 m` 必然导致碰撞。建议分离“数据是否允许使用”和“记忆点是否落入紧急包络”的判断，并明确降级的停车距离约定。

### 2. P2：首个有效点可能被重复去畸变

位置：[imu_processor.cpp](src/fw_mid_fastlio2/src/map_builder/imu_processor.cpp)，`undistort` 中到达 `points.begin()` 后的 `break`。

这里只退出点循环，外层 IMU 区间遍历继续。首个有效点因过滤而具有较大的时间偏移时，同一点会在多个更早区间再次补偿。按代码公式做一维匀速复现：扫描结束 `100 ms`，点偏移 `25 ms`，区间头为 `20/10/0 ms`，速度 `1 m/s`，原点坐标 `1 m`，期望结果 `0.925 m`，重复补偿后为 `0.775 m`。

建议点云耗尽后退出整个遍历，并补覆盖首点时间偏移的回归测试。该项已核对控制流和数值公式，尚未通过 ROS/C++ 点云回放验证。

### 3. P2：路径重采样可将无碰撞折线变成穿障折线

位置：[path_processing.py](src/fw_mid_local_planner/fw_mid_local_planner/path_processing.py)，`resample_path` 与 `process_path`。

复现使用当前配置参数：原路径 `[(0,0), (0.06,0), (0.06,0.06)]`，占用区域 `0.02 <= x <= 0.05 且 0.01 <= y <= 0.04`。开启捷径和平滑、重采样间距 `0.10 m` 后，输出为 `[(0,0), (0.06,0.06)]`。用相同 `0.05 m` 步长检查，原路径通过，输出路径失败。

重采样未保留拐点，且最后返回前缺少整路复核。最终车身检查仍可阻止危险运动，但可能反复停车或重规划。建议保留必要拐点，或对每次重采样结果复核并回退到最近一次验证通过的路径。

### 4. P2：经典控制器提高到 20 Hz 后可能无法起步或转向

位置：[pid_controller.py](src/fw_mid_controller/fw_mid_controller/pid_controller.py) 和 [pure_pursuit_controller.py](src/fw_mid_controller/fw_mid_controller/pure_pursuit_controller.py)，`compute` 末尾的死区及 `prev_cmd` 更新。

当前参数下，`dt=0.05 s` 时的单步加速度增量小于输出死区。死区将输出归零后又将零保存为下一次的内部状态，导致速度斜坡无法累积。连续调用两种控制器各 100 次均得到零速；相同场景在 `dt=0.1 s` 下可正常输出。

默认 `10 Hz` 未触发此复现，但 `control_rate` 允许设置为更高频率。建议像 APF 一样保留死区处理前的内部斜坡，同时在真正停车或下游降速时同步实际速度。

### 5. P2：规划服务返回没有超时边界

位置：[follower_runtime.py](src/fw_mid_local_planner/fw_mid_local_planner/follower_runtime.py)，`_plan_worker` 的 `ServiceProxy` 调用及恢复出口规划调用。

`planner_service_timeout` 只约束快照确认和等待服务出现，不约束 `client(request)` 返回。离线替身复现中，配置超时 `0.02 s`，调用阻塞 `0.1 s` 后工作线程仍存活，`waiting_for_plan` 仍为真。服务挂起时节点会长期等待，正常重试被挡住；继续行驶与否仍取决于旧路径及最终安全检查。

建议为请求设置独立截止时间，超时后结束等待状态，并使用规划代次丢弃迟到响应，避免积累无法回收的工作线程。

### 6. P2：A* 角点坐标浮点回算会落入相邻栅格

位置：[astar_planner_node.py](src/fw_mid_global_planner/scripts/astar_planner_node.py)，`grid_to_world` 和 `world_to_grid`。

当前地图 `origin=[-26.3,-13.3,0]`、`resolution=0.05` 下，栅格 `(54,54)` 输出 `(-23.6,-10.600000000000001)`，再取 `floor` 得到 `(53,53)`。局部规划器二次检查时可能把贴近膨胀边界的合法路径判为占用。

建议区分路径使用的栅格中心坐标与区域边界坐标；不要直接修改所有 `grid_to_world` 调用，因为禁行区边界也依赖该函数。

### 7. P2：乱序点云可清空障碍记忆，并让最终检查重新放行

位置：[dynamic_obstacle_layer.py](src/fw_mid_local_planner/fw_mid_local_planner/dynamic_obstacle_layer.py)，`cloud_cb` 的小幅时间回退分支。

时间戳回退不超过 `0.5 s` 时，代码在检查点云内容前直接清空记忆，但未使 `last_update_time`、`last_received`、`projection_valid` 失效，也未更新清除代次。若该帧为空则直接返回；下一次投影得到空障碍集，数据新鲜度却仍可通过。

复现保留默认 `timeout=0.5 s`、`point_step=2`，模拟当前 ROS 时间 `10.0 s`。先接收同时间戳的三个支持点 `(0.44,0,0.4)`、`(0.46,0.01,0.4)`、`(0.48,-0.01,0.4)`，每点重复两次以保留默认抽样。真实记忆产生两个占用体素，真实 `publish_cmd(0.2,0,0)` 返回拒绝并发布零速。再接收时间戳 `9.9 s` 的空云并更新投影，记忆变为 `[]`、`is_fresh()` 仍为真，同一命令被放行并发布 `0.2 m/s`。此过程没有自由射线证据或显式清除请求。

触发需要允许时间窗内的乱序或时间回退帧；正常严格递增输入不触发。建议丢弃乱序帧并保留障碍；真正的时钟重置应使输入失效，并通过完整的新会话流程恢复，不能静默清空后沿用旧的新鲜状态。

### 8. P2：classic/APF 按车头方向裁剪路径会丢掉必要的后方绕行段

位置：[follower_runtime.py](src/fw_mid_local_planner/fw_mid_local_planner/follower_runtime.py)，`_control_loop` 中 `uses_path_progress` 为假时的路径裁剪条件。

`forward_projection < 0` 只说明路径点在当前车头后方，不能说明已经走过。机器人和目标均朝东、但机器人位于仅西侧开放的 U 形障碍内时，合法路线需要先转向西侧出口；当前循环会删除这段尚未执行的路线。

复现机器人位姿 `(0,0,0)`、目标 `(2,0)`，路径经西侧 `(-1.5,0)`、`(-1.5,-2)` 和南侧 `(2,-2)` 绕出。障碍查询使用东侧墙 `abs(x-0.9)<=0.6 且 -1.6<=y<=1.6`，以及南北墙 `abs(abs(y)-1)<=0.6 且 -1.2<=x<=1.5`。生产 `process_planned_path` 完成默认捷径、平滑和重采样后生成 86 点，用生产 `path_collision_free` 检查通过。生产 `_control_loop` 一个周期后只剩 39 点，首点约为 `(0.0940,-1.9998)`，当前位置连接到剩余路径的检查失败。

这会丢失可行路线，导致错误接近侧墙或反复停车、重规划；最终车身碰撞检查仍可拦截危险命令。V3 已使用路径投影和弧长进度并绕过该裁剪；classic/APF 仍沿用原逻辑。

### 9. P2：定时发布可能让局部代价图退回旧障碍状态

位置：[astar_planner_node.py](src/fw_mid_global_planner/scripts/astar_planner_node.py)，`publish_visuals`；接收位置为 [follower_runtime.py](src/fw_mid_local_planner/fw_mid_local_planner/follower_runtime.py) 的 `costmap_cb`。

定时线程取得旧 `current_costmap` 引用后，动态障碍回调可以更新共享地图、发布新地图并确认快照；定时线程随后仍会构造并发布旧地图。局部 `costmap_cb` 全量接收，因而可回退到旧状态。

使用真实 `publish_visuals`、`dynamic_points_cb` 和消息构造方法，以线程事件固定顺序：新地图在 `11.0 s` 将单元设为 `100` 并发送 ACK，定时线程在 `12.0 s` 发布旧地图的 `0`。实际发布结果为 `[(stamp=11.0, cell=100), (stamp=12.0, cell=0)]`，服务器当前单元仍为 `100`，ACK 仍为 `11.0`。旧内容甚至带有更新的消息时间戳，因此仅比较 `header.stamp` 不能解决。

影响是局部代价图及可视化暂时回退，依赖该图的查询会读到旧占用。本地动态记忆和最终车身检查仍独立存在；ACK 对“服务端已应用快照”的含义仍成立，不能据此说后续 A* 必然使用旧图。建议串行保护地图内容与发布顺序，或携带明确的内容版本并在接收端拒绝旧版本；仅在取得数组引用时加锁不够。

## 验证与待确认项

第 7 至 9 项已用 ROS 替身调用生产方法离线复现并独立复跑：分别替代消息读取/TF/时钟、隔离传感器及命令发布、替代障碍膨胀并固定线程顺序。当前自动化验证范围与运行命令见 [README](README.md#模块与验证)，测试通过不代表上述边界已修复。未执行 Noetic/C++ 构建、ROS 联调、SocketCAN 或实车验证。

- [localizer_node.cpp](src/fw_mid_localizer/src/localizer_node.cpp) 的 ICP、输入和状态心跳共用单线程 `ros::spin()`。一次配准若超过默认 `pose_timeout=0.5 s`，会延迟心跳并触发安全停车。需测量实际地图、点数和车机上的配准时延。
- [check_mid360_topics.py](src/livox_ros_driver2/scripts/check_mid360_topics.py) 的话题健康检查验证时间戳递增和接收频率，没有验证采样年龄。旧时间戳持续递增也可通过频率检查；下游定位仍另行检查采样年龄。

- 障碍投影分别查询 `map <- memory`、`base <- memory` 的 `Time(0)`，车体位姿另查 `map <- base`；定位校正在当前时间发布，里程计使用扫描时刻，几次查询可能采用不同的共同时间。需在真实 tf2 中注入错开的变换时间及定位校正变化，验证同一障碍与车体是否一致；该风险尚未确认。
