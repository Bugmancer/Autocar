# Autocar_v1

`Autocar_v1` 是面向车载计算机的 ROS1 Noetic 导航工作区，完成 MID360 驱动、FAST-LIO2 里程计、PCD 地图定位、A* 全局规划、局部路径跟踪、动态障碍处理和 CAN 底盘控制。电脑端只负责离线建图、优化和导出匹配的 PCD 与 PGM/YAML 地图，不再通过 UDP 向车机发送速度。

本文以当前 ROS1 源码、配置和启动方式为准。

整理日期：2026-09-26。

> [!CAUTION]
> `navigation.launch` 默认 `enable_can=false`，这是刻意设置的安全边界。当前尚未确认“RViz 下发目标 -> 自主行驶 -> 到点停车”的整车完整闭环，动态避障也仍需实车确认。首次运行必须架空轮组或置于受控场地，并保留人工急停。

## 1. 项目边界

- 当前运行工程是本目录下 `src/` 中的 ROS1 catkin 包。
- ROS1 车机通过 `/cmd_vel -> JSON -> can0` 直接控制底盘。
- 当前项目没有注册自己的开机自启动。车机上学长遗留的 Livox、FAST-LIO、move_base、CAN 或网络自启动仍可能与本项目争用节点、端口、TF 和硬件，启动前必须检查。
- ROS 包、Livox SDK2 和 `python-can` 均随项目保存；ROS Noetic、编译器、PCL、Eigen、yaml-cpp、OpenCV 等属于系统依赖。

## 2. 导航与控制总链路

```text
MID360 (192.168.1.118)
  |  /livox/lidar  livox_ros_driver2/CustomMsg, 实测约 10 Hz
  |  /livox/imu    sensor_msgs/Imu,             实测约 200 Hz
  v
fw_mid_fastlio2
  |  /fastlio2/body_cloud
  |  /fastlio2/lio_odom
  |  TF: lidar -> body
  v
fw_mid_localizer + map.pcd
  |  ICP 粗配准/精配准
  |  /localizer/localization_valid
  |  TF: map -> lidar
  v
TF: map -> lidar -> body -> base_link
  |
  v
fw_mid_local_planner <---- GetPlan 请求/响应 ----> fw_mid_global_planner
  |                                              map_2d.yaml/map_2d.pgm
  |                                              静态膨胀 + 动态 overlay
  |
  +-- 路径跟踪和障碍处理
         目标: /move_base_simple/goal（兼容 /goal_pose）
         路径处理 + PID/Pure Pursuit + 会话障碍记忆/重规划
         发布前: 车身扫掠及制动碰撞检查
         输出: /cmd_vel (m/s, rad/s)
                    |
                    v
         cmd_vel_to_command_node.py
         rad/s -> deg/s，仅在这里转换一次
         /fw_mid/command_dict (JSON)
                    |
                    v
         can_driver_node.py -> can0 -> 底盘
```

完整编排入口是：

```text
src/fw_mid_bringup/launch/navigation.launch
```

它包含 Livox、FAST-LIO2、ICP 定位、A*、局部规划以及 Twist/CAN 适配器。各节点由 roslaunch 启动，并通过数据就绪和有效性检查衔接；include 的书写顺序不表示等待前一个模块就绪。A* 本身不查询 TF，起点和目标由局部规划器转换/组织成 `map` 坐标后提交。CAN 驱动默认不启动。

## 3. 快速启动

以下命令应在车机 Linux 环境执行。项目在移植和实测时位于 `/home/robot/Autocar_v1`；若目录不同，请同步修改文中的绝对路径。

### 3.1 首次构建（只做一次）

目标环境需要 `/opt/ros/noetic/setup.bash`、catkin、Python 3 和 C++14 编译器。依赖按当前各包的 `package.xml` 和 `CMakeLists.txt` 核对：

| 类别 | 依赖 |
| --- | --- |
| ROS | `roscpp`、`rospy`、标准消息、`tf2_ros`、`message_filters`、`message_generation`、`message_runtime`、`pcl_ros`、`pcl_conversions`、`rosbag`、`rviz` |
| C++ 系统库 | PCL、Eigen3、yaml-cpp、Boost；Livox 构建还探测 APR |
| Python | NumPy、OpenCV (`cv2`)、PyYAML、setuptools |
| 网络/CAN | `iproute2`、`iputils-ping`、sudo、Linux SocketCAN |
| 项目内依赖 | Livox SDK2、`python-can`，随包构建/安装；FAST-LIO 的 SO(3) 运算使用项目内 Eigen 实现，无需系统 Sophus |

`scripts/ros1.sh` 会清除外部 ROS/colcon 工作区环境，只叠加系统 Noetic 和本工作区，避免误加载旧工程。

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh catkin_make -DCMAKE_BUILD_TYPE=Release -j2
```

若工作区从另一台机器或另一条绝对路径复制而来，不应复用原 `build/`、`devel/`、`install/` 缓存；应在目标路径重新生成。

### 3.2 启动前准备

1. 停止其他 Livox 可视化工具，并确认没有另一套 Livox 驱动或旧导航节点正在运行。
2. 确认急停有效、遥控器可接管，首次运行时架空轮组。
3. 确认地图与当前位置对应，且 `.ros/initial_pose.txt` 中保存的初值对应本次停车位置和朝向（保存方法见第 3.6 节）。
4. 确认 `can0` 接口名称和 500000 bit/s 波特率正确。

可先查看可能冲突的节点：

```bash
bash scripts/ros1.sh rosnode list
```

ROS master 未启动时该检查会连接失败；随后运行 `roslaunch` 会自动启动本机 master，通常不必另开 `roscore`。若有意使用外部 master，请先核对终端继承的 `ROS_MASTER_URI`、`ROS_IP` 和 `ROS_HOSTNAME`；环境包装器不会清空这三项。

`navigation.launch` 不负责创建或设置 SocketCAN 设备。系统未提前配置时执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

### 3.3 直接启动完整导航、RViz 和 CAN

推荐使用网络包装器。它会在当前终端请求 sudo、准备 MID360 网络并通过 ARP 检查后，再以普通用户启动 ROS；不要执行 `sudo roslaunch`。

下面的启动命令从 `.ros/initial_pose.txt` 读取上次保存的初值，无需手动填写六个数值。关机前，在车辆停稳、当前定位正常且定位节点仍运行时，另开终端保存一次：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh python3 scripts/save_initial_pose.py
```

下次使用同一地图、同一停车位置和朝向时启动：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch enable_can:=true start_rviz:=true max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

这条命令会同时启动 MID360、FAST-LIO2、ICP 定位、A*、局部规划、RViz、Twist/JSON 适配器和 CAN 驱动。最终速度限幅为 `0.05 m/s`、`5 deg/s`，横移速度固定为零。节点启动后先等待定位有效，再在 RViz 中下发目标。

建议把整条 `roslaunch` 写在一行。此前对话中曾因反斜杠后出现空行或空格，使后续参数没有传给 `roslaunch`。

首次尚无 `.ros/initial_pose.txt` 时，需要先使用现场可信的近似初值完成一次定位，再运行保存脚本；脚本不能在尚未定位时自动求出初值。文件不存在或为空时，不要执行上述重启命令。读取文件由命令中的 `$(cat .ros/initial_pose.txt)` 完成；省略 `initial_pose` 参数时，launch 仍默认使用 `[0,0,0,0,0,0]`，仅适合接近地图原点的情况。

#### 通过远程 SSH 启动和导航

远端电脑只需要 SSH 客户端，ROS 命令全部在车机执行。先在车机用 `ip -br -4 addr` 确认无线网卡或管理网口地址；下方 `CAR_IP` 替换成该地址。不要使用雷达地址 `192.168.1.118`；也避免通过会被启动脚本迁移的 `192.168.1.50` 登录，否则启动时 SSH 可能断开（见第 8 节）。

**窗口一：连接车机并启动导航。** 在电脑终端执行：

```bash
ssh robot@CAR_IP
```

登录后执行以下命令。建议车机安装 `tmux`（缺少时执行 `sudo apt install tmux`），以便断线后重新连接原来的导航终端。若已有导航在运行，连接原会话即可，不要重复启动。

```bash
tmux new -s autocar_nav
cd /home/robot/Autocar_v1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
test -s .ros/initial_pose.txt && bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch enable_can:=true start_rviz:=false max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

这里使用第 3.3 节保存的初值；文件不存在或为空时命令不会启动，需先按该节完成首次定位。CAN 接口仍需按第 3.2 节准备。`start_rviz:=false` 适合没有图形桌面的 SSH，限速与本地启动相同。ROS 只在车机本地通信，因此不需要 SSH 图形转发，也不需要在电脑上启动 `roscore`。若项目原先使用外部 ROS master，请先统一所有终端配置，不要混用。

**窗口二：检查定位并发布目标。** 在电脑另开终端执行 `ssh robot@CAR_IP`，登录后执行：

```bash
cd /home/robot/Autocar_v1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
bash scripts/ros1.sh rosservice call /localizer/relocalize_check "{code: 0}"
bash scripts/ros1.sh rosrun tf tf_echo map base_link
```

确认返回 `valid: True`，TF 位置与现场相符，并完成第 3.4 节检查；`tf_echo` 持续输出，按 `Ctrl-C` 结束查看。然后修改下面的 `goal_x`、`goal_y` 为地图中可通行的目标坐标，单位为米。示例 `(2.0, 0.0)` 仅演示格式，不表示现场一定能通行：

```bash
goal_x=2.0
goal_y=0.0
bash scripts/ros1.sh rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

发布一次目标后会自动规划并导航，无需另发“开始”或速度命令。窗口一显示 `Goal accepted`、路径接受日志，到点显示 `Goal reached; waiting for the next goal`。到点后修改 `goal_x`、`goal_y`，再次执行发布命令即可继续导航；行驶中发布则替换当前目标，不会排队。当前按 XY 距离判定到点，不保证最终朝向。

窗口二还可执行 `bash scripts/ros1.sh rostopic echo /cmd_vel` 查看导航速度输出，按 `Ctrl-C` 只结束查看。要停止导航，应回到窗口一，在运行 launch 的终端按 `Ctrl-C`。`tmux` 中按 `Ctrl-B` 后按 `D` 仅分离会话，SSH 断开也不会使该会话中的导航停止；重新 SSH 登录后用 `tmux attach -t autocar_nav` 返回。正常关机前仍可在窗口二按第 3.6 节保存定位初值。

### 3.4 启动后的放行检查

在其他终端通过同一个环境包装器检查：

以下 `rostopic hz`、`rostopic echo` 和 `tf_echo` 会持续运行；分别开终端，或检查完一项按 `Ctrl-C` 后再执行下一项。

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh rostopic hz /livox/lidar /livox/imu
bash scripts/ros1.sh rostopic hz /fastlio2/lio_odom /fastlio2/body_cloud
bash scripts/ros1.sh rosservice call /localizer/relocalize_check "{code: 0}"
bash scripts/ros1.sh rosrun tf tf_echo map base_link
bash scripts/ros1.sh rostopic echo /cmd_vel
bash scripts/ros1.sh rostopic echo /fw_mid/command_dict
```

放行前至少应满足：

- `/livox/lidar` 与 `/livox/imu` 连续，典型实测频率约为 10 Hz / 200 Hz。
- `/fastlio2/lio_odom` 和 `/fastlio2/body_cloud` 连续且无 NaN、跳变或倒退时间戳。
- `/localizer/relocalize_check` 返回 `valid: True`。
- `map -> lidar -> body -> base_link` 全链可查询且时间新鲜，每个 child 只有一个 TF 父节点。
- 尚未发送目标时，不应有非零 `/cmd_vel`，JSON 命令应持续为 `gear=1` 且三个速度为零。局部规划器无路径时不会持续刷新零 `Twist`，因此启动后再订阅 `/cmd_vel` 可能暂时没有消息，这不等于底盘适配器没有发送停车命令。

定位无效时到达的目标会被直接拒绝，不会在定位恢复后自动执行。必须先等到 `valid: True`，再重新发送目标。

`/localizer/relocalize` 返回 `success: true` 只表示地图和请求已被接受，不表示 ICP 已收敛。提交后可先等待约 5–10 秒，或观察 `relocalization converged` 日志，再检查 `relocalize_check`；等待时间本身不代表定位有效。下面仅保留人工重定位的服务示例，数值是历史实验室车位记录，需按现场修改；日常重启使用第 3.3 节的保存文件命令：

```bash
bash scripts/ros1.sh rosservice call /localizer/relocalize "{pcd_path: '/home/robot/Autocar_v1/src/fw_mid_localizer/maps/underground/map.pcd', x: 2.503, y: -0.023, z: -0.0395, yaw: -0.0175, pitch: 0.713, roll: 0.030}"
bash scripts/ros1.sh rosservice call /localizer/relocalize_check "{code: 0}"
```

### 3.5 RViz 下发目标

项目 RViz 配置位于 `src/fw_mid_localizer/rviz/localizer.rviz`，`start_rviz:=true` 会自动加载它。

如果启动导航时使用了 `start_rviz:=false`，可在另一个终端单独打开同一套 RViz 配置：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh rosrun rviz rviz -d "$(rospack find fw_mid_localizer)/rviz/localizer.rviz"
```

1. Fixed Frame 使用 `map`。
2. 确认 `Map Cloud`、`Static Grid Map`、`Inflated Costmap`、`Base Link Axes` 和路径显示正常。
3. 使用 `2D Nav Goal`，在膨胀代价图中的空闲区按住并拖动设置目标朝向。
4. 蓝线是 A* 原始路径，绿线是局部规划器处理后的路径，绿色/橙色箭头是当前 lookahead 的期望前进/后退方向。
5. 箭头和 `/cmd_vel` 表示规划器当前输出的期望运动方向和速度；实际运动还取决于 CAN、急停和底盘控制权。

到达目标点后，车辆停车并清除旧的局部路径，导航节点继续运行、等待下一个目标。直接再次使用 `2D Nav Goal` 下发新目标即可从当前位置重新规划并行驶，无需重启 launch 或重新定位。也支持继续向 `/move_base_simple/goal`（或兼容话题 `/goal_pose`）发布 `map` 坐标系的 `geometry_msgs/PoseStamped`。同次导航的障碍物记忆会保留；新目标执行时仍检查定位、点云和碰撞安全。

没有活动目标、定位无效、规划失败或到点停车时，方向箭头会被删除或到期消失，不是显示故障。颜色接口支持前进/后退区分，但当前 PID 和 Pure Pursuit 跟踪器按前进路径控制，不代表已实现倒车规划。

起点或目标落入膨胀障碍区时，A* 会返回空路径。局部规划器会对起点尝试第 7.2 节的受限前行恢复：必须车身及整段制动扫掠安全，且出口能规划到目标，否则保持停车。不能通过随意缩小车体尺寸或清除真实障碍来强行得到路径。

### 3.6 停止与重新启动

正常结束时在导航 launch 的终端按 `Ctrl-C`，等节点退出并确认车辆停止；CAN 节点正常退出会发送停车帧。紧急情况直接使用实物急停。不要把向 `/cmd_vel` 手发一次零速当作取消导航，仍在运行的跟踪器会继续发布后续命令。

重新启动前确认旧 Livox/CAN 节点已退出，再启动一次并重新检查定位、重新发送目标。网络地址迁移不会随 launch 退出而回滚。LIO 遇到时间回退（例如循环回放 bag）也需要重启，不能继续使用前一时钟周期的滤波状态。

#### 保存下次启动使用的初值

当前定位正常时，把车停在下次准备启动的位置，保持定位节点运行，在另一个终端执行：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh python3 scripts/save_initial_pose.py
```

脚本读取 `/localizer/pose`，确认定位有效且数据新鲜，输出可直接使用的 `initial_pose:="[x,y,z,yaw,pitch,roll]"`，同时保存六个数值到 `.ros/initial_pose.txt`。角度单位是弧度，脚本已按项目的 `Rz(yaw)*Rx(roll)*Ry(pitch)` 约定转换。默认最多等待 10 秒，失败时返回非零退出码且保留原有文件。可用 `--timeout 30` 延长等待，用 `--output /路径/initial_pose.txt` 指定保存位置。

下次在**同一地图、同一停车位置和朝向**启动时，直接读取保存值：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch enable_can:=true start_rviz:=true max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

这份脚本用于保存当前已定位的位置，不执行无初值的全地图搜索。车辆关机后被挪动，或切换了地图时，旧初值可能不再适用。

### 3.7 只运行地图重定位并输出位姿

独立入口是 `src/fw_mid_bringup/launch/relocalization.launch`，启动 MID360、FAST-LIO2、PCD 地图配准和 RViz，不启动规划器或 CAN 控制。先停止包含同名定位节点的 `navigation.launch`，再执行：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh catkin_make --pkg fw_mid_localizer fw_mid_bringup -DCMAKE_BUILD_TYPE=Release -j2
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup relocalization.launch initial_pose:="$(cat .ros/initial_pose.txt)"
```

上面的命令同样读取第 3.6 节保存的初值，文件需对应当前地图、位置和朝向。`initial_pose` 顺序为 `[x,y,z,yaw,pitch,roll]`，单位为米和弧度；当前实现是有初值的粗/精两级 ICP，不包含无初值的全地图搜索。默认读取 `src/fw_mid_localizer/maps/underground/map.pcd`，可用 `map_pcd:=/绝对路径/map.pcd` 替换。`map_2d.yaml` 是导航用二维地图，不能代替三维 PCD 参与此配准。

终端在每次有效配准后打印位置 `xyz_m`、姿态 `rpy_rad`、四元数 `q_xyzw` 和匹配误差 `fitness_m2`。`rpy_rad` 按标准 `Rz(yaw)*Ry(pitch)*Rx(roll)` 表示姿态；历史初值接口保留 `Rz(yaw)*Rx(roll)*Ry(pitch)` 约定，两者有非零横滚/俯仰时不能直接互填。

| 输出话题 | 含义 |
| --- | --- |
| `/localizer/pose` | `PoseStamped`，融合最新 LIO 里程计的当前位姿 |
| `/localizer/icp_pose` | `PoseStamped`，每次成功 ICP 配准得到的位姿，时间戳对应扫描时刻 |
| `/localizer/aligned_cloud` | 同次配准后的扫描点云，已转换到 `map`；RViz 显示为绿色 |
| `/localizer/fitness_score` | 精配准平均最近邻距离平方，单位 m²，越小越好，不是置信概率 |
| `/localizer/localization_valid` | 当前定位有效性；输入断流或配准超时后为 `false` |

```bash
bash scripts/ros1.sh rostopic echo /localizer/pose
bash scripts/ros1.sh rostopic echo /localizer/localization_valid
```

以上两条命令分别在终端运行。位姿参考点是 **body/IMU**，坐标位于 **map**，姿态为四元数；它不是雷达光心或底盘 `base_link`。断流后停止发布新位姿，但 RViz 可能保留最后一帧，使用者必须同时检查有效性和时间戳。

雷达和 LIO 已单独启动时，给入口增加 `start_lidar:=false start_lio:=false`，并改用 `scripts/ros1.sh` 包装器。`start_rviz:=false` 可关闭 GUI，`print_pose:=false` 可关闭位姿日志。运行中更换地图或初值仍使用第 3.4 节的 `/localizer/relocalize` 服务，接受请求后需等待定位重新有效。

## 4. 完整入口参数

`fw_mid_bringup/launch/navigation.launch` 的当前参数如下：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `map_pcd` | 包内 `map.pcd` | ICP 使用的三维地图 |
| `map_yaml_path` | 包内 `map_2d.yaml` | A* 使用的二维地图元数据 |
| `initial_pose` | `[0,0,0,0,0,0]` | `map -> body` 初值，顺序为 x/y/z/yaw/pitch/roll |
| `start_lidar` | `true` | 是否启动项目内 MID360 驱动 |
| `takeover_video_address` | `true` | 是否允许把指定静态地址从图传网口迁到雷达网口 |
| `start_lio` | `true` | 是否启动项目内 FAST-LIO2 |
| `enable_can` | `false` | 是否启动 CAN 驱动；Twist->JSON 适配器仍会运行 |
| `start_rviz` | `false` | 是否加载项目 RViz 配置 |
| `can_interface` | `can0` | SocketCAN 接口 |
| `max_vx` | `0.05` | 直线巡航速度及最终纵向速度上限，m/s；转弯、到点和碰撞保护可降速 |
| `max_wz` | `5.0` | 最终角速度限幅，deg/s |
| `body_to_base_link` | `0.28991 0 -0.81317 0 -0.785398 0` | 静态外参 x/y/z/yaw/pitch/roll |
| `lio_config` | 包内 `lio.yaml` | FAST-LIO2 参数文件 |
| `localizer_config` | 包内 `localizer.yaml` | ICP 参数文件 |
| `planner_config` | 包内 `local_planner.yaml` | 路径跟踪和动态障碍参数 |
| `geometry_config` | `fw_mid_common_utils/config/collision_geometry.yaml` | A*、路径处理、车身碰撞检查和 RViz 共用的几何参数 |

`body_to_base_link` 是从来源工程保留的安装值，必须结合实车尺寸和坐标轴重新确认。若已有其他节点发布同一个 TF，应在单独启动局部规划时设置 `publish_body_to_base_link_tf:=false`，完整入口则应先消除重复发布者。

如果 Livox 已由本项目的另一条 launch 启动，完整入口应使用 `start_lidar:=false`，不能并行启动第二个驱动：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh roslaunch fw_mid_bringup navigation.launch start_lidar:=false enable_can:=true start_rviz:=true max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

上例读取上次保存的初值，无需手填。如果 LIO 也已单独运行，另加 `start_lio:=false`。

## 5. 坐标系与定位

| TF | 发布者 | 含义 |
| --- | --- | --- |
| `map -> lidar` | `fw_mid_localizer` | PCD 地图到 LIO 局部世界系的变换，只在 ICP 有效时发布 |
| `lidar -> body` | `fw_mid_fastlio2` | LIO 估计的车体位姿；`lidar` 在这里是 LIO 的局部世界帧 |
| `body -> base_link` | `fw_mid_local_planner` launch 中的静态发布器 | IMU/body 到导航车体参考点的安装外参 |

关键约束：

- 导航、二维地图和目标统一使用 `map`。
- `/fastlio2/lio_odom.header.frame_id=lidar`，`child_frame_id=body`。
- `/fastlio2/body_cloud` 在 `body` 帧，局部规划器将观测存入 `lidar` 世界系的会话记忆，再转换到 `base_link` 和 `map`。
- 每个 TF child 只能有一个父节点。旧 FAST-LIO 或旧静态 TF 节点并存会造成跳变。
- 定位有效性同时要求 ICP 已收敛、最近同步输入不超过 0.5 s、最近配准不超过 3 s。
- 完整导航要求定位有效且 `map -> base_link` TF 不陈旧；否则局部规划器发布零速。

## 6. 核心话题与服务

| 名称 | 类型 | 生产者 -> 消费者 / 说明 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox -> FAST-LIO2 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox -> FAST-LIO2 |
| `/fastlio2/body_cloud` | `sensor_msgs/PointCloud2` | FAST-LIO2 -> ICP、动态障碍 |
| `/fastlio2/world_cloud` | `sensor_msgs/PointCloud2` | LIO 局部世界系点云，主要用于观察 |
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | FAST-LIO2 -> ICP |
| `/fastlio2/lio_path` | `nav_msgs/Path` | LIO 轨迹可视化 |
| `/localizer/map_cloud` | `sensor_msgs/PointCloud2` | ICP 加载的 PCD 地图，锁存发布 |
| `/localizer/localization_valid` | `std_msgs/Bool` | 定位健康联锁 -> 局部规划器 |
| `/localizer/relocalize` | `fw_mid_localizer/Relocalize` | 提交 PCD 与 `map -> body` 初值 |
| `/localizer/relocalize_check` | `fw_mid_localizer/IsValid` | 查询最近一次重定位是否已收敛 |
| `/astar_planner_node/get_plan` | `nav_msgs/GetPlan` | 局部规划器调用 A* |
| `/astar_planner_node/map` | `nav_msgs/OccupancyGrid` | 原始二维占用栅格，锁存发布 |
| `/astar_planner_node/costmap` | `nav_msgs/OccupancyGrid` | 车体膨胀和动态 overlay 后的代价图 |
| `/astar_planner_node/visual_plan` | `nav_msgs/Path` | A* 原始路径 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 标准目标输入 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 兼容旧接口的目标输入 |
| `/local_planner/global_plan` | `nav_msgs/Path` | 平滑、重采样后的当前跟踪路径 |
| `/local_planner/desired_direction` | `visualization_msgs/Marker` | 当前 lookahead 方向箭头 |
| `/local_planner/dynamic_obstacle_points` | `geometry_msgs/PoseArray` | 本次会话障碍记忆的完整快照 -> A* overlay |
| `/astar_planner_node/dynamic_overlay_stamp` | `std_msgs/Time` | A* 已应用的障碍快照时间戳，供规划请求确认 |
| `/dynamic_obstacles_markers` | `visualization_msgs/MarkerArray` | 绿色近期观测、橙色历史记忆的障碍体素 |
| `/local_planner/collision_check` | `visualization_msgs/MarkerArray` | 当前车身轮廓及预测运动中的车身轮廓 |
| `/local_planner/collision_blocked` | `std_msgs/Bool` | 当前速度是否被碰撞检查或障碍输入联锁拦截 |
| `/path_follower_node/clear_obstacle_memory` | `std_srvs/Trigger` | 停车、取消目标并清空本次障碍记忆和 overlay |
| `/cmd_vel` | `geometry_msgs/Twist` | 局部规划器 -> Twist/JSON 适配器 |
| `/fw_mid/command_dict` | `std_msgs/String` | JSON 命令 -> CAN 驱动 |
| `/fw_mid/feedback/velocity` | `std_msgs/Float32MultiArray` | `[gear,vx,vy,wz_deg]` |
| `/fw_mid/feedback/bms` | `std_msgs/Float32MultiArray` | `[voltage,current,capacity]` |

## 7. 规划、避障与控制实现

### 7.1 A* 全局规划

`fw_mid_global_planner/scripts/astar_planner_node.py` 读取 YAML/PGM，将未知区默认视为不可通行。A*、路径后处理、车身碰撞检查和 RViz 共用 `src/fw_mid_common_utils/config/collision_geometry.yaml`：车体为 `0.68 m x 0.55 m`，每侧安全余量为 `0.02 m`，带余量的矩形为 `0.72 m x 0.59 m`。

A* 用该矩形的包围圆，再计入障碍体素、栅格离散和碰撞采样误差，推导静态与动态障碍膨胀半径。额外跟踪余量 `planner_tracking_clearance` 已由 `0.02 m` 减至 `0`；膨胀圆也改为按实际米制半径判断，只有数组范围向上取整，不再把整个圆半径放大到整格。在当前 `0.05 m` 栅格下，两者计算半径约为 `0.612 m`，轴向最外占用格中心距障碍格中心为 `0.60 m`（此前为 `0.65 m`），斜向按圆内格中心判断。这个半径从障碍格中心算起，不是车壳与障碍之间的净距离。地图边界也按占用膨胀，实际车身尺寸、每侧 `0.02 m` 余量和制动碰撞检查保持不变。A* 使用包围圆、局部检查使用有朝向的矩形，并非画成同一种形状。停车重启导航后参数生效，RViz 的 `Inflated Costmap` 会同步缩小。

规划使用八邻域 A*，禁止对角穿过障碍角；返回完整栅格路径，再交由局部规划器检查和后处理，避免原先每 4 格抽样连线切入障碍角。

本次会话观测到的新增障碍按上述动态膨胀半径写入临时代价图。overlay 是完整快照，默认 `dynamic_overlay_timeout=0`，不会因障碍进入视野盲区而超时消失；新快照替换旧快照，空快照清空 overlay。只过滤原始静态占用格已经覆盖的障碍点（`dynamic_static_filter_radius=0`），不再过滤墙边额外 `0.18 m` 范围内的点，避免局部检查看到了障碍而 A* 没收到。原始 PCD、PGM 和 YAML 文件均不修改。起点或终点处于地图外、膨胀后占用区时，规划返回空路径。

A* 收到快照后立即更新并发布 costmap，再发布 `/astar_planner_node/dynamic_overlay_stamp`。局部规划器默认等待该确认后才请求路径，路径 shortcut 和平滑也会复查当前障碍记忆。

当前 `GetPlan` 请求中的 `tolerance` 没有参与 A* 计算，目标姿态也不参与路径搜索。局部规划器以目标 XY 距离判断到点，因此不能依靠目标箭头要求车辆在终点达到指定朝向。

### 7.2 局部规划

`fw_mid_local_planner` 完成：

- 接收目标并异步调用 `/astar_planner_node/get_plan`。
- 对路径做 shortcut、平滑、0.10 m 重采样和碰撞复查。
- 默认以 10 Hz 运行 PID 跟踪；可通过 `tracking_controller: pure_pursuit` 切换 Pure Pursuit。
- 默认 lookahead 为 0.50 m，到点容差为 0.35 m。
- 从 `/fastlio2/body_cloud` 过滤地面和车身点，将各已观测方向的障碍写入三维体素记忆；向 A* 投影时排除原静态地图已覆盖的点。
- 稳定检测到前方障碍后请求 A* 重规划；默认关闭障碍距离减速，安全轨迹按跟踪器原速度执行，近距离急停和碰撞检查仍生效。
- 每次发布速度前，用完整矩形车身检查候选运动、制动过程和当前运动速度的制动轨迹，覆盖转弯时侧面、车尾扫过的区域。
- 规划失败、路径为空、定位无效、TF 或点云超时、碰撞风险、紧急障碍或到点时均输出零 `Twist`。

普通 PID / Pure Pursuit 跟踪、重规划和恢复段现在都以启动参数 `max_vx` 作为直线巡航速度及纵向速度上限。局部规划器不再使用旧的 `pid_max_vx=0.28`、`pp_max_vx=0.26` 独立上限，也不再由前视点距离乘 `k_v` 决定巡航速度，避免设置 `max_vx=0.60` 后普通直线仍只有约 `0.28 m/s`。控制器保留加速度限制、转弯和到点减速，角速度仍受控制器参数与启动参数共同限制。完整入口把 `max_vx`、`max_wz` 同时传给局部规划器和底盘。局部规划器先限幅，再预测碰撞和发布速度，保证预测使用与底盘一致的转弯半径。分开启动局部规划器和底盘时，两者的 `max_vx`、`max_wz` 必须一致。

障碍记忆仅保存在进程内存中，坐标系为 FAST-LIO 连续局部世界系 `lidar`（此名称并非雷达自身坐标系）。控制时再投影到 `map` 和 `base_link`，使 ICP 对 `map -> lidar` 的修正能同步作用于已有记忆。无目标、换目标和到点均保留记忆；完整导航重启后记忆为空，不会遗留到下次地图中。FAST-LIO 重启导致 `lidar` 原点重置时，也应重启导航，不能沿用旧记忆。

障碍离开视野后不按时间删除。后续真实激光射线经过原位置附近并收到更远处的回波，才提供空闲证据。默认 `dynamic_memory_clear_confirmations=1`，首次有效空闲观测即可删除，不等待第二帧，也不经过重规划的约 `0.3 s` 确认。当前回波和射线终点附近的 `0.15 m` 保护距离仍生效。`dynamic_memory_free_confirmation_window=1.0 s` 仅在手动将确认帧数设为大于 1 时用于累计证据，不是障碍记忆的过期时间。

例如行人经过留下几个旧位置，之后雷达重新看到这些位置后方的墙面/地面，就逐格清掉被射线证实为空的旧位置；删除同步作用于车身检查、RViz 体素和 A* overlay。没有回波、没有射线覆盖的侧面盲区、被近处物体遮挡的位置不会仅因“当前没点”而清除。原始静态地图中的障碍也不会被此机制擦除。该策略不估计障碍物速度，也不能发现从未被雷达看到的侧面障碍。

RViz 默认配置已加入以下显示，Fixed Frame 使用 `map`：

| 显示项 | 含义 |
| --- | --- |
| `Observed and Remembered Obstacles` | `/dynamic_obstacles_markers`：绿色为最近 `0.5 s` 观测到的体素；橙色为仍被保留的历史体素。两者同样参与避障 |
| `Observed and Remembered Obstacles` 中的青色线框 | 同一话题的 `observation_box`：随 `base_link` 移动的新增障碍有效采集框，默认前方 `2.5 m`、后方 `1.5 m`、左右各 `1.5 m`，高度 `0.12 < z ≤ 1.50 m` |
| `Vehicle Collision Prediction` | `/local_planner/collision_check`：蓝色为当前车身轮廓；绿色为通过检查的预测轮廓，红色表示运动被拦截；轮廓含安全余量。红色球标出触发车身碰撞的障碍 XY 投影位置 |

默认蓝色轮廓由原来的 `0.84 m x 0.71 m` 缩为 `0.72 m x 0.59 m`。预测轮廓还包含行驶和制动期间的不同车身位置，运动时整体覆盖范围大于蓝框是正常的；碰撞检查另计入体素和栅格误差，不能只根据蓝框是否碰到障碍中心判断是否应停车。

需人工清空误记忆时，先确认车辆周围环境，再执行：

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh rosservice call /path_follower_node/clear_obstacle_memory "{}"
```

该服务会停车、取消当前目标并清空 overlay，等待新点云；之后需重新下发目标。它不会改动原地图，实际存在的障碍再次被观测后仍会写回记忆。

控制和障碍记忆参数位于 `src/fw_mid_local_planner/config/local_planner.yaml`；车身尺寸、`footprint_margin`、`dynamic_memory_resolution`、`collision_sample_distance` 和 `planner_tracking_clearance` 位于共享的 `src/fw_mid_common_utils/config/collision_geometry.yaml`。完整导航的 `geometry_config` 会同时传给两个规划器；分开启动时也应使用同一文件。共享配置在局部配置之后加载，覆盖其中同名参数。旧的 `robot_length_m`、`robot_width_m`、`dynamic_inflation_radius` 不再用于调整规划尺寸。

| 参数 | 默认值 / 作用 |
| --- | --- |
| `dynamic_memory_resolution` / `dynamic_memory_max_voxels` | `0.10 m` / `60000`；容量耗尽时停车，不自动丢弃旧障碍 |
| `dynamic_memory_clear_confirmations` / `dynamic_memory_max_clear_rays` | `1` 帧有效空闲观测即清除 / 每帧最多 `600` 条清除射线 |
| `dynamic_memory_clear_neighbor_radius` | `0.15 m`；沿真实空闲射线清理中心距射线不超过该半径的残留；不依赖旧占用格，不能超过体素边长的 2 倍，`0` 恢复精确射线清除 |
| `dynamic_memory_free_confirmation_window` | `1.0 s`；仅在确认帧数大于 1 时生效，不自动删除障碍 |
| `dynamic_obstacle_timeout` | `0.50 s`；点云新鲜度要求，与记忆保留时长无关 |
| `dynamic_point_step` / `dynamic_max_points` | `3` / `1600`；每帧点云抽样步长和最多参与障碍处理的点数，仍处理每个收到的最新帧，不跳过整帧 |
| `dynamic_x_min` / `dynamic_x_max` / `dynamic_y_abs` | `-1.5` / `2.5` / `1.5 m`；相对 `base_link` 的新增障碍采集框，框外点不加入障碍记忆，不能扩展雷达实际视野 |
| `dynamic_max_range` | `8.0 m`；真实回波射线的最大处理距离，不是新增障碍的采集框尺寸 |
| `dynamic_obstacle_support_radius` / `dynamic_obstacle_support_min_points` | `0.15 m` / `3`；同一帧内，每点三维半径内至少 3 个不同坐标点（含自身）才写入障碍记忆 |
| `dynamic_z_min` / `dynamic_z_max` / `dynamic_ground_z_max` | `-0.30` / `1.50` / `0.12 m`，相对 `base_link`；地面过滤开启时剔除 `z ≤ 0.12 m` 的点，需按安装高度和地面校准 |
| `dynamic_sensor_origin` | `[-0.011,-0.02329,0.04412] m`，雷达光心在 body cloud 坐标系中的位置，默认取 FAST-LIO `t_il` |
| `footprint_front` / `footprint_rear` / `footprint_half_width` | `0.34` / `0.34` / `0.275 m`，相对 `base_link`；必须按车体尺寸及原点偏移实测 |
| `footprint_margin` | `0.02 m`，四周安全余量；A* 和车身框同步使用 |
| `planner_tracking_clearance` | `0 m`，取消 A* 在车身和离散误差以外的额外跟踪距离 |
| `dynamic_static_filter_radius` | `0.00 m`，只排除原静态占用格覆盖的点 |
| `obstacle_slowdown_enabled` / `obstacle_slowdown_distance` | `false` / `0.30 m`；普通障碍不使用独立距离减速，仍由碰撞检查和 A* 重规划决定 |
| `emergency_stop_mode` / `dynamic_center_stop_radius` | `center_envelope` / `0.40 m`（原 `0.45 m`）；前半平面近距离急停圆，半径从 `base_link` 原点计，不是距车边的距离 |
| `obstacle_min_speed_scale` | `0.35`；仅在手动开启普通障碍距离减速时生效 |
| `dynamic_degraded_motion_enabled` / `dynamic_degraded_timeout` / `dynamic_degraded_speed_scale` | `true` / `0.50 s` / `0.25`；已有有效点云但暂时失联时，最多短时沿上一条命令的方向以 25% 速度继续；超过窗口、首次从未收到点云或低速制动仍不安全时停车 |
| `collision_prediction_time` / `collision_reaction_time` | `1.0` / `0.2 s`；预测及反应时间 |
| `collision_linear_deceleration` / `collision_angular_deceleration` | `0.25 m/s²` / `0.5 rad/s²`；制动模型参数，需用实际停车能力校准 |
| `collision_sample_distance` | `0.025 m`；车身扫掠采样间距 |
| `collision_speed_reduction_enabled` | `true`；候选运动未来碰撞时，尝试较低速度并重新检查 |
| `start_recovery_enabled` | `true`；A* 起点被膨胀区覆盖时尝试经过车身检查的前行出口 |
| `start_recovery_max_distance` | `1.2 m`；恢复段最长距离（包含出口余量），速度统一使用启动参数 `max_vx`，按该速度检查完整轨迹和制动距离 |

清理计算在少量残点时先建立残点邻格索引，并用分批向量距离计算跳过不可能经过任何旧障碍及其清理范围的射线；保留的射线仍执行完整栅格遍历和遮挡检查。被跳过射线的真实回波仍参与终点保护。密集场景按需缓存，清除规则不变。当前每个收到的点云都会进入回调，但通过 `dynamic_point_step=3` 和 `dynamic_max_points=1600` 限制单帧计算量，不采用每两三帧处理一次的方式。日志 `Obstacle cloud processing took ...` 记录处理过慢，`Obstacle input stale: scan_age=... receipt_age=...` 记录进入雷达失联处理。首次没有有效点云、失联超过 `dynamic_degraded_timeout`，或沿上一条命令的低速制动仍不安全时才停车；短时失联窗口内保持原方向并按 `dynamic_degraded_speed_scale` 降速。传感器超时阈值仍为 `0.50 s`，不无限期使用旧障碍数据。

移动障碍的残留清除沿真实空闲射线检查固定半径邻域：默认半径由 `0.10 m` 扩到 `0.15 m`，包括满足距离条件的斜向邻格；首次有效空闲观测即清除，不再等待第二帧。邻格中心必须位于射线半径内、回波终点之前，扩展不会递归扩大，也不要求射线先穿过仍占用的旧格。所有经过原有抽样及有效距离/车身过滤的真实回波（含未被抽选进 600 条清除射线的回波）都用于终点保护和直接射线遮挡检查。没有清除射线经过的盲区记忆继续保留，不按时间过期，也不修改原地图；因此不能保证从人离开起固定多少毫秒内删除，仍取决于后续扫描覆盖。邻域扩展是空间容差，并非周围区域已完整扫描；单帧清除与更大半径增加了误清附近稀疏障碍的可能，可将半径设为 `0`、确认帧数改回 `2` 恢复更严格的判定。停车重启导航生效。

新增障碍先经过同帧三维邻域过滤，再写入体素记忆：默认每个点在 `0.15 m` 半径内至少有 `3` 个不同坐标点（含自身），孤立点、两点和重复坐标不足以成为障碍；保留所有满足条件的点簇，不只保留最大点簇。计数在原有点云抽样、地面/车身/范围过滤及最多 `2400` 点采样之后、体素合并之前进行。因此多个有效回波合并后，在 RViz 中仍可能只显示一个体素。孤立点不会新增到 RViz 障碍记忆、A* 膨胀层或动态碰撞检查中，但其真实回波终点仍保留用于空闲射线判定，不把过滤当成障碍消失。原静态地图不受此过滤影响；原有盲区记忆仍保留，需确认空闲射线才能删除。该过滤也会舍弃回波不足的细小或稀疏障碍，参数需结合实际点云密度校准。停车重启导航后生效，重启清空之前会话的记忆。

默认前方障碍连续 3 个控制周期确认后触发重规划（10 Hz 下约 `0.3 s`），不是等待 3 秒；经过上述邻域过滤的点可立即进入记忆和碰撞检查。重规划期间若有旧路径，按正常跟踪速度继续行驶，不再额外限制为 `0.08 m/s`、`0.12 rad/s`；普通跟踪、重规划和恢复段统一受启动参数 `max_vx`、`max_wz` 限幅，每条速度命令仍须通过碰撞检查。恢复段按 `max_vx` 检查和执行，不在段末单独降速；接回跟踪器时从上一条已发布速度衔接，避免从零重新加速。旧参数 `start_recovery_speed`、`dynamic_replan_slow_vx_limit`、`dynamic_replan_slow_wz_limit` 已移除、不再生效。统一的是速度上限，不强制转弯、到点或有碰撞风险时维持恒速。碰撞检查同时使用原始静态栅格和障碍记忆，静态地图尚未收到时默认禁止运动。

地面剔除高度由 `0.08 m` 提高到 `0.12 m`。这是点云转换到 `base_link` 后的 Z 坐标阈值，不是雷达原始 Z，也只有原点在地面时才等于离地高度。默认只把 `z > 0.12 m` 且满足其他范围条件的点写入障碍记忆；地面回波仍参与原有的空闲射线更新，不修改原始 PCD/PGM 地图。同一阈值以下的低矮障碍也会被过滤，若仍有大片地板误检，应检查 `body -> base_link` 外参和地面倾斜，不能无限抬高阈值。修改后停车重启导航生效，重启会清空本次会话之前误记的地面障碍。

新增障碍还必须落在 RViz 青色采集框内。先在 `base_link` 中按框裁剪障碍候选，再限制障碍点数量，避免刹车时前倾产生的远处地面点写入记忆或挤占近处障碍的采样预算。框外真实回波只保留空闲射线用途：例如框内行人消失后，雷达看到框外墙面，仍能据此清掉射线穿过的旧行人位置。不会仅因记忆点移到框外就删除它；被遮挡、进入盲区的历史障碍仍保留。该过滤只作用于障碍层，FAST-LIO、定位点云和 RViz 原始点云显示不裁剪。有效框只能抑制远处误检，不能消除框内地面误检或代替外参/倾斜补偿。

已移除旧的 `dynamic_stop_distance=0.50` 宽范围一刀切停车规则，该旧参数不再生效。现在默认 `obstacle_slowdown_enabled=false`，取消按前后、两侧障碍距离渐进减速的区间；车身和预测轨迹安全时按跟踪器原速度执行。启动时的 `max_vx`、转弯和到点减速仍然生效，重规划与恢复段不再设置独立的低速上限。近距离急停、车身扫掠和实测运动制动检查保留；候选轨迹预测碰撞时仍允许尝试更低的安全速度，全部候选不安全则停车。这与按距离提前减速是独立的机制。

动态重规划时，起点落入 A* 膨胀区不再直接发停车指令。开启 `dynamic_keep_moving_during_replan` 且已有跟踪路径、车身碰撞检查开启时，允许在等待恢复路径期间继续跟踪旧路径，每条命令仍检查实时障碍和实测制动轨迹。日志 `Inflated start: planning recovery while tracking with live collision checks` 表示进入该流程；无旧路径、输入失效、规划失败或实际碰撞检查不通过时仍停车。

近距离急停圆半径缩为 `0.40 m`，只调整这一提前停车条件。实际矩形车身尺寸、每侧 `0.02 m` 余量、预测与实测制动检查仍有效，圆外也可能因扫掠碰撞而停车；等待恢复规划或恢复路径失效时的停车也不由这一半径决定。

若候选轨迹仅在未来位置碰到障碍，额外依次尝试该候选速度的 75%、50%、25%，线速度和角速度同比缩放；采用第一个通过完整碰撞检查的速度。每次重查仍使用未缩放的实测速度检查当前运动的制动轨迹。当前车身已碰撞、实际运动来不及停下、输入过期或全部候选不安全时仍停车；不会靠缩小实测速度来绕过急停。

靠近障碍或急停期间仍可按重规划周期请求新路径，不再因为前向距离小于 `0.50 m` 而直接跳过规划。若 A* 因起点位于保守圆形膨胀区而返回空路径，局部规划器会检查沿当前车头方向、最长 `1.2 m` 的短段，确认当前矩形车身、实测运动的制动轨迹、整段前行及段末制动均安全，再从出口请求 A*。优先选择出口及其 X/Y 各偏移 `0.10 m` 的九个位置均在膨胀区外的候选；若没有这样的安全候选，则尝试出口中心在膨胀区外的候选，同样执行完整车身和制动检查。额外 `10 cm` 规划余量不再是一票否决条件，避免车身可通过却一直等不到出口。只有出口到目标也有路径才执行恢复段；不会清除占用格、忽略障碍或自动倒车。当前车身已占用、前方堵塞、无出口、出口到目标无路、点云无效时仍停车。这是受限恢复，不保证狭窄环境中都能自行脱困。

恢复短段保留原形状，并与后续 A* 路径一起显示在 `/local_planner/global_plan`；不参与 shortcut 或平滑。执行时每条速度仍经过实时车身检查。距出口不超过 `0.05 m` 时，用最新代价图和障碍记忆复查实测位置到后续路径的连接及整条后续路径，并检查当前运动制动轨迹；通过后同一控制周期直接切回跟踪，不再无条件停车重规划。若接续检查暂未通过且出口尚未抵达，继续经过实时车身检查前行，不再在出口前 `2 cm` 提前停车。日志 `Recovery handoff: ... continuing tracking` 表示完成接续。抵达出口仍不能安全接续、横向偏差超过 `0.08 m` 或朝向偏差超过 `0.25 rad` 时仍停车重规划。`Blocked start recovery: checked ... forward, then A*` 表示恢复候选已通过检查，其中 `buffered_exit=False` 表示没有额外 `10 cm` 规划余量，但车身检查已通过。找不到出口时日志额外输出 `reason`、空闲出口数量或碰撞点，用于区分几何范围不足与实际碰撞。`obstacle_map_xy` 和 RViz 红球仍用于标识真正触发车身检查的点。

代码与参数修改后需在车辆停稳时重启导航生效，正在运行的节点不会自动热更新。

`obstacle_memory.py` 和 `footprint_collision.py` 均不依赖 A* 或 PID。以后接入 A* + 人工势场法时，可复用同一障碍记忆，并让新的速度输出继续经过车身碰撞检查；主要修改控制和局部避障策略。

### 7.3 Twist 到 JSON

`cmd_vel_to_command_node.py` 是唯一的单位边界：

```text
/cmd_vel.linear.x/y : m/s
/cmd_vel.angular.z  : rad/s
                    |
                    | math.degrees()，仅一次
                    v
{"gear":6,"vx":...,"vy":...,"wz":...}
                              wz: deg/s
```

- 有非零速度时默认 `gear=6`，全零时 `gear=1`。
- NaN/Inf 会被拒绝并替换成停车命令。
- 默认 20 Hz 发布；超过 0.5 s 没有新的 `/cmd_vel` 时持续发布停车 JSON。
- `max_vy=0`，当前不允许横移。

### 7.4 JSON 到 CAN

`can_driver_node.py` 再次校验 JSON、档位、有限数值和速度上限，然后以 20 Hz 发送控制帧。它使用单调时钟实现独立的 0.5 s 看门狗，ROS 时钟暂停时仍会停车；节点退出时也会发送停车帧。

| CAN ID | 方向/内容 |
| --- | --- |
| `0x18C4D1D0` | 控制命令，扩展帧，含 gear/vx/vy/wz/alive/BCC |
| `0x18C4D1EF` | 运动反馈 |
| `0x18C4E1EF` | BMS 反馈 |
| `0x18C4DAEF` | 急停和遥控器控制权反馈 |

导航使用 `gear=6` 行驶、`gear=1` 停车。驱动还识别 0、2、8，但不能在未确认底盘协议和现场安全条件时使用。

当前 CAN 节点会解析并记录急停状态、遥控器控制权和底盘速度反馈，但没有把急停/控制权发布为 ROS 状态，也没有让反馈参与上游速度闭环。软件导航的命令发布不能替代底盘硬件急停和遥控器接管。

`driver.launch` 只启动 JSON/CAN 驱动，不订阅 `/cmd_vel`。导航必须使用 `navigation_driver.launch`，或另行启动 `cmd_vel_to_command_node.py`。

## 8. MID360 网络

当前固定配置：

| 项目 | 值 |
| --- | --- |
| 雷达 IP | `192.168.1.118` |
| 车机接收 IP | `192.168.1.50/24` |
| 雷达物理网口 | `eno1` |
| 原图传网口 | `enp100s0` |
| 配置文件 | `src/livox_ros_driver2/config/MID360_config.json` |

默认启动允许把静态地址 `192.168.1.50/24` 从 `enp100s0` 迁移到 `eno1`。这会中断依赖该地址的图传，退出 ROS 后不会自动迁回。脚本不改默认路由、不 flush 整张网卡、不写持久网络配置，只维护该地址、雷达 `/32` 路由和邻居检查。

若必须保留图传，准备阶段和 launch 都要禁止接管；若地址仍冲突，启动会安全失败：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh --keep-video-address roslaunch fw_mid_bringup navigation.launch takeover_video_address:=false enable_can:=true start_rviz:=true max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

相关保护：

- `start_mid360.py` 负责提权准备、进程互斥和最终 `exec` 驱动。
- `mid360_network.py` 校验接口、地址、路由和 ARP。
- `check_mid360_topics.py` 要求启动 45 s 内同时收到有效点云与 IMU，最低频率 5 Hz / 50 Hz。
- Livox 驱动和健康检查均为 required 节点；硬件或话题检查失败会终止本次 launch。
- 单实例锁会拒绝第二个受保护的 MID360 驱动。其他 Livox 可视化工具和未经过包装器的旧驱动仍需人工停止。

只启动雷达：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh
```

只读预检查不会改网卡，也不会发送探测包：

```bash
bash scripts/ros1.sh /usr/bin/python3 src/livox_ros_driver2/scripts/mid360_network.py --check --takeover-video-address
```

网络脚本的边界和失败处理以 `scripts/with_mid360_network.sh` 及其调用的车机网络助手源码为准。

## 9. 地图文件

实时链路使用下面这一对同坐标系地图：

```text
src/fw_mid_localizer/maps/underground/map.pcd
src/fw_mid_global_planner/maps/underground/map_2d.yaml
src/fw_mid_global_planner/maps/underground/map_2d.pgm
```

当前二维地图参数：

| 项目 | 值 |
| --- | --- |
| 尺寸 | `691 x 593` 像素 |
| 分辨率 | `0.05 m/px` |
| origin | `[-26.3, -13.3, 0]` |
| 模式 | `trinary` |

`src/fw_mid_localizer/maps/underground/poses.txt` 是离线建图轨迹记录，实时定位不读取它。当前地图 origin 的 yaw 为 0；A* 支持 origin 旋转，但局部规划器的栅格查询尚未处理该 yaw，不能直接换成旋转 origin 的地图而不修改和验证对应代码。

更换地图时必须同时更新匹配的 PCD、PGM 和 YAML，并记录 SHA256、分辨率、origin、生成时间和初始位姿。仅复制 PCD 或仅复制 PGM 会导致三维定位与二维规划错位。

## 10. 目录与关键文件

### 10.1 顶层目录

| 路径 | 用途 | 是否参与实时运行 |
| --- | --- | --- |
| `src/` | 当前 ROS1 源码、配置和运行地图 | 是 |
| `scripts/ros1.sh` | 构造干净的 Noetic + 本工作区环境 | 是，推荐所有命令通过它执行 |
| `scripts/with_mid360_network.sh` | 交互准备 MID360 网络后启动 ROS 命令 | 是，硬件启动推荐入口 |
| `scripts/save_initial_pose.py` | 保存当前有效位姿到 `.ros/initial_pose.txt`，供下次启动读取 | 关机前按需运行 |
| `.ros/` | ROS 运行目录；`initial_pose.txt` 保存下次启动所需初值，其余内容运行时生成 |
| `.catkin_workspace` | catkin 工作区标记 | 构建识别 |

### 10.2 ROS1 包与关键文件

下表路径相对于 `src/`。项目自有逻辑按文件列出；厂商 SDK、第三方库、生成消息及构建产物按目录说明，不逐一解释重复的上游/生成文件。

| 包/文件 | 代码作用 |
| --- | --- |
| `fw_mid_bringup/launch/navigation.launch` | 唯一推荐的完整导航编排入口 |
| `fw_mid_common_utils/fw_mid_common_utils/utils.py` | 无 ROS 依赖的角度、四元数和 map/body 坐标变换工具 |
| `fw_mid_common_utils/config/collision_geometry.yaml` | A*、局部车身检查和 RViz 共用的尺寸与离散参数 |
| `fw_mid_common_utils/fw_mid_common_utils/collision_geometry.py` | 共用几何校验和规划膨胀半径推导 |
| `fw_mid_controller/fw_mid_controller/pid_controller.py` | 默认 4T4D 路径跟踪器，输出 `vx + wz`，含死区、加速度限制和到点减速 |
| `fw_mid_controller/fw_mid_controller/pure_pursuit_controller.py` | 可选 Pure Pursuit 跟踪器 |
| `fw_mid_controller/fw_mid_controller/_params.py` | 统一读取 ROS 参数或字典形式的参数源 |
| `fw_mid_ctrl/launch/driver.launch` | 仅 JSON -> CAN；不消费 `/cmd_vel` |
| `fw_mid_ctrl/launch/navigation_driver.launch` | 启动 Twist->JSON，并按参数选择是否启动 CAN |
| `fw_mid_ctrl/scripts/cmd_vel_to_command_node.py` | Twist 限幅、rad/s -> deg/s、JSON 编码和输入看门狗 |
| `fw_mid_ctrl/scripts/can_driver_node.py` | SocketCAN 编解码、反馈、BCC/alive 和硬件看门狗 |
| `fw_mid_ctrl/scripts/can/` | 项目内随附的 `python-can` 代码 |
| `fw_mid_fastlio2/config/lio.yaml` | Livox/IMU 话题、外参、滤波、IEKF 与局部地图参数 |
| `fw_mid_fastlio2/launch/lio.launch` | 在 `/fastlio2` 命名空间启动 LIO，默认 `imu_acc_scale=9.80665` |
| `fw_mid_fastlio2/src/lio_node.cpp` | 输入校验、点云/IMU 同步、发布里程计/点云/轨迹/TF |
| `fw_mid_fastlio2/src/map_builder/` | 点云预处理、IMU 处理、IESKF、ikd-tree 和局部地图维护 |
| `fw_mid_fastlio2/src/map_builder/map_builder.*` | 管理初始化、滤波更新和局部地图流程 |
| `fw_mid_fastlio2/src/map_builder/imu_processor.*` | IMU 初始化、状态传播和扫描运动补偿 |
| `fw_mid_fastlio2/src/map_builder/lidar_processor.*` | 点云变换、局部地图及激光观测处理 |
| `fw_mid_fastlio2/src/map_builder/ieskf.*` | 迭代误差状态卡尔曼滤波器 |
| `fw_mid_fastlio2/src/map_builder/ikd_Tree.*` | 增量 KD 树和最近邻搜索 |
| `fw_mid_fastlio2/src/map_builder/commons.*` | 点类型、滤波状态、配置和同步数据结构 |
| `fw_mid_fastlio2/src/utils.*`、`fw_mid_fastlio2/src/so3_math.h` | Livox/PCL 和时间转换、SO(3) 数学工具 |
| `fw_mid_localizer/config/localizer.yaml` | ICP 输入、帧、频率、体素分辨率、迭代和阈值 |
| `fw_mid_localizer/launch/localizer.launch` | 加载 PCD/初值，可选连带启动 LIO/RViz |
| `fw_mid_localizer/src/localizer_node.cpp` | 同步 body cloud 与 odom，管理重定位、有效性和 `map -> lidar` |
| `fw_mid_localizer/src/localizers/icp_localizer.*` | 粗配准与精配准实现 |
| `fw_mid_localizer/srv/*.srv` | 重定位请求和有效性查询接口 |
| `fw_mid_global_planner/launch/global_planner.launch` | 加载正式二维地图并启动 A* |
| `fw_mid_global_planner/scripts/astar_planner_node.py` | 地图读取、障碍膨胀、动态 overlay、A* 服务与可视化 |
| `fw_mid_local_planner/config/local_planner.yaml` | 路径处理、控制器、动态障碍和重规划参数主表 |
| `fw_mid_local_planner/launch/local_planner.launch` | 发布 `body -> base_link` 静态 TF 并启动跟踪节点 |
| `fw_mid_local_planner/fw_mid_local_planner/path_follower_node.py` | 目标、定位联锁、规划请求、路径跟踪、停车和动态重规划状态机 |
| `fw_mid_local_planner/fw_mid_local_planner/dynamic_obstacle_layer.py` | 点云过滤、时间戳 TF 变换、障碍记忆投影、RViz 体素和输入新鲜度检查 |
| `fw_mid_local_planner/fw_mid_local_planner/obstacle_memory.py` | 无 ROS 依赖的会话三维体素记忆、射线空闲确认和容量保护 |
| `fw_mid_local_planner/fw_mid_local_planner/footprint_collision.py` | 无 ROS 依赖的矩形车身扫掠与制动轨迹碰撞检查 |
| `fw_mid_local_planner/fw_mid_local_planner/start_recovery.py` | 起点落入保守膨胀区时的受限前行出口检查 |
| `fw_mid_local_planner/fw_mid_local_planner/path_processing.py` | shortcut、平滑、重采样与碰撞复查 |
| `fw_mid_local_planner/fw_mid_local_planner/obstacle_processing.py` | 点云过滤、聚类、前方障碍选择和历史点去重算法 |
| `fw_mid_local_planner/fw_mid_local_planner/local_avoidance_planner.py` | 保留的局部绕行算法，目前没有接入运行节点 |
| `fw_mid_local_planner/scripts/path_follower_node.py` | catkin 可执行入口，调用同名 Python 模块中的 `main()` |
| `livox_ros_driver2/config/MID360_config.json` | 雷达/主机 IP、UDP 端口和雷达型号配置 |
| `livox_ros_driver2/launch_ROS1/msg_MID360.launch` | 受保护的 MID360 驱动和双话题健康检查 |
| `livox_ros_driver2/scripts/start_mid360.py` | 网络准备入口、互斥锁和驱动包装 |
| `livox_ros_driver2/scripts/mid360_network.py` | 网络校验与受限修改逻辑 |
| `livox_ros_driver2/scripts/check_mid360_topics.py` | 点云/IMU 类型、数据、频率、时间戳和单发布者检查 |
| `livox_ros_driver2/third_party/Livox-SDK2/` | 随包静态编译的厂商 SDK |
| `livox_ros_driver2/src/` | ROS 驱动入口、设备管理、数据队列、SDK 回调、配置解析与消息发布 |
| `livox_ros_driver2/msg/CustomMsg.msg`、`livox_ros_driver2/msg/CustomPoint.msg` | 帧级/逐点自定义消息 |
各包的 `CMakeLists.txt` 和 `package.xml` 负责构建及依赖声明；Python 库包另有 `setup.py`、`__init__.py` 用于安装和导出接口。`fw_mid_localizer/rviz/localizer.rviz` 是整套导航共用的观察配置。

## 11. 安全机制

当前代码中的主要联锁和失效处理如下：

- 完整入口默认关闭 CAN，并使用很低的最终速度上限。
- MID360 启动前检查物理链路、IP、路由、ARP 和驱动单实例。
- 点云或 IMU 未达到最低频率时，required 健康节点会终止 launch。
- FAST-LIO2 拒绝非有限值、乱序/重复时间戳、无效扫描时长和非法外参。
- ICP 未收敛、输入超时或配准过期时，`localization_valid=false`。
- 局部规划要求定位有效和 TF 新鲜；默认还要求新鲜有效点云、可用静态地图和障碍记忆投影。规划失败、无路径、到点和紧急障碍均停车。
- 速度发布前检查完整车身及制动过程，静态栅格未知区和地图外按占用处理；预测碰撞时停车，默认模式下尝试重规划。
- 点云时间倒退或障碍记忆容量耗尽时阻止运动，不通过丢弃历史障碍恢复行驶。
- Twist/JSON 适配器和 CAN 驱动各有独立 0.5 s 看门狗。
- 非法 JSON、非法档位、NaN/Inf 会触发停车；缺失速度字段按零处理，缺失档位默认按行驶档处理（全零仍转停车档）；越界速度会限幅。
- CAN 驱动使用单调时钟，关闭节点时主动发送停车帧。
- 底盘自身急停和遥控器控制权仍是最终硬件安全层；当前软件只记录其 CAN 反馈，没有建立上游联锁，不能替代人工安全措施。

`dynamic_obstacle_timeout=0.5 s` 现在也是独立运动联锁：即使定位仍有效，障碍层长时间未收到有效点云、点云时间戳过期或记忆无法转换到规划坐标系，也会停车。停车不会删除障碍记忆。暂时无效的定位或输入恢复后，原目标可能继续跟踪；需取消时可使用第 7.2 节的清空服务（同时丢弃记忆）或停止导航。

新机制仍需实车确认车身尺寸、雷达外参、地面/车身过滤和制动距离。它能保留曾被看到而随后进入盲区的障碍，但无法感知从未看到的侧面障碍，也不预测移动障碍的未来位置；完整侧向覆盖仍需调整安装或补充传感器，不能把当前动态避障作为唯一防撞保障。

## 12. 常见问题

### 没有 `/livox/lidar` 或 `/livox/imu`

- 检查 `eno1` 是否 UP/LOWER_UP。
- 检查 `ip -4 route get 192.168.1.118` 是否显示 `dev eno1 src 192.168.1.50`。
- 检查 `ip -4 neigh show to 192.168.1.118 dev eno1`。
- 确认其他 Livox 可视化工具、旧驱动和第二条 launch 已退出。
- 查看 `.ros/log/mid360_startup.log`。

### 报 `Another guarded MID360 driver is starting/running`

上一条受保护驱动仍在运行。回到原 launch 终端按 `Ctrl-C`，等 `livox_lidar_publisher2` 从 `rosnode list` 消失后，只启动一次。不要删除锁文件，也不要自动杀掉未知旧节点。

### RViz 全空或提示缺少 `map` TF

通常是 ICP 尚未收敛，`/localizer/localization_valid` 为 false，因此没有 `map -> lidar`。先检查 LIO 输出，再提交合适的重定位初值。另请确认加载了项目 RViz 配置；原始 `/livox/lidar` 是 `CustomMsg`，RViz 应观察 `/fastlio2/body_cloud`。

### 重定位服务返回 success，但仍然 valid=false

`success` 只表示请求接受。检查 PCD 是否匹配、初值是否接近、body cloud/odom 是否同步、点云是否与地图重合，以及 ICP 分数/超时日志。

### A* 返回空路径

检查起点和目标是否在地图内、是否落在 `/astar_planner_node/costmap` 的膨胀区。历史记录中实际车位约 `(-0.35, 0.05)` 虽在原图为空闲，但距障碍仅约 4 cm，在当时约 `0.45 m` 的膨胀配置下已不可规划；以人为指定的 `(1,0) -> (2,0)` 则曾返回路径。这说明蓝线成功不等于实际车位可通行。当前膨胀由共享几何推导，默认值见第 7.1 节。

### 有 `/cmd_vel`，底盘不动

- `enable_can=false` 时本来就不会打开 CAN。
- `driver.launch` 不消费 `/cmd_vel`；需要 `navigation_driver.launch` 或完整入口。
- 检查 `can0` 是否 UP、波特率是否为 500000。
- 检查 `/fw_mid/command_dict`、底盘反馈、急停和遥控器控制权。
- 确认角速度只在适配器中从 rad/s 转为 deg/s，没有二次转换。

### 动态障碍没有显示或不触发

检查 `/fastlio2/body_cloud`、`lidar -> body -> base_link` 和 `map -> lidar`、点云高度范围、地面/车身过滤及 `/dynamic_obstacles_markers`。加载项目 RViz 配置后，障碍离开视野应从绿色转为橙色并继续保留。查看 `/local_planner/collision_blocked` 和节点日志可区分碰撞、输入过期、地图未收到或记忆满等停车原因；清空记忆见第 7.2 节。在完成静态障碍、盲区记忆、点云中断和停车距离实测前，不应放宽过滤参数直接上车。

### 行驶途中短暂停车

先区分日志原因，不能只调整急停半径：`Vehicle footprint blocked` 是车身/制动碰撞检查，`Dynamic obstacle emergency stop` 是近距离急停圆，`Navigation stop: inflated start` 是等待恢复规划的主动停车；`suffix unavailable or blocked`、`tracking deviation` 表示恢复接续失败或跟踪偏差。旧版 `recovery exit reached` 会在出口前主动停车，新版通过后续路径复查后直接接续。`Twist input timed out; publishing stop` 来自底盘适配器，表示 `/cmd_vel` 超过 `0.5 s` 未更新，与障碍距离无关。局部节点在控制周期超过 `0.3 s` 时输出 `Navigation control cycle took ...`，可用于对照超时发生时的计算延迟；不通过延长看门狗或重复发送旧的非零速度掩盖延迟。无路径或到点后也会持续发布零速度，避免正常驻车期间反复报输入超时。

### 编译后仍加载旧代码

始终通过 `scripts/ros1.sh` 构建和运行，检查 `rospack find` 是否指向本目录。跨目录复制后重新生成 catkin 缓存，不要叠加学长旧工作区。

## 13. 包内说明

各 ROS1 包中的 README 只保留当前节点、话题和配置说明；运行入口和安全要求以本文为准。
