# Autocar

ROS1 Noetic 车载导航工作区：MID360 -> FAST-LIO2 -> PCD 定位 -> A* -> 局部跟踪 -> CAN 底盘。电脑端负责离线建图，车机直接通过 `/cmd_vel` 控制底盘。

> `navigation.launch` 默认 `enable_can:=false`。完整自主行驶闭环与动态避障仍需实车验证；首次运行应架空轮组或使用受控场地，保留人工急停和遥控接管。车身尺寸、安装外参、地面过滤及制动参数必须按实车校准。

## 构建

下列命令在车机 Linux 上执行，示例工作区为 `/home/robot/Autocar_v1`，按实际路径替换。需要 ROS Noetic、catkin、Python 3、C++14、PCL、Eigen3、yaml-cpp、Boost、NumPy、OpenCV 和 PyYAML；包级依赖见各 `package.xml`。Livox SDK2 和裁剪后的 `python-can` 已随项目提供，CAN 仅保留当前底盘使用的 SocketCAN 内建后端。

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh catkin_make -DCMAKE_BUILD_TYPE=Release -j2
```

`scripts/ros1.sh` 只叠加系统 Noetic 和本工作区，避免加载旧工程；后续 ROS 命令也建议通过它运行。源码不携带编译产物，首次使用或复制到其他机器、路径后执行上方命令，重新生成 `build/`、`devel/`，不要复用旧路径缓存。代码或配置变更后停车、重建并重启导航。

## 地图与初值

默认使用同一坐标系的一套地图：

- 三维定位：[map.pcd](src/fw_mid_localizer/maps/underground/map.pcd)。
- 二维规划：[map_2d.yaml](src/fw_mid_global_planner/maps/underground/map_2d.yaml) 和对应 PGM。

更换地图时同时指定 `map_pcd:=/绝对路径/map.pcd` 与 `map_yaml_path:=/绝对路径/map_2d.yaml`，确认分辨率、origin 和坐标系一致。当前局部栅格查询要求地图 origin 的 yaw 为零。`poses.txt` 是离线建图记录，不参与实时定位。

`initial_pose` 表示 `map -> body`，顺序为 `[x,y,z,yaw,pitch,roll]`，单位为米和弧度，旋转约定为 `Rz(yaw)*Rx(roll)*Ry(pitch)`。ICP 需要接近真实位置的初值，不支持无初值全地图搜索；默认全零只适合地图原点附近。

首次使用可信的现场初值启动；定位正常后，在车辆停稳且定位节点仍运行时保存：

```bash
bash scripts/ros1.sh python3 scripts/save_initial_pose.py
```

脚本检查定位有效性与数据新鲜度，保存到 `.ros/initial_pose.txt`，失败时保留原文件。看到 `Saved map -> body seed to ...` 后再停止导航、关机；系统不会在关机时自动保存，下次启动也需要显式传入保存的 `initial_pose`。保存值仅适用于同一地图、同一停车位置和朝向；关机后车辆被挪动时需要重新给初值。不要把定位日志中的常规 RPY 直接当作非零 roll/pitch 的启动初值。

## 启动导航

启动前确认旧 Livox、FAST-LIO、导航和 CAN 节点已退出；本项目未注册开机自启动，但车机原有服务可能占用设备或重复发布 TF。准备急停和遥控器，并检查管理网络不会受雷达地址迁移影响。

首次调试保持 CAN 关闭，以下全零初值必须按现场修改：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch start_rviz:=true initial_pose:="[0,0,0,0,0,0]"
```

已有有效保存初值时：

```bash
test -s .ros/initial_pose.txt && bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch start_rviz:=true initial_pose:="$(cat .ros/initial_pose.txt)"
```

网络包装器会请求 sudo 准备 MID360 网络，再以普通用户启动 ROS。不要使用 `sudo roslaunch`。`roslaunch` 会按需启动本机 master；包装器保留 `ROS_MASTER_URI`、`ROS_IP`、`ROS_HOSTNAME`，使用外部 master 时应统一配置。

### 选择跟踪策略

完整导航与局部规划入口均支持 `follower_variant`：

| 值 | 可执行节点 | 跟踪方法 |
| --- | --- | --- |
| `classic`（默认） | `path_follower_node.py` | PID，或由 `tracking_controller` 选择 Pure Pursuit |
| `apf` | `path_follower_node_v1.py` | 人工势场 |
| `v3` | `path_follower_node_v3.py` | 自适应前视、几何斥力与短轨迹选择 |

在上述完整启动命令后添加 `follower_variant:=classic`、`follower_variant:=apf` 或 `follower_variant:=v3` 即可选择策略。三个独立入口组合相同的导航运行逻辑和安全检查，不通过节点继承替换策略。旧参数 `follower_node` 仍兼容脚本名，显式设置时优先；改用 `follower_variant` 时删除原命令中的 `follower_node:=...`，避免旧脚本覆盖新选择。

首次更新到包含 V3 的代码后，先执行上方构建命令，让 catkin 生成新入口。在项目根目录使用已保存初值启动 V3（默认关闭 CAN）：

```bash
test -s .ros/initial_pose.txt && \
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch \
  follower_variant:=v3 enable_can:=false start_rviz:=true \
  initial_pose:="$(cat .ros/initial_pose.txt)"
```

没有有效保存初值时，改用现场确认的 `initial_pose:="[x,y,z,yaw,pitch,roll]"`；自定义地图、限速等参数照常追加。启用底盘前完成下方 CAN 放行步骤。

V3 使用弧长进度和自适应前视，保留必要的后方绕行段；结合曲率、横向加速度与终点制动限速，按车身净距聚合障碍斥力，并保持绕行侧。短轨迹经过排序和完整碰撞检查后才进入公共发布检查，持续缺少路径进展时尝试重规划。`v3_*` 参数仅由 V3 读取，详见[算法与调参说明](src/fw_mid_local_planner/README.md#v3-算法与参数)。

切换前在原 launch 终端按 `Ctrl-C`，等待退出后再启动。三种策略保留相同 ROS 节点名 `/path_follower_node` 和接口，不能同时运行。用 `bash scripts/ros1.sh rostopic info /cmd_vel` 确认只有一个速度发布者。局部独立调试见[局部规划说明](src/fw_mid_local_planner/README.md)。

### 放行检查与 CAN

在其他终端逐项检查，持续输出命令按 `Ctrl-C` 结束：

```bash
bash scripts/ros1.sh rostopic hz /livox/lidar /livox/imu
bash scripts/ros1.sh rostopic hz /fastlio2/lio_odom /fastlio2/body_cloud
bash scripts/ros1.sh rosservice call /localizer/relocalize_check "{code: 0}"
bash scripts/ros1.sh rosrun tf tf_echo map base_link
bash scripts/ros1.sh rostopic echo /cmd_vel
bash scripts/ros1.sh rostopic echo /fw_mid/command_dict
```

放行前须满足：雷达与 LIO 连续，定位返回 `valid: True`，TF 位置与现场一致且新鲜，无目标时速度为零。雷达典型频率为点云 10 Hz、IMU 200 Hz；`relocalize` 返回 `success: true` 仅表示请求已接受，不代表 ICP 收敛。定位无效时收到的目标会被拒绝，恢复后须重新发送。

确认可安全运动后，准备 SocketCAN（launch 不负责配置）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

停止调试 launch，再启动底盘控制：

```bash
test -s .ros/initial_pose.txt && bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup navigation.launch enable_can:=true start_rviz:=true max_vx:=0.05 max_wz:=5.0 initial_pose:="$(cat .ros/initial_pose.txt)"
```

启动后再次检查定位和零速，再发送目标。完整入口默认上限为 `0.05 m/s`、`5 deg/s`。`/cmd_vel.angular.z` 使用 rad/s，仅底盘适配器转换为 deg/s；横移固定为零。单独启动局部规划与底盘时，两者的 `max_vx`、`max_wz` 必须一致。

### 目标、停机与远程操作

RViz 使用项目配置 `src/fw_mid_localizer/rviz/localizer.rviz`，Fixed Frame 为 `map`。确认地图、膨胀层和车辆位置正确后，用 `2D Nav Goal` 选择可通行目标。蓝线为 A* 路径，绿线为跟踪路径，箭头为期望方向；显示命令不代表底盘已经执行。

也可发布 `map` 坐标系目标。下例仅展示格式，坐标必须按现场修改：

```bash
bash scripts/ros1.sh rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

到点后停车等待下一个目标，无需重启；行驶中发送目标会替换当前目标。到点要求 XY 距离进入容差，V3 还要求剩余弧长进入容差；均不保证最终朝向，也没有倒车规划。

正常结束前可保存初值，然后在导航 launch 终端按 `Ctrl-C`，等待节点退出并确认车辆停止。紧急情况使用实物急停。手发一次零 `/cmd_vel` 不会取消目标，跟踪器仍可能继续发布命令；短时定位或传感器异常恢复后，已有目标也可能继续执行。

SSH 使用车机管理网口或无线地址，不要使用雷达 IP 或会被迁移的 `192.168.1.50`。无图形桌面时设置 `start_rviz:=false`；可用 `tmux` 保持会话，但 SSH 断开或 tmux 分离不会停车。重新连接原会话后停止 launch，不要重复启动。

## 关键配置

完整参数见 [navigation.launch](src/fw_mid_bringup/launch/navigation.launch)，常用覆盖项如下：

| 参数 | 用途 |
| --- | --- |
| `start_lidar`、`start_lio` | 已有对应节点时设为 `false`，避免重复启动 |
| `enable_can`、`can_interface` | 默认关闭 CAN，接口默认 `can0` |
| `start_rviz` | 默认 `false` |
| `max_vx`、`max_wz` | 巡航/纵向上限（m/s）与角速度上限（deg/s） |
| `follower_variant` | `classic`（默认）、`apf` 或 `v3` |
| `body_to_base_link` | 实车安装外参，必须校准且只保留一个发布者 |
| `lio_config`、`localizer_config`、`planner_config` | 对应模块 YAML |
| `geometry_config` | A*、路径处理与车身检查共用的几何配置 |

算法参数见 [local_planner.yaml](src/fw_mid_local_planner/config/local_planner.yaml)，车身、余量和采样参数见 [collision_geometry.yaml](src/fw_mid_common_utils/config/collision_geometry.yaml)。分开启动模块时也必须加载相同几何配置。不要缩小车身、忽略真实障碍或延长看门狗来掩盖无法规划和计算延迟。

TF 链为 `map -> lidar -> body -> base_link`，分别由 ICP、FAST-LIO2 和静态安装外参提供。这里 `lidar` 是 LIO 的连续世界坐标系，`body` 是 IMU/body 参考系，都不等同于雷达当前光心。

## MID360 网络

当前配置：雷达 `192.168.1.118`，车机接收 `192.168.1.50/24`，雷达网口 `eno1`，原图传网口 `enp100s0`。配置源为 [MID360_config.json](src/livox_ros_driver2/config/MID360_config.json)。

默认包装器允许把该静态地址从图传口迁到雷达口，会中断依赖该地址的连接，退出 ROS 后不会回滚。它不更改默认路由、不清空整张网卡，也不写持久网络配置。需要保留图传地址时，包装器与 launch 都要禁用接管；冲突时启动会失败：

```bash
bash scripts/with_mid360_network.sh --keep-video-address roslaunch fw_mid_bringup navigation.launch takeover_video_address:=false initial_pose:="[0,0,0,0,0,0]"
```

只启动雷达可执行 `bash scripts/with_mid360_network.sh`。只读网络检查：

```bash
bash scripts/ros1.sh /usr/bin/python3 src/livox_ros_driver2/scripts/mid360_network.py --check --takeover-video-address
```

驱动有单实例保护；启动健康检查要求 45 秒内收到有效点云与 IMU，最低频率分别为 5 Hz 和 50 Hz。驱动或健康检查失败会结束 launch。报单实例冲突时停止原 launch，不要删除锁文件。

## 安全与排查

已确认但尚未修复的边界问题见 [待处理代码问题](CODE_REVIEW.md)，清理旧代码不代表这些问题已解决。

- 定位、TF、静态地图、障碍记忆、速度合法性与车身制动检查共同约束输出。未知栅格及地图外按占用处理，无路径、到点或无法安全运动时停车。
- 默认允许短时雷达数据过期后按已有受检命令降速，仍检查静态地图、障碍记忆和制动轨迹；超过允许窗口停车。具体开关和时限见 `dynamic_degraded_*`，这段时间无法感知新出现的障碍。当前紧急包络只对新鲜输入检查，降级期间不保证额外配置的紧急停车距离。
- 障碍记忆保留已观察到的盲区障碍，只有真实空闲射线或人工清空才会删除，不按时间自动消失。无法感知从未看见的障碍，也不预测移动障碍速度。
- Twist/JSON 与 CAN 分别有 0.5 秒输入看门狗；非法数据触发停车，CAN 正常退出发送停车帧。底盘急停及控制权反馈仅被记录，没有上游控制联锁。
- LIO 时钟回退或原点重置后重启整套导航，不能沿用旧滤波状态和障碍记忆。

| 现象 | 先检查 |
| --- | --- |
| 雷达无数据 | `eno1` 链路、到雷达的路由/ARP、重复驱动、`.ros/log/mid360_startup.log` |
| RViz 无 `map` | LIO 输出、定位有效性、PCD/初值；原始 Livox 是 CustomMsg，观察 `/fastlio2/body_cloud` |
| A* 空路径 | 起点/目标是否在地图内及膨胀区外；受限前行恢复也必须通过车身和制动检查 |
| 有速度但底盘不动 | CAN 是否启用、接口/波特率、`/fw_mid/command_dict`、急停及遥控控制权 |
| 行驶中停车 | `/local_planner/collision_blocked` 与日志中的碰撞、输入过期、恢复失败或看门狗原因 |
| 重建后仍是旧代码 | 用包装器执行 `rospack find`，检查旧工作区环境及 catkin 缓存 |

## 模块与验证

| 包 | 职责 |
| --- | --- |
| `fw_mid_bringup` | 完整导航与独立重定位入口 |
| `livox_ros_driver2` | MID360、网络检查与厂商 SDK |
| [fw_mid_fastlio2](src/fw_mid_fastlio2/README.md) | 点云/IMU 里程计 |
| [fw_mid_localizer](src/fw_mid_localizer/README.md) | PCD ICP 定位、初值服务与位姿输出 |
| `fw_mid_global_planner` | 二维地图、膨胀、动态 overlay 与 A* |
| [fw_mid_local_planner](src/fw_mid_local_planner/README.md) | 策略选择、路径执行、障碍记忆与碰撞检查 |
| `fw_mid_controller`、`fw_mid_common_utils` | 控制器与共享几何/坐标工具 |
| `fw_mid_ctrl` | Twist -> JSON -> SocketCAN |

仅定位时使用 `fw_mid_bringup relocalization.launch`，不启动规划或 CAN；不要与完整导航同时运行。详细接口见定位包文档。

离线回归测试（Python 3、NumPy、PyYAML）：

```bash
python3 -m unittest discover -s src/fw_mid_local_planner/tests -v
python3 -m unittest discover -s src/fw_mid_global_planner/tests -v
```

离线验证（2026-10-04）：清理后局部规划 73 项、全局规划 1 项测试通过，覆盖三选一入口、弧长进度、速度约束、障碍聚合、候选预算及计算期间取消命令。V3 另在理想运动模型和 ROS 替身下跑通直线纠偏、直角转弯、后方绕行、低速行驶、双侧墙通道和已规划障碍绕行 6 个场景；这些场景不属于上面的单元测试计数。

CAN 裁剪前后，150 组控制帧编码、SocketCAN 封装/解包和反馈解析结果一致；该验证使用 ROS 与插件发现替身，不涉及真实总线通信。源码语法、配置解析、launch 引用及文档链接检查通过。

清理后保留当前 ROS1/MID360 启动链、三种跟踪策略、独立定位与雷达诊断入口，以及地图、标定、保存初值脚本和离线测试。Livox 驱动与 SDK 共用一份 RapidJSON；第三方许可证和必要的底层实现保留。

尚未执行 Noetic 构建、ROS 实机联调或实车性能验证。离线结果不能替代受控场地的静态障碍、盲区记忆、传感器中断和停车距离验证。
