# fw_mid_fastlio2

FAST-LIO2 激光惯性里程计。输入为 `/livox/lidar`（`livox_ros_driver2/CustomMsg`）和 `/livox/imu`（`sensor_msgs/Imu`），完成 IMU 初始化和初始地图建立后才发布输出。

按[主 README](../../README.md)完整构建工作区。以下命令均在项目根目录执行；已有 MID360 驱动运行时，可单独启动里程计：

```bash
bash scripts/ros1.sh roslaunch fw_mid_fastlio2 lio.launch
```

此入口不启动雷达；完整导航会启动本节点，不要重复启动。`config_path:=/absolute/path/to/lio.yaml` 可选择配置文件。

## 坐标系与输出

默认 TF 为 `lidar -> body`：`lidar` 是连续的 LIO 世界坐标系，不是当前雷达原点；`body` 是 IMU/机体参考系。距离单位米，角度单位弧度。

| 话题 | 含义 |
| --- | --- |
| `/fastlio2/body_cloud` | 已去畸变并经雷达到 IMU 外参转换的 `body` 点云。 |
| `/fastlio2/world_cloud` | `lidar` 世界系点云。 |
| `/fastlio2/lio_odom` | `body` 在 `lidar` 中的位姿；`twist.linear` 位于子坐标系 `body`。 |
| `/fastlio2/lio_path` | `lidar` 中的历史轨迹，长度受限。 |

`config/lio.yaml` 的 `r_il`、`t_il` 表示雷达到 IMU/body 的变换：旋转矩阵按行排列，平移单位米。点的 `curvature` 借用为扫描内毫秒偏移，供去畸变使用，不表示几何曲率。

## IMU 与时序

配套 Livox 驱动的加速度单位为 `g`，默认 `imu_acc_scale=9.80665`，进入滤波器前换算为 `m/s^2`。静止原始加速度模长应接近 `1`；只有替换为已经输出 SI 单位的驱动时，才使用 `imu_acc_scale:=1.0`。launch 的同名私有参数会覆盖 YAML。默认启用重力对齐，初始化阶段应保持车辆静止。

节点拒绝非有限值、重复或乱序时间戳和非法扫描时长，并丢弃早于可用 IMU 历史的扫描；只有 IMU 已覆盖扫描结束时刻才处理该帧。`max_scan_duration` 默认 `0.5 s`，输入队列和输出轨迹均有限长。ROS 时间回退后必须重启 LIO；回放时定位器也应一并重启。

启动检查可在项目根目录执行：

```bash
bash scripts/ros1.sh rostopic echo -n 1 /livox/imu
bash scripts/ros1.sh rostopic hz /fastlio2/lio_odom
bash scripts/ros1.sh rosrun tf tf_echo lidar body
```

修改入口和同步逻辑看 `src/lio_node.cpp`；状态估计、去畸变与地图维护在 `src/map_builder/`，点云格式转换在 `src/utils.cpp`。
