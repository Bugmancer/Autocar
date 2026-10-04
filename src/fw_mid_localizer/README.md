# fw_mid_localizer

使用粗、精两级 ICP 将 `/fastlio2/body_cloud` 与 PCD 地图配准，并结合 `/fastlio2/lio_odom` 发布地图定位。初值须接近真实位姿；当前不提供全局位置或朝向搜索。

按[主 README](../../README.md)完整构建工作区。以下命令在项目根目录执行，独立启动雷达、LIO、定位器和 RViz，不启动规划器或 CAN：

```bash
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup relocalization.launch \
  initial_pose:="[0,0,0,0,0,0]"
```

将示例初值改为实际位置。完整导航已包含定位器，独立调试前先停止导航；雷达和 LIO 已运行时，添加 `start_lidar:=false start_lio:=false`。可用 `start_rviz:=false`、`print_pose:=false` 关闭显示或位姿日志；`map_pcd`、`lio_config`、`localizer_config` 分别选择 PCD 地图和两个 YAML 配置。

## 坐标与初值

定位器发布 `map -> lidar`，LIO 发布 `lidar -> body`。其中 `lidar` 是 LIO 世界系，`body` 是 IMU/机体参考，不等同于原始雷达原点或底盘 `base_link`。导航启动入口另提供 `body -> base_link`；独立重定位入口不提供该底盘变换。

`initial_pose` 顺序为 `[x,y,z,yaw,pitch,roll]`，描述 `map -> body`，位置单位米，角度单位弧度。初值采用 `Rz(yaw) * Rx(roll) * Ry(pitch)`；日志 `rpy_rad` 则采用标准 `Rz(yaw) * Ry(pitch) * Rx(roll)`，横滚或俯仰非零时不能直接把日志角度填回初值。位姿保存和下次启动方法见主 README。

默认 PCD 为 `maps/underground/map.pcd`，与 `fw_mid_global_planner/maps/underground/map_2d.yaml` 配套。定位只读取三维 PCD；更换定位地图不会自动更换导航的二维地图，两者必须保持同一坐标系。

## 输出与有效性

| `/localizer/` 下的话题 | 含义 |
| --- | --- |
| `pose` | `body` 在 `map` 中的 `PoseStamped`，由最近接受的配准与最新 LIO 位姿合成。 |
| `icp_pose` | 最近接受的 ICP 位姿，使用参与配准的扫描时间戳。 |
| `map_cloud` / `aligned_cloud` | 三维地图与已接受的对齐点云，均在 `map` 中。 |
| `fitness_score` | 精配准最近邻平方距离均值，单位平方米；是几何残差，不是定位正确的概率。 |
| `localization_valid` | 当前是否已收敛，且输入和最近成功配准均未过期。 |

点云坐标系必须与里程计 `child_frame_id` 一致，时间戳非零且严格递增。节点私有参数 `~max_sync_dt` 默认 `0.05 s`，限制点云与里程计时间差及允许的未来时间偏差；`~input_timeout` 默认 `0.5 s`，同时检查 ROS 采样年龄和墙钟接收年龄；`~alignment_timeout` 默认 `3 s`，限制最近成功配准的墙钟年龄。它们是节点私有 ROS 参数，不是当前 YAML 中的配置项。

定位失效后停止更新定位 TF 和 `pose`；RViz 或下游缓存可能仍显示旧结果，必须同时检查 `localization_valid`。回放使用 ROS 模拟时间，时钟回退后重启 LIO 和定位器。

## 重定位服务

节点不订阅 RViz `/initialpose`。保持车辆停稳，通过服务提交 PCD 和 `map -> body` 初值；以下零位姿仅为示例：

```bash
bash scripts/ros1.sh rosservice call /localizer/relocalize \
  "{pcd_path: '$(pwd)/src/fw_mid_localizer/maps/underground/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
bash scripts/ros1.sh rosservice call /localizer/relocalize_check "{code: 0}"
```

`success: true` 只表示已接受地图和初值，随后等待 ICP；拒绝时返回 `success: false` 和原因。只有状态服务 `valid: true` 才表示本次请求已收敛且仍有效，`code` 不改变检查逻辑。请求被接受后到定位恢复前，不发布有效定位；更换 PCD 时还需单独配置配套的导航二维地图。

节点订阅、同步、时效和服务逻辑在 `src/localizer_node.cpp`；粗精配准及 fitness 门限在 `src/localizers/icp_localizer.cpp` 和 `config/localizer.yaml`。
