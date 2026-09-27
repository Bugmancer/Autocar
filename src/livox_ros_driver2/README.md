# livox_ros_driver2（ROS1）

本包提供 MID360 的 ROS1 Noetic 驱动，发布 FAST-LIO2 使用的 `CustomMsg` 点云和 IMU。完整导航由 `fw_mid_bringup/launch/navigation.launch` 编排，单独启动雷达时使用：

```bash
cd /home/robot/Autocar_v1
bash scripts/with_mid360_network.sh roslaunch livox_ros_driver2 msg_MID360.launch
```

`config/MID360_config.json` 保存雷达网络配置，`launch_ROS1/msg_MID360.launch` 是 ROS1 启动文件，`scripts/check_mid360_topics.py` 用于启动健康检查。不要同时启动第二个 MID360 驱动或 LivoxViewer2。
