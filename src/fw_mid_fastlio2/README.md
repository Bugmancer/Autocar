# fw_mid_fastlio2

ROS1/Noetic catkin port of the FAST-LIO2 implementation used by this
workspace. The estimator consumes the local `livox_ros_driver2/CustomMsg`
message and `sensor_msgs/Imu`, then publishes:

* `body_cloud` (`sensor_msgs/PointCloud2`)
* `world_cloud` (`sensor_msgs/PointCloud2`)
* `lio_path` (`nav_msgs/Path`)
* `lio_odom` (`nav_msgs/Odometry`)
* TF `lidar -> body`

The launch file puts the node in the `fastlio2` namespace, so the resulting
topics are `/fastlio2/body_cloud`, `/fastlio2/world_cloud`,
`/fastlio2/lio_path`, and `/fastlio2/lio_odom`.

`lio_odom.pose` and the `lidar -> body` TF describe the body pose in the LIO
world frame. In accordance with `nav_msgs/Odometry`, `lio_odom.twist.linear`
is rotated into the child (`body`) frame. `body_cloud` is transformed with
the configured LiDAR-to-IMU extrinsic; `world_cloud` is in `lidar`.

The package contains an Eigen-only SO(3) helper instead of requiring a
system Sophus installation. The bundled Livox ROS1 driver documents its raw
accelerometer values as `g` and copies them directly into `sensor_msgs/Imu`,
so `imu_acc_scale` defaults to the standard gravity value `9.80665`. Set it
to `1.0` only when using a driver that already publishes acceleration in
`m/s^2`.

Build and launch from this workspace after sourcing ROS Noetic:

```bash
catkin_make --pkg fw_mid_fastlio2
roslaunch fw_mid_fastlio2 lio.launch
```

The launch command requires the local `livox_ros_driver2` message package and
live IMU/LiDAR topics. No hardware node is started by this package.

## Runtime checks

Start the bundled Livox driver separately, then start this package. Before
moving the vehicle, verify the input units and output interfaces:

```bash
rostopic type /livox/lidar
rostopic type /livox/imu
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /livox/imu
rostopic hz /fastlio2/lio_odom
rostopic hz /fastlio2/body_cloud
rosrun tf tf_echo lidar body
```

For the bundled driver, a stationary raw acceleration magnitude near `1`
means the default `imu_acc_scale=9.80665` is correct. If a replacement driver
reports a magnitude near `9.81`, launch with `imu_acc_scale:=1.0`. Output is
not published until IMU initialization and the initial point-cloud map have
completed.

The node rejects non-finite samples, invalid scan durations, stale scans,
duplicate/out-of-order timestamps, malformed extrinsics, and invalid ranges.
Input queues and the published path are bounded. If ROS time moves backward
(for example when replaying a bag in a loop), restart the node so the filter
state and timestamps begin in the same epoch.
