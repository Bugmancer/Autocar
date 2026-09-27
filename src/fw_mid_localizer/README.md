# fw_mid_localizer

ROS1/Noetic port of the ICP localizer supplied in the ROS2 reference tree.
It synchronizes `/fastlio2/body_cloud` with `/fastlio2/lio_odom`, aligns the
body-frame scan against a PCD map, and publishes TF `map -> lidar`. FAST-LIO
publishes `lidar -> body`, so the two transforms produce the complete map-to-
vehicle transform without involving the chassis controller.

The alignment uses coarse-to-fine PCL ICP. Supply an approximate initial pose
near the real pose; ICP does not perform automatic global position or yaw
search. `initial_pose` is `[x, y, z, yaw, pitch, roll]`, with position in metres
and angles in radians, and describes `map -> body`.
For compatibility, seed rotation uses `Rz(yaw) * Rx(roll) * Ry(pitch)`.
The printed `rpy_rad` uses standard `Rz(yaw) * Ry(pitch) * Rx(roll)`;
do not copy those angles back into the seed when roll and pitch are nonzero.

After ICP converges, `/localizer/pose` publishes the current fitted `body` pose
as `geometry_msgs/PoseStamped` in the `map` frame. Position is in metres and
orientation is an `x/y/z/w` quaternion. The pose combines the latest FAST-LIO
odometry with the most recent accepted map alignment and stops publishing when
the localization validity timeouts expire.

`body` is FAST-LIO's IMU/body reference. It is not necessarily the raw Livox
sensor origin or the chassis `base_link`; the FAST-LIO extrinsics convert raw
LiDAR measurements to `body`. The TF frame named `lidar` is FAST-LIO's local
odometry frame, not the current physical laser origin.

## Standalone Relocalization

Build from the workspace, source its environment, then start the MID-360,
FAST-LIO and map localizer with the dedicated entry:

```bash
cd /home/robot/Autocar_v1
bash scripts/ros1.sh catkin_make --pkg fw_mid_localizer fw_mid_bringup -DCMAKE_BUILD_TYPE=Release -j2
source devel/setup.bash
bash scripts/with_mid360_network.sh roslaunch fw_mid_bringup relocalization.launch \
  initial_pose:="[2.503,-0.023,-0.0395,-0.0175,0.713,0.030]"
```

The values above are an example nearby estimate for the bundled map; set them
to the vehicle's actual starting estimate. This launch starts no planner or
CAN controller. RViz opens by default with the map (gray), accepted aligned
scan (green), fitted body pose and coordinate axes. Use `start_rviz:=false`
for a terminal-only run; `print_pose:=false` disables the pose log.

Set `map_pcd:=/absolute/path/to/map.pcd` to load another existing 3D point-cloud
map. A 2D occupancy-map YAML is not an input to ICP. The default map is
`fw_mid_localizer/maps/underground/map.pcd`.

When the MID-360 driver and FAST-LIO are already running, start only localization:

```bash
roslaunch fw_mid_bringup relocalization.launch \
  start_lidar:=false start_lio:=false \
  initial_pose:="[2.503,-0.023,-0.0395,-0.0175,0.713,0.030]"
```

Only one localizer should run at a time. Stop the navigation launch before
starting this standalone entry, since navigation already contains a localizer.
`lio_config` and `localizer_config` accept alternate YAML configuration paths.

The output topics are:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/localizer/pose` | `geometry_msgs/PoseStamped` | Current fitted `body` pose in `map`, including odometry between accepted matches. |
| `/localizer/icp_pose` | `geometry_msgs/PoseStamped` | Direct accepted ICP result at the fitted scan timestamp. |
| `/localizer/map_cloud` | `sensor_msgs/PointCloud2` | Loaded 3D map in `map`. |
| `/localizer/aligned_cloud` | `sensor_msgs/PointCloud2` | Accepted scan transformed into `map`, stamped at the fitted scan time. |
| `/localizer/fitness_score` | `std_msgs/Float64` | Refined ICP mean squared nearest-neighbor distance in square metres; lower is better. Published for accepted matches. |
| `/localizer/localization_valid` | `std_msgs/Bool` | Whether the latest accepted match and input data are still valid. |

Inspect both the pose and validity, since the last RViz display or previously
received pose may remain visible after localization becomes invalid:

```bash
rostopic echo /localizer/pose
rostopic echo /localizer/localization_valid
rostopic echo /localizer/fitness_score
```

Each accepted fit also logs `xyz_m`, conventional `rpy_rad` in roll/pitch/yaw
order, `q_xyzw`, and `fitness_m2` when `print_pose` is enabled. Fitness is a
geometric residual, not a probability or proof of globally correct alignment.

Input clouds must share the odometry child frame and have strictly increasing,
nonzero timestamps. Cloud and odometry timestamps must differ by at most
`~max_sync_dt` (default 0.05 s). Input timestamps and wall-clock receipt age
must remain within `~input_timeout` (default 0.5 s); the last accepted alignment
must remain within `~alignment_timeout` (default 3 s). Stale inputs are not
re-fitted. For bag playback use ROS simulated time, and restart the localizer
and LIO after resetting the clock.

## Localizer-Only Entry

Build and launch:

```bash
catkin_make --pkg fw_mid_localizer
source devel/setup.bash
roslaunch fw_mid_localizer localizer.launch
```

Start the workspace's FAST-LIO node from the same launch when desired:

```bash
roslaunch fw_mid_localizer localizer.launch start_lio:=true rviz:=true
```

For a vehicle startup, the launch file defaults to the bundled laboratory map
and an approximate zero pose. Override the map and initial estimate when
needed. The localizer loads the PCD before it accepts synchronized LIO samples
and does not publish a valid TF until ICP converges:

```bash
roslaunch fw_mid_localizer localizer.launch \
  map_pcd:=$(rospack find fw_mid_localizer)/maps/underground/map.pcd \
  initial_pose:="[0.0,0.0,0.0,0.0,0.0,0.0]"
```

The project laboratory PCD is bundled at
`maps/underground/map.pcd`. Its matching 2D map is
`fw_mid_global_planner/maps/underground/map_2d.yaml`; both use the same map
coordinate frame. The default navigation launch selects this pair and an
approximate `[0,0,0,0,0,0]` seed. Refine `initial_pose` or use the relocalization
service before enabling motion. The node does not subscribe to RViz's
`/initialpose` topic; use the service to replace the estimate at runtime.

Submit a map and initial map-to-body estimate (angles are radians):

```bash
rosservice call /localizer/relocalize \
  "{pcd_path: '$(rospack find fw_mid_localizer)/maps/underground/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

`success: true` means the request was accepted, not that ICP has converged.
Poll the separate status service:

```bash
rosservice call /localizer/relocalize_check "{code: 0}"
```

Only `valid: true` means the latest request converged. Until then the node
does not publish `map -> lidar`; a rejected request returns `success: false`
with its reason. The `code` field is retained for source-interface
compatibility and does not bypass the convergence result.

Read the fitted current position and orientation:

```bash
rostopic echo /localizer/pose
```

When a chassis transform publisher supplies `body -> base_link` (as in the
navigation launch), read the chassis reference pose through TF instead of
applying the LiDAR/body offset again. The standalone relocalization launch
does not publish this chassis transform:

```bash
rosrun tf tf_echo map base_link
```
