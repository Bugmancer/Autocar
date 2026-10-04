cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_local_planner)

find_package(catkin REQUIRED COMPONENTS
  fw_mid_common_utils
  fw_mid_controller
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

# 安装可导入的 Python 包；策略、运行层和算法模块通过 setup.py 一起发布。
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    fw_mid_common_utils
    fw_mid_controller
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    visualization_msgs
)

# 三个脚本分别提供经典/APF/V3 可执行入口；launch 只选择其中一个。
catkin_install_python(PROGRAMS
  scripts/path_follower_node.py
  scripts/path_follower_node_v1.py
  scripts/path_follower_node_v3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
