cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_global_planner)

find_package(catkin REQUIRED COMPONENTS
  fw_mid_common_utils
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(CATKIN_DEPENDS fw_mid_common_utils)

catkin_install_python(PROGRAMS
  scripts/astar_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
