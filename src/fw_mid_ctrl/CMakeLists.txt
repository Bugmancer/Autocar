cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_ctrl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/can_driver_node.py
  scripts/cmd_vel_to_command_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 随包安装 python-can，使车辆工作空间无需额外安装该 Python 依赖。
# 安装到节点脚本旁，确保安装空间中按 __file__ 查找也能导入。
install(DIRECTORY scripts/can
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
