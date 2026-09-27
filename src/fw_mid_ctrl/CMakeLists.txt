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

# python-can is vendored with this package so the vehicle workspace does not
# depend on a separately installed Python package.  Install it beside the
# relayed scripts; this is also where __file__ points in an install space.
install(DIRECTORY scripts/can
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
