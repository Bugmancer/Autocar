cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_controller)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS fw_mid_common_utils
)
