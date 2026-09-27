cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_common_utils)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
