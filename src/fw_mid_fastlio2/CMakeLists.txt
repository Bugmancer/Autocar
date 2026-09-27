cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_fastlio2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP QUIET)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2_ros pcl_ros pcl_conversions livox_ros_driver2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  src
)

add_definitions(-DMP_EN -DMP_PROC_NUM=2)
if(PCL_DEFINITIONS)
  add_definitions(${PCL_DEFINITIONS})
endif()

set(SRC_LIST
  src/lio_node.cpp
  src/utils.cpp
  src/map_builder/commons.cpp
  src/map_builder/ieskf.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
)

add_executable(fw_mid_fastlio2_node ${SRC_LIST})
add_dependencies(fw_mid_fastlio2_node ${catkin_EXPORTED_TARGETS})
target_compile_options(fw_mid_fastlio2_node PRIVATE -O3 -Wall -Wextra -Wpedantic)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fw_mid_fastlio2_node OpenMP::OpenMP_CXX)
endif()
target_link_libraries(fw_mid_fastlio2_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS fw_mid_fastlio2_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
