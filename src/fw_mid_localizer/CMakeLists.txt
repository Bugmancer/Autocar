cmake_minimum_required(VERSION 3.0.2)
project(fw_mid_localizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(yaml-cpp REQUIRED)

add_service_files(
  FILES
  IsValid.srv
  Relocalize.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src
)

if(PCL_DEFINITIONS)
  add_definitions(${PCL_DEFINITIONS})
endif()

add_executable(fw_mid_localizer_node
  src/localizer_node.cpp
  src/localizers/icp_localizer.cpp
)
add_dependencies(fw_mid_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_options(fw_mid_localizer_node PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_link_libraries(fw_mid_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS fw_mid_localizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
