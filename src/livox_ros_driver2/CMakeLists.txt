# This checkout is the ROS1/Noetic copy used by Autocar_v1.  Keep the
if(NOT DEFINED ROS_EDITION)
  set(ROS_EDITION "ROS1")
endif()

if(ROS_EDITION STREQUAL "ROS1")

  # Copyright(c) 2019 livoxtech limited.

  cmake_minimum_required(VERSION 3.0)


  #---------------------------------------------------------------------------------------
  # Start livox_ros_driver2 project
  #---------------------------------------------------------------------------------------
  include(cmake/version.cmake)
  project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  message(STATUS "livox_ros_driver2 version: ${LIVOX_ROS_DRIVER2_VERSION}")

  #---------------------------------------------------------------------------------------
  # Add ROS Version MACRO
  #---------------------------------------------------------------------------------------
  add_definitions(-DBUILDING_ROS1)

  #---------------------------------------------------------------------------------------
  # find package and the dependecy
  #---------------------------------------------------------------------------------------
  find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
  )

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    rosbag
    pcl_ros
  )

  ## Find pcl lib
  find_package(PCL REQUIRED)

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
  #   Message2.msg
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(DEPENDENCIES
    std_msgs
  )

  find_package(PkgConfig)
  pkg_check_modules(APR apr-1)
  if (APR_FOUND)
    message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
  endif (APR_FOUND)

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects als    o need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also n    eed
  catkin_package(CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime
    pcl_ros
  )

  #---------------------------------------------------------------------------------------
  # Set default build to release
  #---------------------------------------------------------------------------------------
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
  endif()

  #---------------------------------------------------------------------------------------
  # Compiler config
  #---------------------------------------------------------------------------------------
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Build the SDK bundled with this project.  The driver must not silently
  # pick up a Livox SDK from another workspace or from /usr/local.
  set(LIVOX_SDK_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Livox-SDK2")
  if(NOT EXISTS "${LIVOX_SDK_SOURCE_DIR}/sdk_core/CMakeLists.txt")
    message(FATAL_ERROR
      "The bundled Livox SDK is missing from ${LIVOX_SDK_SOURCE_DIR}.")
  endif()
  add_subdirectory(
    "${LIVOX_SDK_SOURCE_DIR}/sdk_core"
    "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk_core"
    EXCLUDE_FROM_ALL
  )
  set(LIVOX_LIDAR_SDK_LIBRARY livox_lidar_sdk_static)

  ## PCL library
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  #---------------------------------------------------------------------------------------
  # generate excutable and add libraries
  #---------------------------------------------------------------------------------------
  add_executable(${PROJECT_NAME}_node
    ""
  )

  #---------------------------------------------------------------------------------------
  # precompile macro and compile option
  #---------------------------------------------------------------------------------------
  target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
  )

  #---------------------------------------------------------------------------------------
  # add projects that depend on
  #---------------------------------------------------------------------------------------
  add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

  #---------------------------------------------------------------------------------------
  # source file
  #---------------------------------------------------------------------------------------
  target_sources(${PROJECT_NAME}_node
    PRIVATE
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/lddc.cpp
    src/livox_ros_driver2.cpp

    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp

    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp

    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  #---------------------------------------------------------------------------------------
  # include file
  #---------------------------------------------------------------------------------------
  target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_SDK_SOURCE_DIR}/include
    ${LIVOX_SDK_SOURCE_DIR}/3rdparty
    src
  )

  #---------------------------------------------------------------------------------------
  # link libraries
  #---------------------------------------------------------------------------------------
  target_link_libraries(${PROJECT_NAME}_node
    ${LIVOX_LIDAR_SDK_LIBRARY}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
    ${CMAKE_DL_LIBS}
    pthread
  )


  #---------------------------------------------------------------------------------------
  # Install
  #---------------------------------------------------------------------------------------

  install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch_ROS1/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_ROS1
  )

  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

  install(DIRECTORY msg/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  )

  install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    FILES_MATCHING PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
  )
  catkin_install_python(PROGRAMS scripts/check_mid360_topics.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # Keep the SDK headers available to downstream builds from the catkin
  # install space as well.  The executable itself links the static target.
  install(DIRECTORY ${LIVOX_SDK_SOURCE_DIR}/include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/livox_sdk2
  )

  #---------------------------------------------------------------------------------------
  # end of CMakeList.txt
  #---------------------------------------------------------------------------------------


else()
  message(FATAL_ERROR "Unsupported ROS_EDITION='${ROS_EDITION}'. Use ROS1 for this workspace.")
endif()
