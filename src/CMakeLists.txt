cmake_minimum_required(VERSION 3.0.2)
project(Autocar)

# 由 scripts/ros1.sh 加载系统 Noetic，源码仓库不保存机器相关的 catkin 绝对路径。
set(CATKIN_TOPLEVEL TRUE)
set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
find_package(catkin REQUIRED NO_POLICY_SCOPE)
unset(CATKIN_TOPLEVEL_FIND_PACKAGE)
catkin_workspace()
