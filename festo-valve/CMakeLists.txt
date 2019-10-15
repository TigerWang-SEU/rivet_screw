cmake_minimum_required(VERSION 2.8.3)
project(festo-valve)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)

find_package( catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES object_localizer
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS roscpp
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
