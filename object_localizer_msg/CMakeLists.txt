cmake_minimum_required(VERSION 2.8.3)
project(object_localizer_msg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  BBox_int.msg
  BBox_float.msg
  BBox_list.msg
  Segment_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES object_localizer_msg
  #  CATKIN_DEPENDS std_msgs
  #  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
