cmake_minimum_required(VERSION 2.8.3)
project(tpu_connection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  object_localizer_msg
)

find_package(OpenCV)
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES object_localizer
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS roscpp std_msgs object_localizer_msg
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

