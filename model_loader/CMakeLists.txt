cmake_minimum_required(VERSION 2.8.3)
project(model_loader)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  sensor_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES model_loader
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_reader src/scan_reader.cpp)
target_link_libraries(scan_reader ${catkin_LIBRARIES})

add_executable(tf_dummy src/tf_dummy.cpp)
target_link_libraries(tf_dummy ${catkin_LIBRARIES})

add_executable(add_and_remove_aircraft_frame src/add_and_remove_aircraft_frame.cpp)
target_link_libraries(add_and_remove_aircraft_frame ${catkin_LIBRARIES})
