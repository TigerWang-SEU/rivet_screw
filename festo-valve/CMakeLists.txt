cmake_minimum_required(VERSION 2.8.3)
project(festo-valve)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)

find_package( catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
)
find_package ( Boost REQUIRED )
find_package ( PkgConfig REQUIRED )

include_directories ( modbuspp ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

catkin_package(
  INCLUDE_DIRS modbuspp
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
)

add_library ( modbus modbuspp/modbus.cpp )

add_executable ( festo_valve_server src/festo_valve_server.cpp )
target_link_libraries ( festo_valve_server modbus ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
