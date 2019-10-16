cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    tf2
    tf2_ros
    std_srvs
    pcl_ros
    pcl_conversions
    object_localizer_msg
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    object_localizer_msg
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories ( SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} )
include_directories ( ${catkin_INCLUDE_DIRS} )
link_directories ( ${catkin_LIBRARY_DIRS} )

add_executable(avoid_obstacle src/avoid_obstacle.cpp)
target_link_libraries(avoid_obstacle ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_node_b src/control_node_b.cpp)
target_link_libraries(control_node_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_node_plan_test src/control_node_plan_test.cpp)
target_link_libraries(control_node_plan_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(control_node_test src/control_node_test.cpp)
target_link_libraries(control_node_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_camera src/move_camera.cpp)
target_link_libraries(move_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_camera_b src/move_camera_b.cpp)
target_link_libraries(move_camera_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(do_scan src/do_scan.cpp)
target_link_libraries(do_scan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(do_scan_b src/do_scan_b.cpp)
target_link_libraries(do_scan_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(point_rivet_c src/point_rivet_c.cpp)
target_link_libraries ( point_rivet_c ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
