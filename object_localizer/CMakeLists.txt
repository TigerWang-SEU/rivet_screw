cmake_minimum_required(VERSION 2.8.3)
project(object_localizer)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)

find_package( catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  OpenCV
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  image_transport
  object_localizer_msg
  std_srvs
)

find_package( Boost REQUIRED )
find_package( PkgConfig REQUIRED )
pkg_check_modules( ARAVIS6 REQUIRED aravis-0.6 )

### BUILD ###
include_directories ( /usr/local/include/mescan-0.2/ ${ARAVIS6_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
FIND_LIBRARY ( LLT_LIBRARY_2  libllt.so.0.2.0 /usr/local/lib )
FIND_LIBRARY ( MESCAN_LIBRARY_2  libmescan.so.0.2.0 /usr/local/lib )

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES object_localizer
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS roscpp std_msgs object_localizer_msg
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(avoid_obstacle_planner src/avoid_obstacle_planner.cpp)
target_link_libraries(avoid_obstacle_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(avoid_obstacle_planner_b src/avoid_obstacle_planner_b.cpp)
target_link_libraries(avoid_obstacle_planner_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcl_merger src/pcl_merger.cpp)
target_link_libraries(pcl_merger ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcl_merger_b src/pcl_merger_b.cpp)
target_link_libraries(pcl_merger_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cam_pc_processor src/cam_pc_processor.cpp)
target_link_libraries(cam_pc_processor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rough_localizer src/rough_localizer.cpp)
add_dependencies( rough_localizer ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rough_localizer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rough_localizer_b src/rough_localizer_b.cpp)
add_dependencies( rough_localizer_b ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rough_localizer_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(box_segmenter src/box_segmenter.cpp)
add_dependencies( box_segmenter ${catkin_EXPORTED_TARGETS} )
target_link_libraries(box_segmenter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(box_segmenter_b src/box_segmenter_b.cpp)
add_dependencies( box_segmenter_b ${catkin_EXPORTED_TARGETS} )
target_link_libraries( box_segmenter_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_planner src/scan_planner.cpp)
add_dependencies( scan_planner ${catkin_EXPORTED_TARGETS} )
target_link_libraries(scan_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_planner_b src/scan_planner_b.cpp)
add_dependencies( scan_planner_b ${catkin_EXPORTED_TARGETS} )
target_link_libraries(scan_planner_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(profile_merger src/profile_merger.cpp)
target_link_libraries(profile_merger ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(profile_merger_b src/profile_merger_b.cpp)
target_link_libraries(profile_merger_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(profile_merger_c src/profile_merger_c.cpp)
target_link_libraries(profile_merger_c ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(profile_merger_c_thread src/profile_merger_c_thread.cpp)
target_link_libraries(profile_merger_c_thread ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LLT_LIBRARY_2} ${MESCAN_LIBRARY_2} ${ARAVIS6_LIBRARIES})

add_executable(GetProfilesCallback src/ProfilesCallback/GetProfilesCallback.cpp)
target_link_libraries(GetProfilesCallback ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LLT_LIBRARY_2} ${MESCAN_LIBRARY_2} ${ARAVIS6_LIBRARIES})

add_executable(pc_writer src/pc_writer.cpp)
target_link_libraries(pc_writer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rivet_localizer src/rivet_localizer.cpp)
target_link_libraries(rivet_localizer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rivet_localizer_b src/rivet_localizer_b.cpp)
target_link_libraries(rivet_localizer_b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rivet_localizer_c src/rivet_localizer_c.cpp)
target_link_libraries(rivet_localizer_c ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rivet_localizer_c_15 src/rivet_localizer_c_15.cpp)
target_link_libraries(rivet_localizer_c_15 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
