cmake_minimum_required( VERSION 2.8.3 )
project( me_scancontrol_b )

add_compile_options( -std=c++11 )

find_package( catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs std_msgs pcl_ros pcl_conversions )
find_package( Boost REQUIRED )
find_package( PkgConfig REQUIRED )
pkg_check_modules( ARAVIS6 REQUIRED aravis-0.6 )

### BUILD ###
include_directories ( common/include /usr/local/include/mescan-0.2/ ${ARAVIS6_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
FIND_LIBRARY ( LLT_LIBRARY_2  libllt.so.0.2.0 /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/lib/x86_64 )
FIND_LIBRARY ( MESCAN_LIBRARY_2  libmescan.so.0.2.0 /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/lib/x86_64 )

catkin_package(
   INCLUDE_DIRS common/include /usr/local/include/mescan-0.2/
   LIBRARIES ${PROJECT_NAME} ${LLT_LIBRARY_2} ${MESCAN_LIBRARY_2}
   CATKIN_DEPENDS roscpp sensor_msgs std_srvs std_msgs
   DEPENDS Boost ARAVIS6
)

add_library ( me_scancontrol_b common/src/me_scancontrol_b.cpp )
add_dependencies ( me_scancontrol_b ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( me_scancontrol_b ${catkin_LIBRARIES} ${LLT_LIBRARY_2} ${MESCAN_LIBRARY_2} ${ARAVIS6_LIBRARIES} )

add_executable ( me_scancontrol_b_node src/me_scancontrol_b_node.cpp )
add_dependencies ( me_scancontrol_b_node me_scancontrol_b ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( me_scancontrol_b_node me_scancontrol_b ${catkin_LIBRARIES} )

add_executable ( GetProfilesCallback src/GetProfilesCallback.cpp )
add_dependencies ( GetProfilesCallback ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( GetProfilesCallback ${catkin_LIBRARIES} ${LLT_LIBRARY_2} ${MESCAN_LIBRARY_2} ${ARAVIS6_LIBRARIES} )
