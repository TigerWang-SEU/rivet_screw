cmake_minimum_required ( VERSION 2.8.3 )
project ( microepsilon_scancontrol )

add_compile_options ( -std=c++11 )

find_package ( catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs std_msgs pcl_ros pcl_conversions )
find_package ( Boost REQUIRED )
find_package ( PkgConfig REQUIRED )
pkg_check_modules ( ARAVIS4 REQUIRED aravis-0.4 )

### BUILD ###
include_directories ( common/include /usr/local/include/mescan-0.1 ${ARAVIS4_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
FIND_LIBRARY ( LLT_LIBRARY  libllt.so.1 )
FIND_LIBRARY ( MESCAN_LIBRARY  libmescan.so.1 )

catkin_package(
   INCLUDE_DIRS common/include /usr/local/include/mescan-0.1
   LIBRARIES ${PROJECT_NAME} ${LLT_LIBRARY} ${MESCAN_LIBRARY}
   CATKIN_DEPENDS roscpp sensor_msgs std_srvs std_msgs
   DEPENDS Boost ARAVIS4
)

add_library ( ${PROJECT_NAME} common/src/${PROJECT_NAME}.cpp )
add_dependencies ( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( ${PROJECT_NAME} ${catkin_LIBRARIES} ${LLT_LIBRARY} ${MESCAN_LIBRARY} ${ARAVIS4_LIBRARIES} )

add_executable ( ${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp )
add_dependencies ( ${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( ${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
