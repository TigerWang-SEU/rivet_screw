#! /bin/bash
sleep 1
source /opt/ros/kinetic/setup.bash
source /home/syn/ros_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/syn/ros_ws/src/object_localizer/python:/home/syn/ros_ws/src/object_localizer/python/slim

rosservice call /start_image_transport
exit
