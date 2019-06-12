#author: ozan güngör#
# A guide to successfully compile platform-ros packages in a catkin_ws for ros-kinetic #
# following packages have been successfully built with this guideline #
# focus-6000, industrial_core, microepsilon_scancontrol, model_loader, motion_control, object_localizer, object_localizer_msg, pepperl_fuchs, realsense2_camera, syn_ur10_bringup, ur_modern_driver #
# I managed compiling the same workspace in ros-melodic without the object_localizer package #

# create catkin workspace #
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# install moveit and visual tools #
apt-get install ros-kinetic-moveit
apt-get install ros-kinetic-moveit-visual-tools

# install ur10 package #
apt-get install ros-kinetic-universal-robot

# install ros industrial core #
apt-get install ros-kinetic-industrial-core

# install real-sense camera packages #
# due to a compiling bug for realsense camera package, there is a new version available and it requires the latest SDK as well as an additional package called ddynamic_reconfigure #
# clone the ddynamic_reconfigure package to catkin_ws 
git clone https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel

# for the latest realsense SDK #
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# clone the realsense package to your catkin_ws #
git clone https://github.com/IntelRealSense/realsense-ros.git

# if you are using one of the older versions of realsense package you can use the following SDK as well #
# for realsense SDK 15 #
apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
rmsu -f /etc/apt/sources.list.d/realsense-public.list.
apt-get install librealsense2-udev-rules=2.15.0-0~realsense0.83
apt-get install librealsense2=2.15.0-0~realsense0.83 -y
apt-get install librealsense2-utils=2.15.0-0~realsense0.83 -y
apt-get install librealsense2-dev=2.15.0-0~realsense0.83 -y
apt-get install librealsense2-dbg=2.15.0-0~realsense0.83 -y

# install aravis 0.4 #
cd /opt
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.0.tar.xz
tar -xf aravis-0.4.0.tar.xz
cd aravis-0.4.0
./configure
make 
make install
ldconfig

#install libmescan and libllt libraries # 
# copy libllt.h LLTDataTypes.h and libmescan.h files to /usr/local/include/mescan-0.1
cd /usr/local/include
mkdir mescan-0.1
scp ~/catkin_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/libllt.h  mescan-0.1/ 
scp ~/catkin_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/LLTDataTypes.h  mescan-0.1/
sudo scp ~/catkin_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/libmescan.h  mescan-0.1/

# copy libllt.so.1.0 and libmescan.so.1.0 files to /usr/local/lib
cd /usr/local/lib
scp ~/catkin_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/libllt.so.1.0 ./
scp ~/catkin_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/libmescan.so.1.0 ./
sudo ldconfig

# if you are getting an error due to tf commands within model_loader package, copy the following line at the beginning of each .cpp file within the src folder # 
 #include <tf/transform_broadcaster.h>

# if you are gettin an error due to missing rgbd_launch or xacro packages, clone the following the packages to catkin_ws #
# make sure they are for kinetic #
git clone https://github.com/ros-drivers/rgbd_launch.git
git clone https://github.com/ros/xacro.git

