rivet_screw
===========
A ros software package for screwing rivets. The used ROS version is Kinetic.

Including driver packages for the following hardware:
-----------------------------------------------------
  - RealSense D435
    - realsense
  - Pepperl+Fuchs R2000
    - pepperl_fuchs
  - ScanCONTROL_2900-50
    - microepsilon_scancontrol
  - Robot UR 10
    - Ur_modern_driver
  - Atlas Copco Focus 6000
    - focus-6000

Package Note
------------
- URDF description is in package syn_ur10_bringup
- The neural network and rivet localizer are in package object_localizer
- The control_node is in package motion_control

Installation Instructions
==========================

1, Install CUDA 9.0, libcudnn7, and tensorflow 1.8.0
---------------------------------
    cd ~/Downloads
    wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.44-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu1604_8.0.44-1_amd64.deb
    wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
    sudo dpkg -i nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
    sudo apt update
    sudo apt install cuda-9-0 libcudnn7=7.2.1.38-1+cuda9.0 libnccl2=2.2.13-1+cuda9.0
    echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc
    sudo reboot -h now
    pip install tensorflow-gpu==1.8.0

2, Install ROS Kinetic
----------------------
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
    ### build local ros workspace ###
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws
    catkin_make
    echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
3, Clone rivet_screw github code to the local ros workspace
------------------------------------------------------------
    ### clone local ros workspace ###
    cd ~/ros_ws/src
    rm -rf *
    git clone git-url .
    
    ### add python path for object detection neural network ###
    export PYTHONPATH=$PYTHONPATH:~/ros_ws/src/object_localizer/python:~/ros_ws/src/object_localizer/python/slim

4, Install moveit
-----------------
    sudo apt-get install ros-kinetic-moveit ros-kinetic-moveit-visual-tools

5, Install ScanCONTROL_2900-50 related head files and libs
----------------------------------------------------------
    sudo mkdir /usr/local/include/mescan-0.2
    sudo cp ~/ros_ws/src/object_localizer/SDK_0.2.0/include/libllt/* /usr/local/include/mescan-0.2
    sudo cp ~/ros_ws/src/object_localizer/SDK_0.2.0/include/libmescan/* /usr/local/include/mescan-0.2
    sudo cp ~/ros_ws/src/object_localizer/SDK_0.2.0/so/* /usr/local/lib
    ###install aravis 0.6 ###
        cd ~/Downloads
        sudo apt-get install intltool
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
        tar -xf aravis-0.6.0.tar.xz
        cd aravis-0.6.0
        ./configure
        make
        sudo make install
        sudo ldconfig

    sudo mkdir /usr/local/include/mescan-0.1
    sudo cp ~/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.h /usr/local/include/mescan-0.1
    sudo cp ~/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.h /usr/local/include/mescan-0.1
    sudo cp ~/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.so.1.0 /usr/local/lib
    sudo cp ~/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.so.1.0 /usr/local/lib
    ###install aravis 0.4 ###
        cd ~/Downloads
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.0.tar.xz
        tar -xf aravis-0.4.0.tar.xz
        cd aravis-0.4.0
        ./configure
        make
        sudo make install
        sudo ldconfig

5, Install realsense driver and related packages
------------------------------------------------
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

    sudo apt-get update
    sudo apt-get install librealsense2-udev-rules=2.22.0-0~realsense0.1105 librealsense2-dkms=1.3.3-0ubuntu1 librealsense2=2.22.0-0~realsense0.1105 librealsense2-utils=2.22.0-0~realsense0.1105 librealsense2-gl=2.22.0-0~realsense0.1105 librealsense2-dev=2.22.0-0~realsense0.1105 librealsense2-dbg=2.22.0-0~realsense0.1105 -y

    sudo apt-mark hold librealsense2-udev-rules librealsense2-dkms librealsense2 librealsense2-utils librealsense2-gl librealsense2-dev librealsense2-dbg

    sudo apt-get install ros-kinetic-ddynamic-reconfigure ros-kinetic-rgbd-launch

6, Install UR robot related packages
------------------------------------
    sudo apt-get install ros-kinetic-universal-robot ros-kinetic-ur-msgs
    sudo apt-get install ros-kinetic-industrial-robot-status-interface ros-kinetic-rqt-controller-manager

7, Install TPU related PYTHON modules
------------------------------------
    sudo apt install python-pip
    pip install grpcio protobuf
