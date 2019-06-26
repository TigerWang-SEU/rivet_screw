Installation Instructions
==========================

1, install CUDA 9.0 and libcudnn7
---------------------------------
    wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.44-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu1604_8.0.44-1_amd64.deb
    wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
    sudo dpkg -i nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
    sudo apt update
    sudo apt install cuda-9-0 libcudnn7=7.2.1.38-1+cuda9.0 libnccl2=2.2.13-1+cuda9.0
    echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc
    sudo reboot -h now

2, install ROS Kinetic
----------------------
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    ### create catkin workspace ###
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws
    catkin_make
    echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

3, install moveit
-----------------
    sudo apt-get install ros-kinetic-moveit ros-kinetic-moveit-visual-tools

4, install me related head files and libs
-----------------------------------------
    sudo mkdir /usr/local/include/mescan-0.2
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/include/libllt/* /usr/local/include/mescan-0.2
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/include/libmescan/* /usr/local/include/mescan-0.2
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/lib/x86_64/* /usr/local/lib
    ###install aravis 0.6 ###
        cd ~/Downloads
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
        tar -xf aravis-0.6.0.tar.xz
        cd aravis-0.6.0
        ./configure
        make
        make install
        ldconfig

    sudo mkdir /usr/local/include/mescan-0.1
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.h /usr/local/include/mescan-0.1
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.h /usr/local/include/mescan-0.1
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.so.1.0 /usr/local/lib
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.so.1.0 /usr/local/lib
    ###install aravis 0.4 ###
        cd ~/Downloads
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.0.tar.xz
        tar -xf aravis-0.4.0.tar.xz
        cd aravis-0.4.0
        ./configure
        make
        make install
        ldconfig

5, install realsense driver and related packages
------------------------------------------------
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils
    sudo apt-get install librealsense2-dev
    sudo apt-get install librealsense2-dbg

    sudo apt-get install ros-kinetic-ddynamic-reconfigure ros-kinetic-rgbd-launch

6, install UR robot related packages
------------------------------------
    sudo apt-get install ros-kinetic-universal-robot ros-kinetic-ur-msgs
    sudo apt-get install ros-kinetic-industrial-robot-status-interface ros-kinetic-rqt-controller-manager 

7, clone rivet_screw github code to the local ros work space
--------------------------------
    ###add python path for object detection neural network###
    export PYTHONPATH=$PYTHONPATH:~/ros_ws/src/object_localizer/python:~/ros_ws/src/object_localizer/python/slim
