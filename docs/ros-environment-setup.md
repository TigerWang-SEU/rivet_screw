Installation Instructions
==========================

1, install CUDA 9.0 and libcudnn7
---------------------------------
    wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.44-1_amd64.deb <br />
    sudo dpkg -i cuda-repo-ubuntu1604_8.0.44-1_amd64.deb <br />
    wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb <br />
    sudo dpkg -i nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb <br />
    sudo apt update <br />
    sudo apt install cuda-9-0 libcudnn7=7.2.1.38-1+cuda9.0 libnccl2=2.2.13-1+cuda9.0 <br />
    echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}" >> ~/.bashrc <br />
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc <br />
    sudo reboot -h now <br />

2, install ROS Kinetic
----------------------
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' <br />
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 <br />
    sudo apt-get install ros-kinetic-desktop-full <br />
    sudo rosdep init <br />
    rosdep update <br />
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc <br />
    source ~/.bashrc <br />
    
    ### create catkin workspace ###
    mkdir -p ~/ros_ws/src <br />
    cd ~/ros_ws <br />
    catkin_make <br />
    echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc <br />
    source ~/.bashrc <br />

3, install moveit
-----------------
    sudo apt-get install ros-kinetic-moveit ros-kinetic-moveit-visual-tools

4, install me related head files and libs
-----------------------------------------
    sudo mkdir /usr/local/include/mescan-0.2 <br />
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/include/libllt/* /usr/local/include/mescan-0.2 <br />
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/include/libmescan/* /usr/local/include/mescan-0.2 <br />
    sudo cp /home/syn/ros_ws/src/me_scancontrol_b/scanCONTROL_Linux_SDK_0.2.0/lib/x86_64/* /usr/local/lib <br />
    ###install aravis 0.6 ###
        cd ~/Downloads <br />
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz <br />
        tar -xf aravis-0.6.0.tar.xz <br />
        cd aravis-0.6.0 <br />
        ./configure <br />
        make <br />
        make install <br />
        ldconfig <br />
    <br />
    sudo mkdir /usr/local/include/mescan-0.1 <br />
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.h /usr/local/include/mescan-0.1 <br />
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.h /usr/local/include/mescan-0.1 <br />
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libllt/*.so.1.0 /usr/local/lib <br />
    sudo cp /home/syn/ros_ws/src/microepsilon_scancontrol/scanCONTROL_Linux_SDK_0.1.0/libmescan/*.so.1.0 /usr/local/lib <br />
    ###install aravis 0.4 ###
        cd ~/Downloads <br />
        wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.0.tar.xz <br />
        tar -xf aravis-0.4.0.tar.xz <br />
        cd aravis-0.4.0 <br />
        ./configure <br />
        make <br />
        make install <br />
        ldconfig <br />

5, install realsense driver and related packages
------------------------------------------------
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE <br />
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u <br />
    sudo apt-get install librealsense2-dkms <br />
    sudo apt-get install librealsense2-utils <br />
    sudo apt-get install librealsense2-dev <br />
    sudo apt-get install librealsense2-dbg <br />
    <br />
    sudo apt-get install ros-kinetic-ddynamic-reconfigure ros-kinetic-rgbd-launch <br />

6, install UR robot related packages
------------------------------------
    sudo apt-get install ros-kinetic-universal-robot ros-kinetic-industrial-robot-status-interface ros-kinetic-rqt-controller-manager ros-kinetic-ur-msgs
