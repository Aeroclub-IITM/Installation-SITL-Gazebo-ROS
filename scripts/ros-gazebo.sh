#!/bin/bash

set -e

echo "*****************************************************************************"
echo
echo "Setup script to install ros melodic, Ardupilot sitl and px4 sitl with Gazebo "
echo
echo

read -p "Do you want to install ros-melodic-desktop-full ? [y/n]: " tempvar

tempvar=${tempvar:-q}

if [ "$tempvar" = "y" ]; then
    echo
    echo "Installing ros-melodic"
    echo
    cd $HOME
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    sudo apt update
    sudo apt install ros-melodic-desktop-full
    echo 'source /opt/ros/melodic/setup.zsh' >> ~/.zshrc
    #source ~/.zshrc
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo apt install python3-pip
    sudo pip3 install -U rospkg
    sudo pip3 install pymavlink
    sudo apt-get install python-rospkg
    sudo rosdep init
    rosdep update
    echo 
    echo "ROS installation completed"
    echo

elif [ "$tempvar" = "n" ];then
    echo
    echo "Skipping ros installation"
    echo
fi


read -p "Which SITL is required? Ardupilot(a) or px4 (p) or nothing(n) : " tempvar

tempvar=${tempvar:-q}

if [ "$tempvar" = "a" ]; then
    echo
    echo "Setting Ardupilot"
    echo
    cd $HOME
    sudo apt-get update
    sudo apt-get install git
    sudo apt-get install gitk git-gui
    git clone https://github.com/ArduPilot/ardupilot.git
    sudo chown -R $(whoami): $HOME/ardupilot
    cd ardupilot
    git submodule update --init --recursive
    echo
    echo "git clone completed"
    echo
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile
    echo
    echo "Building bebop board"
    echo
    ./waf configure --board bebop --static
    ./waf copter
    cd $HOME
    echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.zshrc
    echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.zshrc
    #source ~/.zshrc
    echo
    echo "Building ardupilot_gazebo plugin"
    echo
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update
    sudo apt install gazebo9 libgazebo9-dev
    echo 'export SVGA_VGPU10=0' >> ~/.zshrc
    git clone https://github.com/SwiftGust/ardupilot_gazebo
    sudo chown -R $(whoami): $HOME/ardupilot_gazebo
    cd $HOME
    cd ardupilot_gazebo
    git checkout gazebo9
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    cd ../../
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.zshrc
    echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/gazebo_models' >> ~/.zshrc
    echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/gazebo_worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.zshrc
    #source ~/.zshrc
    echo
    echo "Ardupilot Installation completed"
    echo

elif [ "$tempvar" = "p" ];then
    echo
    echo "Starting px4 installation"
    echo
    cd $HOME
    git clone https://github.com/PX4/Firmware
    sudo chown -R $(whoami): $HOME/Firmware
    wget -c https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_ros_melodic.sh
    source ubuntu_sim_ros_melodic.sh​
    echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$HOME/Firmware/build/px4_sitl_default/build_gazebo' >> ~/.zshrc
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/Firmware/Tools/sitl_gazebo/models' >> ~/.zshrc
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/Firmware/build/px4_sitl_default/build_gazebo' >> ~/.zshrc
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/Firmware' >> ~/.zshrc
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/Firmware/Tools/sitl_gazebo' >> ~/.zshrc
    sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
    #source ~/.zshrc
    sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
    cd Firmware
    DONT_RUN=1 make px4_sitl_default gazebo
    source ~/catkin_ws/devel/setup.zsh
    cd $HOME
    echo
    echo "px4 installation completed"
    echo

elif [ "$tempvar" = "n" ];then
    echo
    echo "Skipping this step"
    echo
fi

read -p "Do you want to install mavros (y/n): " tempvar

tempvar=${tempvar:-q}

if [ "$tempvar" = "y" ]; then
    cd $HOME
    echo
    echo "Installing Mavros"
    echo
    sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod a+x install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh
    sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins
    sudo apt-get install python-catkin-tools
    echo
    echo "Mavros installation completed"
    echo

elif [ "$tempvar" = "n" ];then
    echo
    echo "Skipping this step"
    echo
fi


read -p "Do you want see the basic cammands for launching: " tempvar

tempvar=${tempvar:-q}

if [ "$tempvar" = "y" ]; then
    xdg-open https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/Basic-Cammands

elif [ "$tempvar" = "n" ];then
    echo
    echo "Skipping this step"
    echo
fi

echo
echo "The END"
echo "ALL THE BEST"
echo
