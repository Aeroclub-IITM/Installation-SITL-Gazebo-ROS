#!/bin/sh
#comment 
#comment sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0
echo "	welcome "

echo "DO YOU WANT TO SITL TERMINAL [y/n]"
read input 
if [ $input = 'y' ] ; then
	xterm -title "SITL" -hold -e "cd;
	cd Firmware;DONT_RUN=1 make px4_sitl_default gazebo;
	source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;
	roslaunch px4 posix_sitl.launch"  &
        pid5=$!
fi

echo "DO YOU WANT TO LAUNCH MAVROS TERMINAL [y/n]"
read input 
if [ $input = 'y' ] ; then
	xterm -title "MAVROS" -hold -e 'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"' &
        pid3=$!
fi

echo "DO YOU WANT TO LAUNCH QGROUND [y/n]"
read input
if [ $input = 'y' ]; then
	xterm -title "QGC" -hold -e "../QGroundControl.AppImage" &
        pid2=$!
fi

#Enter your package and program name for arm-takeoff
echo "DO YOU WANT TO ARM THE DRONE [y/n]"
read input 
if [ $input = 'y' ] ; then                                                         
	xterm -title "ROS NODE FOR ARMING " -hold -e "cd;
	cd catkin_ws;      
	cd src;
	cd smartcopter;                                                       
	python3 arm.py;" &
fi

echo "DO YOU WANT TO Close All Terminals [y/n] "
read input 
if [ $input = 'y' ] ; then
        kill -9 $pid3
        kill -2 $pid2
        kill -2 $pid5
fi

echo end 




