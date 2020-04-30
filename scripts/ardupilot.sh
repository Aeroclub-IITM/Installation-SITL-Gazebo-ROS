#!/bin/sh
#comment 
#All required things for ardupilot

echo "	welcome "
echo " Running roscore"
	xterm -title "roscore" -hold -e "roscore" &
        pid6=$!

echo "DO YOU WANT TO LAUNCH SITL TERMINAL [y/n]"
read input 
if [ $input = 'y' ] ; then
	xterm -title "SITL" -hold -e "sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0" &
        pid5=$!
fi

echo "DO YOU WANT TO LAUNCH GAZEBO TERMINAL [y/n]"
read input 
echo "your input is $input"
if [ "$input" = "y" ] ; then
	xterm -title "GAZEBO" -hold -e "gazebo --verbose iris_ardupilot.world" &
        pid4=$!
fi

echo "DO YOU WANT TO LAUNCH MAVROS TERMINAL [y/n]"
read input 
if [ $input = 'y' ] ; then
	xterm -title "MAVROS" -hold -e 'roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"' &
        pid3=$!
fi

echo "DO YOU WANT TO LAUNCH QGROUND [y/n]"
read input
if [ $input = 'y' ]; then
	xterm -title "QGC" -hold -e "../QGroundControl.AppImage" &
        pid2=$!
fi

echo "DO YOU WANT TO Close All Terminals [y/n] "
read input 
if [ $input = 'y' ] ; then
		kill -9 $pid2
        kill -9 $pid3
        kill -2 $pid5
        kill -9 $pid6
fi
echo end 




