## Installation-SITL-Gazebo-ROS
Installation and integration guide on SITL, Gazebo and ROS.

## Introduction

This page intend to make you capable of communicating drones with programs written by you.
Also we are going to control a Quadcopter. OS we going to work is Ubuntu 18.04.

Before directly controlling a drone, we should test our code is working perfectly or not or to
identify the flaws in it. This we can achieve by running a simulation.

1. We need to setup a virtual drone.

2. We need to establish some connection between drone and program.

A simulation requires a virtual Pixhawk( Flight Controller ) so that to make it respond similar to
the original one, and a model of the drone with an environment so we can actually observe
what are the impacts of our program.

Inorder to do this we need to install some softwares:

1. Setting up an SITL : SITL allows you to have a virtual flight controller attached to a 3 D
modelled vehicle. So that we can observe the impacts.

2. Gazebo Simulation : Our club is following gazebo as its standard simulator. It is a light
weight simulator where we can observe what all are happening to the drone with less
CPU usage compared to others. Drone model can be imported during installation.

Now after these steps drone is set up in simulation which is fully functional. Now we need to establish a
connection. This can be done by ROS.

ROS have a subdivision called MAVROS , which converts all commands in our program into
messages( Mavlink messages) that can be interpreted by the FC ( Flight Controller) to perform a
task. These messages are transmitted to SITL on running the code as we can define which port
the messages have to be transmitted( Each computer has its own port ID).

So the flow is like this:

In simulation

Program -> Mavros -> Mavlink Messages -> SITL -> Drone in simulation environment.

In real drone

Program -> Mavros -> Mavlink Messages -> Pixhawk -> Drone movements.

So first we need to install and make our Laptop compatible for this.

## Installations

[Ubuntu](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/Ubuntu-18.04)

Just after Ubuntu installation your system will be clean.To install basic neccessities,

[Setup script and graphics driver](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/Setup-script-and-Driver)

Now our OS( Operating System) is ready. Now let's start with installation of required softwares.

Here is a setup script to set your system up with ros-melodic, sitl ( Ardupilot and px4 ) and gazebo.
If you are running this script you dont want to follow next installation steps.

https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/scripts/ros-gazebo.sh

If you are preferring manual installation go through this guide.

[ROS](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/ROS-Installation)

There are many firmwares available for drones.PX4 and Ardupilot are most common of them. 
You need either of PX4 or Ardupilot SITL.

[PX4](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/PX4-SITL-with-Gazebo-Simulation-and-ROS)

[Ardupilot](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/wiki/Ardupilot-SITL)

To launch sitl, gazebo and ros, download the arupilot.sh script for [ardupilot.sh](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/scripts/ardupilot.sh) and [px4_posix.sh](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/scripts/px4_posix.sh) for px4.

Run these script by ./ardupilot.sh or ./px4_posix

Also missions are carried out in 'Guided' mode in ardupilot and 'offboard' mode in px4. Also in px4, mode can only be changed if something is published to mavros topics.

Check these codes for arm and takeoff.

For c++ users : [arm.cpp](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/arm.cpp)

For python users : [arm.py](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/arm.py)

Check out these codes as a reference to write a node.

For c++ users : [square.cpp](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS)

For python users : [circle.py](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS/blob/master/circle.py)

For additional plugin checkout [plugins_setup](https://github.com/Aeroclub-IITM/Setting_up_sensors) section.
