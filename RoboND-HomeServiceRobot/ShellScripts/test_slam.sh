#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../World/MyWorld.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 10
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"


