#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../World/MyWorld.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=$(pwd)/../wall_follower/launch/gmapping.launch" & 
sleep 7
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun wall_follower wall_follower"


