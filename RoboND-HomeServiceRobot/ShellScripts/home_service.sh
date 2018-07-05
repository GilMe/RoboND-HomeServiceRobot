#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../World/MyWorld.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../World/myMap.yaml" & 
sleep 10
xterm  -e  " roslaunch add_markers view_navigation.launch rviz_config_file:=$(pwd)/../RvizConfig/rviz_with_marker.rviz" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects"


