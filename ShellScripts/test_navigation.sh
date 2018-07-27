#!/bin/sh
# find where the directory is
DIR="$( cd "$( dirname "$0" )" && pwd )"
export TURTLEBOT_GAZEBO_WORLD_FILE=$DIR/../World/test.world
# launch turtlebot Gazebo with test world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# launch ACML
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
# launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
