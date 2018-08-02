#!/bin/sh
# find where the directory is
DIR="$( cd "$( dirname "$0" )" && pwd )"
source $DIR/../../../devel/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE=$DIR/../World/test.world
# launch turtlebot Gazebo with test world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# AMCL
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$DIR/../World/map.yaml"&
sleep 5
# launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
# launch add markers 
#xterm  -e  " rosrun add_markers add_markers" &
sleep 5
