#!/bin/sh
# find where the directory is
DIR="$( cd "$( dirname "$0" )" && pwd )"
source $DIR/../../../devel/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE=$DIR/../World/test.world
# launch turtlebot Gazebo with test world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# gmapping
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$DIR/../World/map.yaml"&
sleep 5
# launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
# launch wall follower 
xterm  -e  " rosrun pick_objects pick_objects" &
sleep 5
