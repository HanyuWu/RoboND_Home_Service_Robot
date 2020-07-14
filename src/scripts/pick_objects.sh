#!/bin/sh
export  TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboND-Home-Service/src/map/apt.world
export  TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/RoboND-Home-Service/src/map/mymap.yaml
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &
