#!/bin/sh

xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &

sleep 5

BASE_DIR=$(dirname "$(realpath -s "$0")")
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$BASE_DIR/../map/Apartment.world" &

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
