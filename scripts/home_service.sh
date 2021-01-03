#!/bin/sh

xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &

sleep 5

BASE_DIR=$(dirname "$(realpath -s "$0")")

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$BASE_DIR/../map/Apartment.world" &

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$BASE_DIR/../map/slam_generated.yaml" &

sleep 5

xterm -e "rosrun pick_objects pick_objects" &

xterm -e "roslaunch add_markers add_markers.launch" &

xterm -e "rosrun rviz rviz -d $BASE_DIR/../rvizConfig/config.rviz"
