#!/bin/bash

# parameter
BAG_FOLDER="/media/fabianhanke/35010486-8770-47c0-a4d6-3088821d0236/2018-04-18_Gerade_Strecke"
LAUNCH_FILE="localize_wheel_odometry.launch"
PKG_NAME="drive_ros_localize_wheel_odometry"
CATKIN_WS="/lnk/phoenix/catkin_ws"

# source ros stuff
. /opt/ros/kinetic/setup.bash
. "$CATKIN_WS"/devel/setup.bash


for bag in "$BAG_FOLDER"/*.bag; do
    roslaunch "$PKG_NAME" "$LAUNCH_FILE" use_bag:=true bag_file:="$bag"
done
