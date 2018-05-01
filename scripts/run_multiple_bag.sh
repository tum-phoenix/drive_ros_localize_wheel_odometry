#!/bin/bash

# parameter
BAG_FOLDER="/folder/to/bag/files"
LAUNCH_FILE="localize_wheel_odometry.launch"
PKG_NAME="drive_ros_localize_wheel_odometry"
CATKIN_WS="/lnk/phoenix/catkin_ws"

# source ros stuff
. /opt/ros/kinetic/setup.bash
. "$CATKIN_WS"/devel/setup.bash


for bag in "$BAG_FOLDER"/*.bag; do
    roslaunch "$PKG_NAME" "$LAUNCH_FILE" use_bag:=true bag_file:="$bag" csv_out:="$bag.csv"
done
