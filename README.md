# Encoder Odometry
This filter transforms [vehicle encoder](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/VehicleEncoder.msg) to standard [odometry messages](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html). It uses [dead reckoning](https://en.wikipedia.org/wiki/Dead_reckoning) and integrates the position and heading over time.

## 1 Encoder
If only 1 encoder is available:

This node will only integrate the relative distance for pos_x and use v for vel_x.

## 4 Encoder
If 4 encoder are available: 

Heading will be calculated from difference of left and right wheel. Heading will be used to calculate pos_x, pos_y, vel_x, vel_y, theta. More infos can be found [here](https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf).
