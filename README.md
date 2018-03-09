# Localization using wheel odometry
This filter transforms [vehicle encoder](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/VehicleEncoder.msg) to standard [odometry messages](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html). It integrates the relative wheel encoder distance and heading over time.

## 1 Encoder
If only 1 encoder is available:

This node will only integrate the relative distance for pos_x and use v for vel_x.

## 4 Encoder
If 4 encoder are available:

Heading will be calculated from difference of left and right wheel. Heading will be used to calculate pos_x, pos_y, vel_x, vel_y, theta.

The vehicle model is based on "Introduction of Autonomous Mobile Robots" in the "Mobile Robot Localization" chapter by Roland Siegwart and Illah R. Nourbaksh (2004).
