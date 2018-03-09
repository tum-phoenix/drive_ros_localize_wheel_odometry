# Localization using wheel odometry
This filter transforms [vehicle encoder](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/VehicleEncoder.msg) to standard [odometry messages](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html). It integrates the relative wheel encoder distance and heading over time. It does not output velocity information!

It assumes that the vehicle has 4 wheel encoder. Heading will be calculated from the difference of left and right wheel. An additional moving average filter will be used to smooth heading. The vehicle model is based on *Introduction of Autonomous Mobile Robots* by Roland Siegwart and Illah R. Nourbaksh from 2004 (page 186 ff).
