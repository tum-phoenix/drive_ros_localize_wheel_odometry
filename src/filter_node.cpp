#include "ros/ros.h"
#include "drive_ros_msgs/VehicleEncoder.h"
#include "nav_msgs/Odometry.h"

#include "drive_ros_encoder_filter/cov_elements.h"

ros::Publisher odom;
int encoder_ct = 0;

nav_msgs::Odometry odom_out;


void encoderCallback(const drive_ros_msgs::VehicleEncoder::ConstPtr& msg)
{

  // set header
  odom_out.header.frame_id = msg->header.frame_id;
  odom_out.header.stamp = msg->header.stamp;
  odom_out.child_frame_id = ""; // this is not a real odom information

  // caculate velocity
  double vel = 0, vel_var = 0;
  if(0 == encoder_ct)
  {
    ROS_ERROR("We need at least one encoder, to work properly!");
    return;

  }else{

    // calculate mean
    for(int i=0; i<encoder_ct; i++){
      vel     += msg->encoder[i].vel;
      vel_var += msg->encoder[i].vel_var;
    }
    vel     = vel/(float)encoder_ct;
    vel_var = vel_var/(float)encoder_ct;

  }

  // save velocities in odom
  odom_out.twist.twist.linear.x = vel;
  odom_out.twist.covariance[CovElem::lin_ang::linX_linX] = vel_var;

  // send out odom
  odom.publish(odom_out);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoder_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get parameters
  encoder_ct = pnh.param<int>("encoder_ct", 1);
  ROS_INFO_STREAM("Loaded encoder_ct: " << encoder_ct);


  // set all not used odom covariances to -1
  for(int i=0; i<36; i++)
  {
    odom_out.twist.covariance[i] = -1;
    odom_out.pose.covariance[i] = -1;
  }

  // setup subscriber and publisher
  odom = pnh.advertise<nav_msgs::Odometry>("enc_out", 100);
  ros::Subscriber sub = pnh.subscribe("enc_in", 100, encoderCallback);

  ros::spin();

  return 0;
}
