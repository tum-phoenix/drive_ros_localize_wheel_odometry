#include "ros/ros.h"
#include "drive_ros_msgs/VehicleEncoder.h"
#include "nav_msgs/Odometry.h"

#include "drive_ros_encoder_filter/cov_elements.h"

ros::Publisher odom;
int encoder_ct = 0;

bool use_rel_pos = true;
double rel_pos = 0;

nav_msgs::Odometry odom_out;


void encoderCallback(const drive_ros_msgs::VehicleEncoder::ConstPtr& msg)
{

  // set header
  odom_out.header.frame_id = msg->header.frame_id;
  odom_out.header.stamp = msg->header.stamp;
  odom_out.child_frame_id = ""; // this is not a real odom information

  // caculate velocity
  double vel = 0, vel_var = 0, pos = 0, pos_var = 0;
  if(0 == encoder_ct)
  {
    ROS_ERROR("We need at least one encoder, to work properly!");
    return;

  }else{

    // calculate mean
    for(int i=0; i<encoder_ct; i++){
      vel     += msg->encoder[i].vel;
      vel_var += msg->encoder[i].vel_var;

      if(use_rel_pos)
      {
        pos     += msg->encoder[i].pos_rel;
        pos_var += msg->encoder[i].pos_rel_var;
      }else{
        pos     += msg->encoder[i].pos_abs;
        pos_var += msg->encoder[i].pos_abs_var;
      }

    }

    vel     = vel/(float)encoder_ct;
    vel_var = vel_var/(float)encoder_ct;
    pos     = pos/(float)encoder_ct;
    pos_var = pos_var/(float)encoder_ct;



  }

  // increment relative position
  if(use_rel_pos){
    pos = rel_pos += pos;
  }

  // save position and velocity in odom_out
  odom_out.pose.pose.position.x = pos;
  odom_out.pose.covariance[CovElem::lin_ang::linX_linX] = pos_var;
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
  use_rel_pos = pnh.param<bool>("use_rel_pos", true);
  ROS_INFO_STREAM("Loaded use_rel_pos: " << use_rel_pos);


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
