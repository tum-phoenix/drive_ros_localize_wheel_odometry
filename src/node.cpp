#include "ros/ros.h"
#include "drive_ros_msgs/VehicleEncoder.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "drive_ros_localize_wheel_odometry/cov_elements.h"


int encoder_ct = 0;           // number of encoder
float theta = 0;              // heading
double track_width;           // distance between wheels on an axis

ros::Publisher odom;          // publisher
nav_msgs::Odometry odom_out;  // published message


void encoderCallback(const drive_ros_msgs::VehicleEncoder::ConstPtr& msg)
{

  // set header
  odom_out.header.stamp = msg->header.stamp;
  odom_out.child_frame_id = msg->header.frame_id;


  // caculate velocity
  float vel(0), vel_var(0), delta_s(0), delta_s_var(0);


  // calculate mean
  for(int i=0; i<encoder_ct; i++){

    vel     += msg->encoder[i].vel;
    vel_var += msg->encoder[i].vel_var;

    delta_s     += msg->encoder[i].pos_rel;
    delta_s_var += msg->encoder[i].pos_rel_var;

  }

  vel         = vel/(float)encoder_ct;
  vel_var     = vel_var/(float)encoder_ct;
  delta_s     = delta_s/(float)encoder_ct;
  delta_s_var = delta_s_var/(float)encoder_ct;


  switch (encoder_ct) {

    // we have one motor encoder
    case 1:
    {
      // save position and velocity in odom_out
      odom_out.pose.pose.position.x += delta_s;
      odom_out.pose.covariance[CovElem::lin_ang::linX_linX] = delta_s_var;
      odom_out.twist.twist.linear.x = vel;
      odom_out.twist.covariance[CovElem::lin_ang::linX_linX] = vel_var;
      break;
    }

    // we have 4 encoder (one for each wheel)
    case 4:
    {
      double dtheta_f, dtheta_r;

      dtheta_f = ( msg->encoder[msg->FRONT_WHEEL_RIGHT].pos_rel -
                   msg->encoder[msg->FRONT_WHEEL_LEFT].pos_rel ) / track_width;

      dtheta_r = ( msg->encoder[msg->REAR_WHEEL_RIGHT].pos_rel -
                   msg->encoder[msg->REAR_WHEEL_LEFT].pos_rel ) / track_width;


      float dtheta = (dtheta_f + dtheta_r) / 2.0;


      // save position and velocity in odom_out
      odom_out.pose.pose.position.x += delta_s * cos(theta + dtheta/2);
      odom_out.pose.covariance[CovElem::lin_ang::linX_linX] = 0;
      odom_out.pose.pose.position.y += delta_s * sin(theta + dtheta/2);
      odom_out.pose.covariance[CovElem::lin_ang::linY_linY] = 0;
      odom_out.twist.twist.linear.x = vel;
      odom_out.twist.covariance[CovElem::lin_ang::linX_linX] = vel_var;

      // integrate theta
      theta += dtheta;

      // save relative theta in odom_out
      tf::Quaternion q;
      q.setRPY(float(0), float(0), theta);
      tf::quaternionTFToMsg(q, odom_out.pose.pose.orientation);
      odom_out.twist.covariance[CovElem::lin_ang::angZ_angZ] = 0;

      break;
    }

    // unknown number of encoder
    default:
    {
      ROS_INFO_STREAM("Unknown number of encoder. Currently only 1 or 4 encoders are supported.");
      return;
    }

  }

  // send out odom
  odom.publish(odom_out);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localize_wheel_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get parameters
  encoder_ct = pnh.param<int>("encoder_ct", 1);
  ROS_INFO_STREAM("Loaded encoder_ct: " << encoder_ct);

  track_width = pnh.param<double>("track_width", 0.22);
  ROS_INFO_STREAM("Loaded track_width: " << track_width);

  odom_out.header.frame_id = pnh.param<std::string>("static_frame_id", "odom");
  ROS_INFO_STREAM("Loaded static_frame: " << odom_out.header.frame_id);

  // set all not used odom covariances to -1
  for(int i=0; i<36; i++)
  {
    odom_out.twist.covariance[i] = -1;
    odom_out.pose.covariance[i] = -1;
  }

  // check encoder_ct
  if(0 == encoder_ct)
  {
    ROS_ERROR("We need at least one encoder, to work properly!");
    return 1;
  }

  // setup subscriber and publisher
  odom = pnh.advertise<nav_msgs::Odometry>("odom_out", 100);
  ros::Subscriber sub = pnh.subscribe("enc_in", 100, encoderCallback);

  ros::spin();

  return 0;
}
