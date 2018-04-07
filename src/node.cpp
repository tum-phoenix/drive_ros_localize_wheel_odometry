#include "ros/ros.h"
#include "drive_ros_msgs/VehicleEncoder.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "eigen3/Eigen/Dense"
#include "std_srvs/Trigger.h"

#include "drive_ros_localize_wheel_odometry/cov_elements.h"
#include "drive_ros_localize_wheel_odometry/moving_average.h"

// matrix indices
static constexpr size_t X     = 0;
static constexpr size_t Y     = 1;
static constexpr size_t THETA = 2;

static constexpr size_t DELTA_R = 0;
static constexpr size_t DELTA_L = 1;


const int encoder_ct = 4;                // number of encoder (assume we have 4 wheel encoder)
double theta = 0;                        // heading
double track_width;                      // actual distance between wheels on an axis
double err_rl;                           // error factor between left and right
bool broadcast_tf;                       // whether to broadcast tf
bool first_msg = true;                   // is first message ?
bool use_sensor_time_for_pub = true;     // use sensor time or not
bool use_static_cov = true;              // use static covariances or not
std::string static_frame_id;             // static frame id

ros::ServiceServer reset_svr;            // reset service
ros::Publisher odom_pub;                 // publisher
nav_msgs::Odometry odom_out;             // published message
tf::TransformBroadcaster* br;            // broadcast transform

drive_ros_msgs::VehicleEncoder msg_old;  // old vehicle encoder message
Eigen::Matrix<double, 3, 3> Sigma_p;     // covariance matrix of previous step
MovingAverage* theta_filter;             // moving average filter for theta

std::vector<float> initial_cov;          // initial covariances
std::vector<double> static_cov;          // static covariances


// reset
void reset()
{
  ROS_INFO_STREAM("Reset Wheel Odometry.");

  // new odometry should be zero
  odom_out = nav_msgs::Odometry();

  theta = 0;
  theta_filter->clear();

  first_msg = true;

  // set all not used odom covariances to -1
  for(int i=0; i<36; i++)
  {
    odom_out.twist.covariance[i] = -1;
    odom_out.pose.covariance[i] = -1;
  }

  // set initial covariances
  for(int i=0; i<9; i++)
  {
    Sigma_p(i/3, i%3) = initial_cov.at(i);
  }
  ROS_INFO_STREAM("Set Sigma_P: " << std::endl << Sigma_p);
}

// services to reset odometry
bool svrResetOdom(std_srvs::Trigger::Request  &req,
                  std_srvs::Trigger::Response &res)
{

  reset();
  res.message = "Reset wheel odometry.";
  return res.success = true;
}

// encoder callback
void encoderCallback(const drive_ros_msgs::VehicleEncoder::ConstPtr& msg)
{

  // check if first message
  if(first_msg)
  {
    first_msg = false;

    // set old encoder message
    for(int i=0; i<encoder_ct; i++)
    {
      msg_old.encoder[i].pos_abs     = msg->encoder[i].pos_abs;
      msg_old.encoder[i].pos_abs_var = msg->encoder[i].pos_abs_var;
      msg_old.encoder[i].pos_rel     = msg->encoder[i].pos_rel;
      msg_old.encoder[i].pos_rel_var = msg->encoder[i].pos_rel_var;
    }

    // return
    ROS_INFO("Got first message.");
    return;
  }


  // set header
  ros::Time out_time;
  if(use_sensor_time_for_pub){
    out_time = msg->header.stamp;
  }else{
    out_time = ros::Time::now();
  }
  odom_out.header.frame_id = static_frame_id;
  odom_out.header.stamp = out_time;
  odom_out.child_frame_id = msg->header.frame_id;


  double delta_s(0), vel(0), vel_var(0);

  // calculate mean
  for(int i=0; i<encoder_ct; i++){

    // use absolute position to avoid errors from message drops
    delta_s += msg->encoder[i].pos_abs - msg_old.encoder[i].pos_abs;
    vel     += msg->encoder[i].vel;
    vel_var += msg->encoder[i].vel_var;

    // save old message
    msg_old.encoder[i].pos_abs = msg->encoder[i].pos_abs;
  }

  delta_s = delta_s/(double)encoder_ct;
  vel     =     vel/(double)encoder_ct;
  vel_var = vel_var/(double)encoder_ct;


  double dtheta_f = ( msg->encoder[msg->FRONT_WHEEL_RIGHT].pos_rel * err_rl -
                      msg->encoder[msg->FRONT_WHEEL_LEFT].pos_rel  / err_rl ) / track_width ;

  double dtheta_r = ( msg->encoder[msg->REAR_WHEEL_RIGHT].pos_rel * err_rl -
                      msg->encoder[msg->REAR_WHEEL_LEFT].pos_rel  / err_rl)  / track_width;


  // use mean of front and rear delta theta
  theta_filter->add((dtheta_f + dtheta_r) / 2.0);
  double dtheta = theta_filter->getCurrentAverage();

  // some intermediate variables
  double th = theta + dtheta/2;
  double costh = cos(static_cast<float>(th));
  double sinth = sin(static_cast<float>(th));

  // save position and velocity in odom_out
  odom_out.pose.pose.position.x += delta_s * costh;
  odom_out.pose.pose.position.y += delta_s * sinth;
  odom_out.twist.twist.linear.x = vel * costh;
  odom_out.twist.twist.linear.y = vel * sinth;

  // integrate theta
  theta += dtheta;

  // save relative theta in odom_out
  tf::Quaternion q;
  q.setRPY(double(0), double(0), theta);
  tf::quaternionTFToMsg(q, odom_out.pose.pose.orientation);

  // create jacobians
  Eigen::Matrix<double, 3, 3> Jacobian_p;
  Jacobian_p.setIdentity();
  Jacobian_p(X, THETA) = - delta_s * sinth;
  Jacobian_p(Y, THETA) =   delta_s * costh;

  Eigen::Matrix<double, 3, 2> Jacobian_delta;
  Jacobian_delta(X, DELTA_R) = costh/2 - (delta_s * sinth)/(2 * track_width);
  Jacobian_delta(X, DELTA_L) = costh/2 + (delta_s * sinth)/(2 * track_width);
  Jacobian_delta(Y, DELTA_R) = sinth/2 + (delta_s * costh)/(2 * track_width);
  Jacobian_delta(Y, DELTA_L) = sinth/2 - (delta_s * costh)/(2 * track_width);
  Jacobian_delta(THETA, DELTA_R) =  1/track_width;
  Jacobian_delta(THETA, DELTA_L) = -1/track_width;

  Eigen::Matrix<double, 2, 2> Sigma_delta;
  Sigma_delta.setZero();

  if(use_static_cov)
  {
    Sigma_delta(DELTA_R, DELTA_R) = static_cov.at(DELTA_R);
    Sigma_delta(DELTA_L, DELTA_L) = static_cov.at(DELTA_L);
  }else{
    Sigma_delta(DELTA_R, DELTA_R) = ( msg->encoder[msg->FRONT_WHEEL_RIGHT].pos_abs_var +
                                      msg->encoder[msg->REAR_WHEEL_RIGHT].pos_abs_var )/2;
    Sigma_delta(DELTA_L, DELTA_L) = ( msg->encoder[msg->FRONT_WHEEL_LEFT].pos_abs_var +
                                      msg->encoder[msg->REAR_WHEEL_LEFT].pos_abs_var )/2;
  }


  Sigma_p = Jacobian_p     * Sigma_p     * Jacobian_p.transpose()       // covariances from previous state
          + Jacobian_delta * Sigma_delta * Jacobian_delta.transpose();  // covariances from new delta


  odom_out.pose.covariance[CovElem::lin_ang::linX_linX] = Sigma_p(X    , X    );
  odom_out.pose.covariance[CovElem::lin_ang::linX_linY] = Sigma_p(X    , Y    );
  odom_out.pose.covariance[CovElem::lin_ang::linX_angZ] = Sigma_p(X    , THETA);
  odom_out.pose.covariance[CovElem::lin_ang::linY_linX] = Sigma_p(Y    , X    );
  odom_out.pose.covariance[CovElem::lin_ang::linY_linY] = Sigma_p(Y    , Y    );
  odom_out.pose.covariance[CovElem::lin_ang::linY_angZ] = Sigma_p(Y    , THETA);
  odom_out.pose.covariance[CovElem::lin_ang::angZ_linX] = Sigma_p(THETA, X    );
  odom_out.pose.covariance[CovElem::lin_ang::angZ_linY] = Sigma_p(THETA, Y    );
  odom_out.pose.covariance[CovElem::lin_ang::angZ_angZ] = Sigma_p(THETA, THETA);


  // todo: could this be better handled?
  odom_out.twist.covariance[CovElem::lin_ang::linX_linX] = vel_var * costh;
  odom_out.twist.covariance[CovElem::lin_ang::linY_linY] = vel_var * sinth;

  // send out odom
  odom_pub.publish(odom_out);

  // broadcast tf message
  if(broadcast_tf)
  {
    geometry_msgs::TransformStamped tf_msg;
    tf::Transform trafo;
    tf_msg.header.stamp            = out_time;
    tf_msg.child_frame_id          = msg->header.frame_id;
    tf_msg.header.frame_id         = odom_out.header.frame_id;
    tf_msg.transform.translation.x = odom_out.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_out.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_out.pose.pose.position.z;
    tf_msg.transform.rotation.x    = odom_out.pose.pose.orientation.x;
    tf_msg.transform.rotation.y    = odom_out.pose.pose.orientation.y;
    tf_msg.transform.rotation.z    = odom_out.pose.pose.orientation.z;
    tf_msg.transform.rotation.w    = odom_out.pose.pose.orientation.w;
    br->sendTransform(tf_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localize_wheel_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get parameters
  track_width = pnh.param<double>("track_width", 0.22);
  ROS_INFO_STREAM("Loaded track_width: " << track_width);

  err_rl = pnh.param<double>("err_rl", 1);
  ROS_INFO_STREAM("Loaded err_rl: " << err_rl);

  static_frame_id = pnh.param<std::string>("static_frame_id", "odom");
  ROS_INFO_STREAM("Loaded static_frame: " << static_frame_id);

  broadcast_tf = pnh.param<bool>("broadcast_tf", true);
  ROS_INFO_STREAM("Loaded broadcast_tf: " << broadcast_tf);

  int theta_filter_length = pnh.param<int>("theta_filter_length", 10);
  ROS_INFO_STREAM("Loaded theta_filter_length: " << theta_filter_length);

  // initial covariances
  pnh.getParam("initial_cov", initial_cov);

  // static covariances
  use_static_cov = pnh.param<bool>("use_static_cov", true);
  ROS_INFO_STREAM("Loaded use_static_cov: " << use_static_cov);
  if(use_static_cov)
  {
    pnh.getParam("static_cov", static_cov);
    ROS_INFO_STREAM("Loaded static_cov: [" << static_cov.at(DELTA_R) << ", "
                                           << static_cov.at(DELTA_L) << "]");
  }

  // initialize filter
  theta_filter = new MovingAverage(theta_filter_length);

  // reset everything
  reset();

  // setup subscriber and publisher
  if(broadcast_tf)
  {
    br = new tf::TransformBroadcaster;
  }
  reset_svr = pnh.advertiseService("reset_odom", svrResetOdom);
  odom_pub = pnh.advertise<nav_msgs::Odometry>("odom_out", 100);
  ros::Subscriber sub = pnh.subscribe("enc_in", 100, encoderCallback);

  ros::spin();

  // clean up
  delete br;
  delete theta_filter;

  return 0;
}
