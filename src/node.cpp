#include "ros/ros.h"
#include "drive_ros_msgs/VehicleEncoder.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "eigen3/Eigen/Dense"
#include "std_srvs/Trigger.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "drive_ros_localize_wheel_odometry/cov_elements.h"
#include "drive_ros_localize_wheel_odometry/moving_average.h"
#include "drive_ros_localize_wheel_odometry/save_odom_in_csv.h"

// matrix indices
static constexpr size_t X     = 0;
static constexpr size_t Y     = 1;
static constexpr size_t THETA = 2;

static constexpr size_t DELTA_R = 0;
static constexpr size_t DELTA_L = 1;


const int encoder_ct = 4;                // number of encoder (assume we have 4 wheel encoder)
double theta = 0;                        // heading
double b_actual;                         // actual distance between wheels on an axis (effective track width)
double err_d;                            // error factor between left and right
double err_s;                            // scaling error of wheels
bool use_front;                          // whether to use front or rear axis encoder
bool broadcast_tf;                       // whether to broadcast tf
bool use_sensor_time = true;             // use sensor time or not
std::string static_frame_id;             // static frame id
std::string axis_frame;                  // axis being used
std::string input_topic;                 // input topic name
std::ofstream file_out_log;              // file output handle
bool use_bag;                            // debug mode

ros::ServiceServer reset_svr;            // reset service
ros::Publisher odom_pub;                 // publisher
nav_msgs::Odometry odom_out;             // published message
tf::TransformBroadcaster* br;            // broadcast transform

Eigen::Matrix<double, 3, 3> Sigma_p;     // covariance matrix of previous step
MovingAverage* theta_filter;             // moving average filter for theta

std::vector<float> initial_cov;          // initial covariances


// reset
void reset()
{
  ROS_INFO_STREAM("Reset Wheel Odometry.");

  // new odometry should be zero
  odom_out = nav_msgs::Odometry();

  theta = 0;
  theta_filter->clear();

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

  // choose front or rear
  uint8_t wheel_left = (use_front) ? msg->FRONT_WHEEL_LEFT : msg->REAR_WHEEL_LEFT;
  uint8_t wheel_right = (use_front) ? msg->FRONT_WHEEL_RIGHT : msg->REAR_WHEEL_RIGHT;


  // correct messages from error
  drive_ros_msgs::EncoderLinear left;
  double left_corr = err_s * 2/(err_d + 1);
  left.pos_rel      = msg->encoder[wheel_left].pos_rel      *     left_corr;
  left.pos_rel_var  = msg->encoder[wheel_left].pos_rel_var  * pow(left_corr, 2);
  left.pos_abs      = msg->encoder[wheel_left].pos_abs      *     left_corr;
  left.pos_abs_var  = msg->encoder[wheel_left].pos_abs_var  * pow(left_corr, 2);
  left.vel          = msg->encoder[wheel_left].vel          *     left_corr;
  left.vel_var      = msg->encoder[wheel_left].vel_var      * pow(left_corr, 2);

  drive_ros_msgs::EncoderLinear right;
  double right_corr = err_s * 2/(1/err_d + 1);
  right.pos_rel     = msg->encoder[wheel_right].pos_rel     *     right_corr;    ;
  right.pos_rel_var = msg->encoder[wheel_right].pos_rel_var * pow(right_corr, 2);;
  right.pos_abs     = msg->encoder[wheel_right].pos_abs     *     right_corr;    ;
  right.pos_abs_var = msg->encoder[wheel_right].pos_abs_var * pow(right_corr, 2);;
  right.vel         = msg->encoder[wheel_right].vel         *     right_corr;    ;
  right.vel_var     = msg->encoder[wheel_right].vel_var     * pow(right_corr, 2);;

  // set header
  odom_out.header.frame_id = static_frame_id;
  odom_out.header.stamp = (use_sensor_time) ? msg->header.stamp : ros::Time::now();
  odom_out.child_frame_id = axis_frame;

  // calculate mean
  double delta_s = (right.pos_rel + left.pos_rel) / 2.0;
  double vel     = (right.vel     + left.vel    ) / 2.0;
  double dtheta  = (right.pos_rel - left.pos_rel) / b_actual; // track width error already included
  dtheta = theta_filter->addAndGetCrrtAvg(dtheta); // filter theta value

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
  Jacobian_delta(X, DELTA_R) = costh/2 - (delta_s * sinth)/(2 * b_actual);
  Jacobian_delta(X, DELTA_L) = costh/2 + (delta_s * sinth)/(2 * b_actual);
  Jacobian_delta(Y, DELTA_R) = sinth/2 + (delta_s * costh)/(2 * b_actual);
  Jacobian_delta(Y, DELTA_L) = sinth/2 - (delta_s * costh)/(2 * b_actual);
  Jacobian_delta(THETA, DELTA_R) =  1/b_actual;
  Jacobian_delta(THETA, DELTA_L) = -1/b_actual;

  Eigen::Matrix<double, 2, 2> Sigma_delta;
  Sigma_delta.setZero();


  Sigma_delta(DELTA_R, DELTA_R) = right.pos_rel_var;
  Sigma_delta(DELTA_L, DELTA_L) = left.pos_rel_var;


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


  // TODO: could this be better handled?
  odom_out.twist.covariance[CovElem::lin_ang::linX_linX] = 0;
  odom_out.twist.covariance[CovElem::lin_ang::linY_linY] = 0;

  // send out odom
  odom_pub.publish(odom_out);

  // broadcast tf message
  if(broadcast_tf)
  {
    // tf to static frame
    geometry_msgs::TransformStamped tf_msg;
    tf::Transform trafo;
    tf_msg.header.stamp            = (use_sensor_time) ? msg->header.stamp : ros::Time::now();
    tf_msg.child_frame_id          = axis_frame;
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

  // save message to debug file
  if(use_bag){
    SaveOdomInCSV::writeMsg(odom_out, file_out_log);
  }
}

// read data from bag and feed it into the callback function
bool readFromBag(std::string bag_file_path)
{
  // open bag
  rosbag::Bag bag;
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // topics to load
  std::vector<std::string> bag_topics;
  bag_topics.push_back(input_topic);

  // create bag view
  rosbag::View bag_view(bag, rosbag::TopicQuery(bag_topics));

  // loop over all messages
  BOOST_FOREACH(rosbag::MessageInstance const m, bag_view)
  {
    if(m.getTopic() == input_topic || ("/" + m.getTopic() == input_topic))
    {
      drive_ros_msgs::VehicleEncoder::ConstPtr enc = m.instantiate<drive_ros_msgs::VehicleEncoder>();
      if (enc != NULL){
        encoderCallback(enc);
      }
    }
  }

  // close bag
  bag.close();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localize_wheel_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get parameters
  b_actual = pnh.param<double>("b_actual", 0.22);
  ROS_INFO_STREAM("Loaded b_actual: " << b_actual);

  err_d = pnh.param<double>("err_d", 1);
  ROS_INFO_STREAM("Loaded err_d: " << err_d);

  err_s = pnh.param<double>("err_s", 1);
  ROS_INFO_STREAM("Loaded err_s: " << err_s);

  use_front = pnh.param<bool>("use_front", true);
  ROS_INFO_STREAM("Loaded use_front: " << use_front);

  static_frame_id = pnh.param<std::string>("static_frame_id", "");
  ROS_INFO_STREAM("Loaded static_frame: " << static_frame_id);

  input_topic = pnh.param<std::string>("input_topic", "");
  ROS_INFO_STREAM("Loaded input_topic: " << input_topic);

  broadcast_tf = pnh.param<bool>("broadcast_tf", true);
  ROS_INFO_STREAM("Loaded broadcast_tf: " << broadcast_tf);

  int theta_filter_length = pnh.param<int>("theta_filter_length", 10);
  ROS_INFO_STREAM("Loaded theta_filter_length: " << theta_filter_length);

  axis_frame = pnh.param<std::string>("axis_frame", "");
  ROS_INFO_STREAM("Loaded axis_frame: " << axis_frame);

  use_bag = pnh.param<bool>("use_bag", false);
  ROS_INFO_STREAM("Loaded use_bag: " << use_bag);

  std::string bag_file = pnh.param<std::string>("bag_file", "");
  ROS_INFO_STREAM("Loaded bag_file: " << bag_file);

  std::string csv_out = pnh.param<std::string>("csv_out", "");
  ROS_INFO_STREAM("Loaded csv_out: " << csv_out);

  pnh.getParam("initial_cov", initial_cov);

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

  // check if read from bag
  if(use_bag){

    SaveOdomInCSV::writeHeader(csv_out, file_out_log);

    // read data directly from a bag
    readFromBag(bag_file);
    ROS_INFO_STREAM("Finished Reading from Bag");


  }else{

    ros::Subscriber sub = pnh.subscribe(input_topic, 100, encoderCallback);
    // spin node normally
    while(ros::ok()){
      ros::spin();
    }
  }

  ROS_INFO_STREAM("Exit Node");
  pnh.shutdown();
  nh.shutdown();
  return 0;
}
