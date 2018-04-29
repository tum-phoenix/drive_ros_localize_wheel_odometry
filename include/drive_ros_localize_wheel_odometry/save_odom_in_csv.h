#ifndef SAVE_ODOM_IN_CSV_H
#define SAVE_ODOM_IN_CSV_H

#include <fstream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

namespace SaveOdomInCSV
{

// write header file of output log file
void writeHeader(const std::string& filename, std::ofstream& file_out_log)
{
  file_out_log.open( filename );

  file_out_log << "timestamp,";

  file_out_log << "pose_posX,"
               << "pose_posY,"
               << "pose_posZ,"
               << "pose_oriX,"
               << "pose_oriY,"
               << "pose_oriZ,";

  for(int i=0; i<36; i++)
    file_out_log << "pose_cov_[" << i << "],";

  file_out_log << "twist_linX,"
               << "twist_linY,"
               << "twist_linZ,"
               << "twist_angX,"
               << "twist_angY,"
               << "twist_angZ,";

  for(int i=0; i<36; i++)
    file_out_log << "twist_cov_[" << i << "],";

   file_out_log << std::endl;
}

// write the odometry message to output log file
void writeMsg(const nav_msgs::Odometry& msg, std::ofstream& file_out_log)
{
  file_out_log << msg.header.stamp.toSec() << ",";

  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  file_out_log << msg.pose.pose.position.x << ","
               << msg.pose.pose.position.y << ","
               << msg.pose.pose.position.z << ","
               << roll  << ","
               << pitch << ","
               << yaw   << ",";

  for(int i=0; i<36; i++)
    file_out_log << msg.pose.covariance.at(i) << ",";

  file_out_log << msg.twist.twist.linear.x << ","
               << msg.twist.twist.linear.y << ","
               << msg.twist.twist.linear.z << ","
               << msg.twist.twist.angular.x << ","
               << msg.twist.twist.angular.y << ","
               << msg.twist.twist.angular.z << ",";

  for(int i=0; i<36; i++)
    file_out_log << msg.twist.covariance.at(i) << ",";

  file_out_log << std::endl;
}


};

#endif
