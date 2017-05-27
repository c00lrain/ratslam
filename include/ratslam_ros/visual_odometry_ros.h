#ifndef RATSLAM_ROS_VISUAL_ODOMETRY_ROS_H
#define RATSLAM_ROS_VISUAL_ODOMETRY_ROS_H

// ratslam
#include "ratslam/visual_odometry.h"
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
// ROS msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

namespace ratslam_ros
{

class VisualOdometryROS
{

public:

  VisualOdometryROS(const std::string &config_file_path = "");

  ~VisualOdometryROS();

  void initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);

  void image_callback(const sensor_msgs::ImageConstPtr& image);

private:
  // vars
  std::shared_ptr<ratslam::VisualOdometry> vo_;

  // params
  std::string config_file_path_;

  // pubs
  ros::Publisher pub_vo_;

  // subs
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber it_sub_;

};

}
#endif //RATSLAM_ROS_VISUAL_ODOMETRY_ROS_H
