// ratslam
#include "ratslam_ros/visual_odometry_ros.h"
#include "utils/utils.h"

namespace ratslam_ros {

VisualOdometryROS::VisualOdometryROS(const std::string &config_file_path)
 : config_file_path_(config_file_path)
{
}

VisualOdometryROS::~VisualOdometryROS()
{

}

void VisualOdometryROS::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh)
{
  // print info
  ROS_INFO_STREAM("VisualOdometryROS - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  // config
  if (config_file_path_.empty())
  {
    priv_nh.param("config_file_path", config_file_path_, config_file_path_);
  }
  if (config_file_path_.empty())
  {
    ROS_FATAL_STREAM("No config file given. Aborting.");
    exit(-1);
  }

  // settings
  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, vo_settings;
  read_ini(config_file_path_, settings);
  ratslam::get_setting_child(vo_settings, settings, "visual_odometry", true);
  ratslam::get_setting_child(general_settings, settings, "general", true);
  // backward compatibility, namespace or private nodehandle should do it more ROS like
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  vo_ = std::make_shared<ratslam::VisualOdometry>(vo_settings);

  std::string odom_topic = "odom";
  std::string image_topic = "image";
  // check backward compatibility with configs that have topic_root set
  if (!topic_root.empty())
  {
    image_topic = topic_root + "/camera/image";
    odom_topic = topic_root + "/odom";
  }

  // pubs
  pub_vo_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 0);

  // image transport
  it_ = std::make_shared<image_transport::ImageTransport>(nh);
  it_sub_ = it_->subscribe(image_topic, 0, &VisualOdometryROS::image_callback, this);
}

void VisualOdometryROS::image_callback(const sensor_msgs::ImageConstPtr& image)
{
  ROS_DEBUG_STREAM("VO:image_callback{" << ros::Time::now().toSec() << "} seq=" << image->header.seq);

  static nav_msgs::Odometry odom_output;

  vo_->on_image(&image->data[0], image->encoding != "bgr8", image->width, image->height,
                &odom_output.twist.twist.linear.x, &odom_output.twist.twist.angular.z);

  odom_output.header.stamp = image->header.stamp;
  odom_output.header.seq++;

  pub_vo_.publish(odom_output);
}

}