#ifndef RATSLAM_ROS_LOCAL_VIEW_ROS_H
#define RATSLAM_ROS_LOCAL_VIEW_ROS_H

// ratslam
#include "ratslam/local_view_match.h"
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
// ROS msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ratslam_ros/ViewTemplate.h>
// irrlicht engine
#ifdef HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
#endif


namespace ratslam_ros {

class LocalViewROS
{
public:

  LocalViewROS(const std::string& config_file_path="");

  ~LocalViewROS();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, bool override_graphics=false);

  void image_callback(const sensor_msgs::ImageConstPtr& image);
  void save_lvm_timer_callback(const ros::TimerEvent& event);

private:
  // fcns
  bool load_lvm(const std::string& file_path);

  // vars
  std::shared_ptr<ratslam::LocalViewMatch> lv_;

  // params
  std::string config_file_path_;
  double lvm_save_period_;
  std::string lvm_file_path_;

  // pubs
  ros::Publisher pub_vt_;

  // subs
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber it_sub_;

  // timers
  ros::Timer lvm_save_timer_;

#if HAVE_IRRLICHT
  std::shared_ptr<ratslam::LocalViewScene> lvs_;
  bool use_graphics_;
#endif
};

}

#endif //RATSLAM_ROS_LOCAL_VIEW_ROS_H
