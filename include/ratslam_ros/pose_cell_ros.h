#ifndef RATSLAM_ROS_POSE_CELL_ROS_H
#define RATSLAM_ROS_POSE_CELL_ROS_H

// ratslam
#include "ratslam/posecell_network.h"
// ROS
#include <ros/ros.h>
// ROS msgs
#include <nav_msgs/Odometry.h>
#include <ratslam_ros/TopologicalAction.h>
#include <ratslam_ros/ViewTemplate.h>
// irrlicht engine
#ifdef HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
#endif


namespace ratslam_ros
{

class PoseCellROS
{

public:

  PoseCellROS(const std::string &config_file_path = "");

  ~PoseCellROS();

  void initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, bool override_graphics=false);

  void odo_callback(const nav_msgs::OdometryConstPtr& odo);
  void template_callback(const ratslam_ros::ViewTemplateConstPtr& vt);
  void save_pcn_timer_callback(const ros::TimerEvent& event);
  bool load_pcn(const std::string& file_path);

private:
  // vars
  std::shared_ptr<ratslam::PosecellNetwork> pc_;
  ratslam_ros::TopologicalAction pc_output_;

  // params
  std::string config_file_path_;
  double pcn_save_period_;
  std::string pcn_file_path_;

  // pubs
  ros::Publisher pub_pc_;

  // subs
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_template_;

  // timers
  ros::Timer pcn_save_timer_;

#if HAVE_IRRLICHT
  std::shared_ptr<ratslam::PosecellScene> pcs_;
  bool use_graphics_;
#endif

};

}
#endif //RATSLAM_ROS_POSE_CELL_ROS_H
