#ifndef RATSLAM_ROS_EXPERIENCE_MAP_ROS_H
#define RATSLAM_ROS_EXPERIENCE_MAP_ROS_H

// ratslam
#include "ratslam/experience_map.h"
// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ratslam_ros/TopologicalAction.h>
#include <ratslam_ros/TopologicalMap.h>
#include <visualization_msgs/MarkerArray.h>
// irrlicht engine
#ifdef HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
#endif

namespace ratslam_ros
{

class ExperienceMapROS
{
public:

  ExperienceMapROS(const std::string& config_file_path="");

  ~ExperienceMapROS();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, bool override_graphics=false);

  void odo_callback(const nav_msgs::OdometryConstPtr& odom);
  void action_callback(const ratslam_ros::TopologicalActionConstPtr& action);
  void set_goal_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose);
  void tf_update_timer_callback(const ros::TimerEvent& event);
  void save_map_timer_callback(const ros::TimerEvent& event);

private:
  // fcns
  bool load_map(const std::string& file_path);

  // vars
  std::shared_ptr<ratslam::ExperienceMap> em_;
  double odom_pose_[3];
  double mpose_[3];
  geometry_msgs::PoseStamped pose_output_;
  ratslam_ros::TopologicalMap em_map_;
  visualization_msgs::Marker em_marker_;

  // params
  std::string config_file_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_update_rate_;
  double map_save_period_;
  std::string map_file_path_; // = std::string();

  // pubs
  ros::Publisher pub_em_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_em_markers_;
  ros::Publisher pub_goal_path_;

  // subs
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_action_;
  ros::Subscriber sub_goal_;

  // timers
  ros::Timer tf_update_timer_;
  ros::Timer save_map_timer_;

  // tf
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform map_to_odom_;

#ifdef HAVE_IRRLICHT
  // graphics
  std::shared_ptr<ratslam::ExperienceMapScene> ems_;
  bool use_graphics_;
#endif
};

}

#endif //RATSLAM_ROS_EXPERIENCE_MAP_ROS_H
