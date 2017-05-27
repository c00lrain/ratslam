// nodelets
#include <nodelet/nodelet.h>
// pluginlib
#include <pluginlib/class_list_macros.h>
// ratslam ros
#include "ratslam_ros/experience_map_ros.h"
#include "ratslam_ros/local_view_ros.h"
#include "ratslam_ros/pose_cell_ros.h"
#include "ratslam_ros/visual_odometry_ros.h"

namespace ratslam_ros
{

class ExperienceMapNodelet : public nodelet::Nodelet
{
public:
  /**
   * @brief Initialize this nodelet.
   */
  virtual void onInit() {
//    em_ros_.initialize(getNodeHandle(), getPrivateNodeHandle(), true);
    em_ros_.initialize(getMTNodeHandle(), getMTPrivateNodeHandle(), true);
  }

private:
  ExperienceMapROS em_ros_;
};

class LocalViewNodelet : public nodelet::Nodelet
{
public:
  /**
   * @brief Initialize this nodelet.
   */
  virtual void onInit() {
//    lv_ros_.initialize(getNodeHandle(), getPrivateNodeHandle(), true);
    lv_ros_.initialize(getMTNodeHandle(), getMTPrivateNodeHandle(), true);
  }

private:
  LocalViewROS lv_ros_;
};

class PoseCellNodelet : public nodelet::Nodelet
{
public:
  /**
   * @brief Initialize this nodelet.
   */
  virtual void onInit() {
//    pc_ros_.initialize(getNodeHandle(), getPrivateNodeHandle(), true);
    pc_ros_.initialize(getMTNodeHandle(), getMTPrivateNodeHandle(), true);
  }

private:
  PoseCellROS pc_ros_;
};

class VisualOdometryNodelet : public nodelet::Nodelet
{
public:
  /**
   * @brief Initialize this nodelet.
   */
  virtual void onInit() {
//    vo_ros_.initialize(getNodeHandle(), getPrivateNodeHandle());
    vo_ros_.initialize(getMTNodeHandle(), getMTPrivateNodeHandle());
  }

private:
  VisualOdometryROS vo_ros_;
};

}

PLUGINLIB_EXPORT_CLASS(ratslam_ros::ExperienceMapNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ratslam_ros::LocalViewNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ratslam_ros::PoseCellNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ratslam_ros::VisualOdometryNodelet, nodelet::Nodelet)
