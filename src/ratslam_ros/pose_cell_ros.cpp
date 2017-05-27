// ratslam
#include "ratslam_ros/pose_cell_ros.h"
// boost
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ratslam_ros {

PoseCellROS::PoseCellROS(const std::string &config_file_path)
  : config_file_path_(config_file_path)
  , pcn_file_path_("ratslam-latest.bpcn")
  , pcn_save_period_(10.0)
#ifdef HAVE_IRRLICHT
  , use_graphics_(false)
#endif
{
}

PoseCellROS::~PoseCellROS() {

}

void PoseCellROS::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, bool override_graphics)
{
  ROS_INFO_STREAM("LocalViewROS - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
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
  boost::property_tree::ptree settings, ratslam_settings, general_settings;
  read_ini(config_file_path_, settings);
  ratslam::get_setting_child(ratslam_settings, settings, "ratslam", true);
  ratslam::get_setting_child(general_settings, settings, "general", true);
  // backward compatibility, namespace or private nodehandle should do it more ROS like
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  // params
  priv_nh.param("pcn_save_period", pcn_save_period_, pcn_save_period_);
  priv_nh.param("pcn_file_path", pcn_file_path_, pcn_file_path_);

  // check backward compatibility with configs that have topic_root set
  std::string odom_topic = "odom";
  std::string action_topic = "PoseCell/TopologicalAction";
  std::string template_topic = "LocalView/Template";
  if (!topic_root.empty())
  {
    odom_topic = topic_root + "/odom";
    action_topic = topic_root + "/PoseCell/TopologicalAction";
    template_topic = topic_root + "/LocalView/Template";
  }

  // create PoseCellNetwork object
  pc_ = std::make_shared<ratslam::PosecellNetwork>(ratslam_settings);

  ROS_INFO("Created pose cell network.");

  // try to load pcn
  if (!load_pcn(pcn_file_path_))
  {
    ROS_WARN_STREAM("Could not load PoseCellNetwork from file " << pcn_file_path_);
  }

  // pubs
  pub_pc_ = nh.advertise<ratslam_ros::TopologicalAction>(action_topic, 0);

  // subs
  sub_odometry_ = nh.subscribe<nav_msgs::Odometry>(odom_topic, 0, &PoseCellROS::odo_callback, this);
  sub_template_ = nh.subscribe<ratslam_ros::ViewTemplate>(template_topic, 0, &PoseCellROS::template_callback, this);

  // timers
  pcn_save_timer_ = nh.createTimer(ros::Duration(pcn_save_period_), &PoseCellROS::save_pcn_timer_callback, this);

#ifdef HAVE_IRRLICHT
  if (override_graphics)
  {
    use_graphics_ = false;
    return;
  }
  boost::property_tree::ptree draw_settings;
  ratslam::get_setting_child(draw_settings, settings, "draw", true);
  ratslam::get_setting_from_ptree(use_graphics_, draw_settings, "enable", true);
  if (use_graphics_)
  {
    pcs_ = std::make_shared<ratslam::PosecellScene>(draw_settings, pc_.get());
  }
#endif
}

void PoseCellROS::odo_callback(const nav_msgs::OdometryConstPtr& odo)
{
  ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now().toSec() << "} seq=" << odo->header.seq
                                      << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();

    pc_output_.src_id = pc_->get_current_exp_id();
    pc_->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
    pc_output_.action = pc_->get_action();
    if (pc_output_.action != ratslam::PosecellNetwork::NO_ACTION)
    {
      pc_output_.header.stamp = ros::Time::now();
      pc_output_.header.seq++;
      pc_output_.dest_id = pc_->get_current_exp_id();
      pc_output_.relative_rad = pc_->get_relative_rad();
      pub_pc_.publish(pc_output_);
      ROS_DEBUG_STREAM("PC:action_publish{odo}{" << ros::Time::now().toSec() << "} action{" << pc_output_.header.seq
                                                 << "}=" << pc_output_.action << " src=" << pc_output_.src_id
                                                 << " dest=" << pc_output_.dest_id);
    }

#ifdef HAVE_IRRLICHT
    if (use_graphics_)
    {
      pcs_->update_scene();
      pcs_->draw_all();
    }
#endif
  }
  prev_time = odo->header.stamp;
}

void PoseCellROS::template_callback(const ratslam_ros::ViewTemplateConstPtr& vt)
{
  ROS_DEBUG_STREAM("PC:vt_callback{" << ros::Time::now().toSec() << "} seq=" << vt->header.seq
                                     << " id=" << vt->current_id << " rad=" << vt->relative_rad);

  pc_->on_view_template(vt->current_id, vt->relative_rad);

#ifdef HAVE_IRRLICHT
  if (use_graphics_)
  {
    pcs_->update_scene();
    pcs_->draw_all();
  }
#endif
}

void PoseCellROS::save_pcn_timer_callback(const ros::TimerEvent& event)
{
  // create and open a character archive for output
  std::ofstream ofs(pcn_file_path_.c_str());

  // save map to archive file
  {
    boost::archive::binary_oarchive binary_archive(ofs);
    // write class instance to archive
    pc_->save(binary_archive, 1);
    // archive and stream closed when destructors are called
  }
}

bool PoseCellROS::load_pcn(const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    pc_->load(binary_archive, 1);
    // archive and stream closed when destructors are called
    return true;
  }
  catch (...)
  {
    return false;
  }
}

}