// ratslam
#include "ratslam_ros/local_view_ros.h"
// boost
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ratslam_ros
{

LocalViewROS::LocalViewROS(const std::string& config_file_path)
  : config_file_path_(config_file_path)
  , lvm_save_period_(10.0)
  , lvm_file_path_("ratslam-latest.blvm")
#ifdef HAVE_IRRLICHT
  , use_graphics_(false)
#endif
{
}

LocalViewROS::~LocalViewROS()
{

}

void LocalViewROS::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, bool override_graphics)
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
  ratslam::get_setting_child(general_settings, settings, "general", true);
  ratslam::get_setting_child(ratslam_settings, settings, "ratslam", true);
  // backward compatibility, namespace or private nodehandle should do it more ROS like
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  // params
  priv_nh.param("lvm_save_period", lvm_save_period_, lvm_save_period_);
  priv_nh.param("lvm_file_path", lvm_file_path_, lvm_file_path_);

  // check backward compatibility with configs that have topic_root set
  std::string image_topic = "image";
  std::string template_topic = "LocalView/Template";
  if (!topic_root.empty())
  {
    // prepend with topic with topic_root setting
    image_topic = topic_root + "/camera/image";
    template_topic = topic_root + "/LocalView/Template";
  }

  lv_ = std::make_shared<ratslam::LocalViewMatch>(ratslam_settings);

  // try to load lvm
  if (!load_lvm(lvm_file_path_))
  {
    ROS_WARN_STREAM("Could not load LocalViewMatch from file " << lvm_file_path_);
  }

  // pubs
  pub_vt_ = nh.advertise<ratslam_ros::ViewTemplate>(template_topic, 0);

  // image transport
  it_ = std::make_shared<image_transport::ImageTransport>(nh);
  it_sub_ = it_->subscribe(image_topic, 0, &LocalViewROS::image_callback, this);

  // timers
  lvm_save_timer_ = nh.createTimer(ros::Duration(lvm_save_period_), &LocalViewROS::save_lvm_timer_callback, this);

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
    lvs_ = std::make_shared<ratslam::LocalViewScene>(draw_settings, lv_.get());
  }
#endif

}

void LocalViewROS::image_callback(const sensor_msgs::ImageConstPtr& image)
{
  ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now().toSec() << "} seq=" << image->header.seq);

  static ratslam_ros::ViewTemplate vt_output;

  lv_->on_image(&image->data[0], !(image->encoding == "bgr8"), image->width, image->height);

  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = lv_->get_current_vt();
  vt_output.relative_rad = lv_->get_relative_rad();

  pub_vt_.publish(vt_output);

#ifdef HAVE_IRRLICHT
  if (use_graphics_)
  {
    lvs_->draw_all();
  }
#endif
}

void LocalViewROS::save_lvm_timer_callback(const ros::TimerEvent& event)
{
  // create and open a character archive for output
  std::ofstream ofs(lvm_file_path_.c_str());

  // save map to archive file
  {
    boost::archive::binary_oarchive binary_archive(ofs);
    // write class instance to archive
    binary_archive << *lv_;
    // archive and stream closed when destructors are called
  }
}

bool LocalViewROS::load_lvm(const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    binary_archive >> *lv_;
    // archive and stream closed when destructors are called
    return true;
  }
  catch (...)
  {
    return false;
  }
  
}

}