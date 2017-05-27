// ratslam
#include "ratslam_ros/experience_map_ros.h"
// boost
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ratslam_ros
{

ExperienceMapROS::ExperienceMapROS(const std::string& config_file_path)
  : odom_pose_{0.0, 0.0, 0.0}
  , mpose_{0.0, 0.0, 0.0}
  , config_file_path_(config_file_path)
  , map_frame_("map")
  , odom_frame_("odom")
  , base_frame_("base_link")
  , tf_update_rate_(20.0)
  , map_save_period_(10.0)
  , map_file_path_("ratslam-latest.bmap")
#ifdef HAVE_IRRLICHT
  , use_graphics_(false)
#endif
{
}

ExperienceMapROS::~ExperienceMapROS()
{

}

void ExperienceMapROS::initialize(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, bool override_graphics)
{
  // print info
  ROS_INFO_STREAM("ExperienceMapROS - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  // params
  if (config_file_path_.empty())
  {
    priv_nh.param("config_file_path", config_file_path_, config_file_path_);
  }
  if (config_file_path_.empty())
  {
    ROS_FATAL_STREAM("No config file given. Aborting.");
    exit(-1);
  }

  // settings file
  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(config_file_path_, settings);

  ratslam::get_setting_child(ratslam_settings, settings, "ratslam", true);
  ratslam::get_setting_child(general_settings, settings, "general", true);
  // backward compatibility, namespace or private nodehandle should do it more ROS like
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  // params
  priv_nh.param("map_frame", map_frame_, map_frame_);
  priv_nh.param("odom_frame", odom_frame_, odom_frame_);
  priv_nh.param("base_frame", base_frame_, base_frame_);
  priv_nh.param("tf_update_rate", tf_update_rate_, tf_update_rate_);
  priv_nh.param("map_save_period", map_save_period_, map_save_period_);
  priv_nh.param("map_file_path", map_file_path_, map_file_path_);

  // check backward compatibility with configs that have topic_root set
  std::string odom_topic = "odom";
  std::string em_topic = "ExperienceMap/Map";
  std::string em_markers_topic = "ExperienceMap/MapMarker";
  std::string robot_pose_topic = "ExperienceMap/RobotPose";
  std::string path_topic = "ExperienceMap/PathToGoal";
  std::string goal_pose_topic = "ExperienceMap/SetGoalPose";
  std::string action_topic = "PoseCell/TopologicalAction";
  if (!topic_root.empty())
  {
    odom_topic = topic_root + "/odom";
    em_topic = topic_root + "/ExperienceMap/Map";
    em_markers_topic = topic_root + "/ExperienceMap/MapMarker";
    robot_pose_topic = topic_root + "/ExperienceMap/RobotPose";
    path_topic = topic_root + "/ExperienceMap/PathToGoal";
    goal_pose_topic = topic_root + "/ExperienceMap/SetGoalPose";
    action_topic = topic_root + "/PoseCell/TopologicalAction";
  }

  // create the experience map object
  em_ = std::make_shared<ratslam::ExperienceMap>(ratslam_settings);

  // try to load map
  if (!load_map(map_file_path_))
  {
    ROS_WARN_STREAM("Could not load map from file " << map_file_path_);
  }

  // pubs
  pub_em_ = nh.advertise<ratslam_ros::TopologicalMap>(em_topic, 1);
  pub_em_markers_ = nh.advertise<visualization_msgs::Marker>(em_markers_topic, 1);
  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>(robot_pose_topic, 1);
  pub_goal_path_ = nh.advertise<nav_msgs::Path>(path_topic, 1);

  // subs
  sub_odometry_ = nh.subscribe<nav_msgs::Odometry>(odom_topic, 0, &ExperienceMapROS::odo_callback, this);
  sub_action_ = nh.subscribe<ratslam_ros::TopologicalAction>(action_topic, 0, &ExperienceMapROS::action_callback,
                                                             this);
  sub_goal_ = nh.subscribe<geometry_msgs::PoseStamped>(goal_pose_topic, 0, &ExperienceMapROS::set_goal_pose_callback,
                                                       this);

  // timers
  tf_update_timer_ = priv_nh.createTimer(ros::Duration(1.0 / tf_update_rate_),
                                                   &ExperienceMapROS::tf_update_timer_callback, this);
  save_map_timer_ = priv_nh.createTimer(ros::Duration(map_save_period_),
                                                  &ExperienceMapROS::save_map_timer_callback, this);

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
    ems_ = std::make_shared<ratslam::ExperienceMapScene>(draw_settings, em_.get());
  }
#endif
}

void ExperienceMapROS::odo_callback(const nav_msgs::OdometryConstPtr& odo)
{
  ROS_DEBUG_STREAM("EM:odo_callback{" << ros::Time::now().toSec() << "} seq=" << odo->header.seq
                   << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();
    em_->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
  }

  static ros::Time prev_goal_update(0);

  if (em_->get_current_goal_id() >= 0)
  {
    // (prev_goal_update.toSec() == 0 || (odo->header.stamp - prev_goal_update).toSec() > 5)
    // em->calculate_path_to_goal(odo->header.stamp.toSec());

    prev_goal_update = odo->header.stamp;

    em_->calculate_path_to_goal(odo->header.stamp.toSec());

    static nav_msgs::Path path;
    if (em_->get_current_goal_id() >= 0)
    {
      em_->get_goal_waypoint();

      static geometry_msgs::PoseStamped pose;
      path.header.stamp = ros::Time::now();
      path.header.frame_id = map_frame_;

      pose.header.seq = 0;
      pose.header.frame_id = map_frame_;
      path.poses.clear();
      unsigned int trace_exp_id = em_->get_goals()[0];
      while (trace_exp_id != em_->get_goal_path_final_exp())
      {
        pose.pose.position.x = em_->get_experience(trace_exp_id)->x_m;
        pose.pose.position.y = em_->get_experience(trace_exp_id)->y_m;
        path.poses.push_back(pose);
        pose.header.seq++;

        trace_exp_id = em_->get_experience(trace_exp_id)->goal_to_current;
      }

      pub_goal_path_.publish(path);

      path.header.seq++;
    }
    else
    {
      path.header.stamp = ros::Time::now();
      path.header.frame_id = map_frame_;
      path.poses.clear();
      pub_goal_path_.publish(path);

      path.header.seq++;
    }
  }

  prev_time = odo->header.stamp;
}

void ExperienceMapROS::action_callback(const ratslam_ros::TopologicalActionConstPtr& action)
{
  ROS_DEBUG_STREAM("EM:action_callback{" << ros::Time::now().toSec() << "} action=" << action->action
                   << " src=" << action->src_id << " dst=" << action->dest_id);

  switch (action->action)
  {
    case ratslam_ros::TopologicalAction::CREATE_NODE:
      em_->on_create_experience(action->dest_id);
      em_->on_set_experience(action->dest_id, 0);
      break;

    case ratslam_ros::TopologicalAction::CREATE_EDGE:
      em_->on_create_link(action->src_id, action->dest_id, action->relative_rad);
      em_->on_set_experience(action->dest_id, action->relative_rad);
      break;

    case ratslam_ros::TopologicalAction::SET_NODE:
      em_->on_set_experience(action->dest_id, action->relative_rad);
      break;

    default:
      ROS_WARN("Received invalid action!");
      break;
  }

  em_->iterate();

  pose_output_.header.stamp = ros::Time::now();
  pose_output_.header.seq++;
  pose_output_.header.frame_id = map_frame_;
  pose_output_.pose.position.x = em_->get_experience(em_->get_current_id())->x_m;
  pose_output_.pose.position.y = em_->get_experience(em_->get_current_id())->y_m;
  pose_output_.pose.position.z = 0;
  pose_output_.pose.orientation.x = 0;
  pose_output_.pose.orientation.y = 0;
  pose_output_.pose.orientation.z = sin(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
  pose_output_.pose.orientation.w = cos(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
  pub_pose_.publish(pose_output_);

  mpose_[0] = em_->get_experience(em_->get_current_id())->x_m;
  mpose_[1] = em_->get_experience(em_->get_current_id())->y_m;
  mpose_[2] = em_->get_experience(em_->get_current_id())->th_rad;

  static ros::Time prev_pub_time(0);

  if (action->header.stamp - prev_pub_time > ros::Duration(30.0))
  {
    prev_pub_time = action->header.stamp;

    em_map_.header.stamp = ros::Time::now();
    em_map_.header.seq++;
    em_map_.node_count = em_->get_num_experiences();
    em_map_.node.resize(em_->get_num_experiences());
    for (int i = 0; i < em_->get_num_experiences(); i++)
    {
      em_map_.node[i].id = em_->get_experience(i)->id;
      em_map_.node[i].pose.position.x = em_->get_experience(i)->x_m;
      em_map_.node[i].pose.position.y = em_->get_experience(i)->y_m;
      em_map_.node[i].pose.orientation.x = 0;
      em_map_.node[i].pose.orientation.y = 0;
      em_map_.node[i].pose.orientation.z = sin(em_->get_experience(i)->th_rad / 2.0);
      em_map_.node[i].pose.orientation.w = cos(em_->get_experience(i)->th_rad / 2.0);
    }

    em_map_.edge_count = em_->get_num_links();
    em_map_.edge.resize(em_->get_num_links());
    for (int i = 0; i < em_->get_num_links(); i++)
    {
      em_map_.edge[i].source_id = em_->get_link(i)->exp_from_id;
      em_map_.edge[i].destination_id = em_->get_link(i)->exp_to_id;
      em_map_.edge[i].duration = ros::Duration(em_->get_link(i)->delta_time_s);
      em_map_.edge[i].transform.translation.x = em_->get_link(i)->d * cos(em_->get_link(i)->heading_rad);
      em_map_.edge[i].transform.translation.y = em_->get_link(i)->d * sin(em_->get_link(i)->heading_rad);
      em_map_.edge[i].transform.rotation.x = 0;
      em_map_.edge[i].transform.rotation.y = 0;
      em_map_.edge[i].transform.rotation.z = sin(em_->get_link(i)->facing_rad / 2.0);
      em_map_.edge[i].transform.rotation.w = cos(em_->get_link(i)->facing_rad / 2.0);
    }
    pub_em_.publish(em_map_);
  }

  em_marker_.header.stamp = ros::Time::now();
  em_marker_.header.seq++;
  em_marker_.header.frame_id = map_frame_;
  em_marker_.type = visualization_msgs::Marker::LINE_LIST;
  em_marker_.points.resize(em_->get_num_links() * 2);
  em_marker_.action = visualization_msgs::Marker::ADD;
  em_marker_.scale.x = 0.01;
  // em_marker.scale.y = 1;
  // em_marker.scale.z = 1;
  em_marker_.color.a = 1;
  em_marker_.ns = "em";
  em_marker_.id = 0;
  em_marker_.pose.orientation.x = 0;
  em_marker_.pose.orientation.y = 0;
  em_marker_.pose.orientation.z = 0;
  em_marker_.pose.orientation.w = 1;
  for (int i = 0; i < em_->get_num_links(); i++)
  {
    em_marker_.points[i * 2].x = em_->get_experience(em_->get_link(i)->exp_from_id)->x_m;
    em_marker_.points[i * 2].y = em_->get_experience(em_->get_link(i)->exp_from_id)->y_m;
    em_marker_.points[i * 2].z = 0;
    em_marker_.points[i * 2 + 1].x = em_->get_experience(em_->get_link(i)->exp_to_id)->x_m;
    em_marker_.points[i * 2 + 1].y = em_->get_experience(em_->get_link(i)->exp_to_id)->y_m;
    em_marker_.points[i * 2 + 1].z = 0;
  }

  pub_em_markers_.publish(em_marker_);

#ifdef HAVE_IRRLICHT
  if (use_graphics_)
  {
    ems_->update_scene();
    ems_->draw_all();
  }
#endif

}

void ExperienceMapROS::set_goal_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose)
{
  em_->add_goal(pose->pose.position.x, pose->pose.position.y);
}

void ExperienceMapROS::tf_update_timer_callback(const ros::TimerEvent& event)
{
  tf::StampedTransform odom_to_base_;
  try
  {
    tf_listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), odom_to_base_);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Failed to compute odom pose. Exception: %s", e.what());
    return;
  }
  tf::Transform odom_to_base = odom_to_base_;

  tf::Transform base_to_map = tf::Transform(
    tf::createQuaternionFromRPY(0, 0, mpose_[2]),
    tf::Vector3(mpose_[0], mpose_[1], 0.0)
  ).inverse();

  map_to_odom_ = (odom_to_base * base_to_map).inverse();

  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.5);
  tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
}

void ExperienceMapROS::save_map_timer_callback(const ros::TimerEvent& event)
{
  // create and open a character archive for output
  std::ofstream ofs(map_file_path_.c_str());

  // save map to archive file
  {
    boost::archive::binary_oarchive binary_archive(ofs);
    // write class instance to archive
    binary_archive << *em_;
    // archive and stream closed when destructors are called
  }
}

bool ExperienceMapROS::load_map(const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    binary_archive >> *em_;
    // archive and stream closed when destructors are called
    return true;
  }
  catch (...)
  {
    return false;
  }
}

}
