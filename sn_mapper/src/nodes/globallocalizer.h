#ifndef GLOBALLOCALIZER_H
#define GLOBALLOCALIZER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sn_mapper/conversions.h>
#include <sn_mapper/localizer.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

typedef nav_msgs::Path Path;

namespace sn{

class GlobalLocalizer
{
public:
  GlobalLocalizer(const ros::NodeHandle& node);

  void scanCallback(LaserPtr scan);

  void mapCallback(OGridPtr map);

  void initialpose(PoseCovStampPtr initial_pose);

  ros::NodeHandle node_;
  ros::Subscriber scanSub_;
  ros::Subscriber mapSub_;
  ros::Subscriber poseSub_;
  ros::Publisher laserPub_;
  ros::Publisher cloudPub_;
  ros::Publisher scorePub_;
  tf::StampedTransform odomTrans_;
  tf::TransformListener listener_;

  Path path_;

  sn::Localizer localizer_;
};

}

#endif // GLOBALLOCALIZER_H
