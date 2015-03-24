#ifndef GLOBALMAPPER_H
#define GLOBALMAPPER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <sn_mapper/map.h>
#include <sn_mapper/conversions.h>

namespace sn {

class GlobalMapper
{
public:
  GlobalMapper(const ros::NodeHandle& node);

  void scanCallback(LaserPtr scan);

  void scoreCallback(std_msgs::Float64::ConstPtr score);

  void mapCallback(OGridPtr map);

  void scoreScan(const sn::vector_pts_t& points, double& mean, double &std);

  void updateMap(const sn::vector_pts_t& point, sn::pose_t& robot, int hit_s, int miss_s);

protected:
  bool mNewMap;
  bool mScoreDisp;


private:

  // ROS
  ros::NodeHandle node_;
  ros::Subscriber mapSub_;
  ros::Subscriber scanSub_;
  ros::Subscriber scoreSub_;
  tf::TransformListener listener_;
  ros::Publisher mapPub_;

  // STUFF
  sn::Map mMap;
  bool mMapReceived;
  bool mGoodScore;
  std::vector<sn::vector_pts_t> mLasers;

};

}

#endif // GLOBALMAPPER_H
