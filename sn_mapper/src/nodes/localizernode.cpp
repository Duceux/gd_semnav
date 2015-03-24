#include "localizernode.h"

using namespace sn;

LocalizerNode::LocalizerNode(const ros::NodeHandle& node):
  node_(node)
{
  mapSub_ = node_.subscribe("/map", 1, &LocalizerNode::mapCallback, this);
  scanSub_ = node_.subscribe("/scan", 1, &LocalizerNode::scanCallback, this);

  laserPub_ = node_.advertise<Laser>("localized_scan", 1);
  scorePub_ = node_.advertise<std_msgs::Float64>("score", 1);
  cloudPub_ = node_.advertise<geometry_msgs::PoseArray>("particles", 1);
  pathPub_ = node_.advertise<nav_msgs::Path>("path", 1, true);
}

void LocalizerNode::scanCallback(LaserPtr scan){
  Laser s = *scan;

  static tf::Transform best_tranform;
  static tf::TransformBroadcaster br;

  static bool first_scan = true;
  if(first_scan){
    best_tranform.getRotation().normalize();
    br.sendTransform(tf::StampedTransform(best_tranform.inverse(),
                                          s.header.stamp,
                                          "/laser", "/map"));
    first_scan = false;
    laserPub_.publish(s);
    return;
  }

  vector_pts_t l;
  sn::rosToLaser(s, l);
  localizer_.process(l);
  geometry_msgs::PoseArray pose_array;
  for(auto it=localizer_.getParticles().begin(); it!=localizer_.getParticles().end(); ++it){
    geometry_msgs::PoseStamped p;
    sn::poseToRos(*it, p);
    pose_array.poses.push_back(p.pose);
  }
  pose_array.header = scan->header;
  pose_array.header.frame_id = "/map";
  cloudPub_.publish(pose_array);

  sn::poseToTf(localizer_.getPosition(), best_tranform);
  std_msgs::Float64 score_msg;
  score_msg.data = localizer_.getScore();
  scorePub_.publish(score_msg);

  geometry_msgs::PoseStamped p;
  p.header = scan->header;
  p.header.frame_id = "/map";
  p.pose = sn::poseToPose6D(localizer_.getPosition());
  path_.poses.push_back(p);
  path_.header = scan->header;
  path_.header.frame_id = "/map";
  pathPub_.publish(path_);

  if(localizer_.getScore() < 0.5)
    laserPub_.publish(s);

  best_tranform.getRotation().normalize();
  br.sendTransform(tf::StampedTransform(best_tranform.inverse(), s.header.stamp, "/laser", "/map"));
}

void LocalizerNode::mapCallback(OGridPtr map){
  sn::Map mymap;
  sn::rosToMap(map, mymap);
  mymap.computeDistSup(120);
  localizer_.init(mymap);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n(std::string("~"));

  LocalizerNode mynode(n);

  ros::spin();
  return 0;
}
