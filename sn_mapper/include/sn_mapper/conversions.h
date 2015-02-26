#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <sn_mapper/map.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef nav_msgs::OccupancyGrid OGrid;
typedef nav_msgs::OccupancyGridConstPtr OGridPtr;
typedef sensor_msgs::LaserScan Laser;
typedef sensor_msgs::LaserScanConstPtr LaserPtr;
typedef geometry_msgs::PoseWithCovarianceStampedConstPtr PoseCovStampPtr;
typedef geometry_msgs::PoseStamped PoseStamped;


namespace sn {

void mapToRos(const Map &map, OGrid& og);

void rosToMap(OGrid& og, Map& map);

void rosToMap(OGridPtr og, Map& map);

void rosToProjectedLaser(const Laser& s, const tf::Transform& tf, vector_pts_t& l);

void poseToTf(const pose_t& robot, tf::Transform& tf);

void tfToPose(const tf::Transform &tf, pose_t &robot);

void rosToLaser(const Laser& s, vector_pts_t& l);

void rosToPose(PoseCovStampPtr pose, pose_t& robot);

void poseToRos(const pose_t& robot, PoseStamped& pose);

}

#endif // CONVERSIONS_H
