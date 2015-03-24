#ifndef LOCALIZERNODE_H
#define LOCALIZERNODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sn_mapper/conversions.h>
#include <sn_mapper/localizer.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

typedef nav_msgs::Path Path;

class LocalizerNode
{
public:
    LocalizerNode(const ros::NodeHandle& node);

    void scanCallback(LaserPtr scan);

    void mapCallback(OGridPtr map);

    ros::NodeHandle node_;
    ros::Subscriber scanSub_;
    ros::Subscriber mapSub_;
    ros::Publisher laserPub_;
    ros::Publisher cloudPub_;
    ros::Publisher scorePub_;
    ros::Publisher pathPub_;

    Path path_;

    sn::Localizer localizer_;
};

#endif // LOCALIZERNODE_H
