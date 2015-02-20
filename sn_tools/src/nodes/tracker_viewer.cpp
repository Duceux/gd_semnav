#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/Tracker.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PoseArray.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));

    std::string filename = "/home/robotic/Desktop/datasets/trackers/end_tracker.bag";
    std::vector<sn_msgs::TrackerPtr> trackers;

    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("/tracking/ended_tracker"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(auto m: view){
            sn_msgs::TrackerPtr g = m.instantiate<sn_msgs::Tracker>();
            if (g != NULL){
                trackers.push_back(g);
            }
        }
        bag.close();
    }
    catch(const std::exception& e){
        ROS_ERROR("%s", e.what());
    }
    ROS_INFO("nb trackers loaded: %u ", trackers.size());

    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/path",1,true);
    ros::Publisher cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("/cloud",1,true);
    ros::Publisher poses_pub = handle.advertise<geometry_msgs::PoseArray>("/poses", 1, true);
    for(sn_msgs::TrackerPtr tk: trackers){
        nav_msgs::Path path;
        sensor_msgs::PointCloud2 cloud;
        std::cout << tk->header << std::endl;
        std::cout << tk->uid << std::endl;
        pcl::PointCloud<pcl::PointXYZ> ptcld;
        geometry_msgs::PoseArray parray;
        for(auto det: tk->detections){
            //Path
            geometry_msgs::PoseStamped pose;
            tf::Quaternion quaternion;
            quaternion.setEuler(det.robot.theta, 0.0, 0.0);
            quaternion.normalize();
            tf::quaternionTFToMsg( quaternion, pose.pose.orientation);
            pose.pose.position.x = det.robot.x;
            pose.pose.position.y = det.robot.y;
            pose.pose.position.z = 0.0;
            pose.header = det.header;
            path.poses.push_back(pose);

            //cloud
            for(sn::point_t p: det.points)
                ptcld.push_back(pcl::PointXYZ(p.x, p.y, p.z));

            //pose array
            parray.poses.push_back(pose.pose);
        }
        path.header = tk->header;
        path_pub.publish(path);
        pcl::toROSMsg(ptcld, cloud);
        cloud.header = tk->header;
        cloud_pub.publish(cloud);
        parray.header = tk->header;
        poses_pub.publish(parray);
        std::cout << "Press enter to continue" << std::endl;
        std::string mystr;
        std::getline(std::cin, mystr);
    }
    return 1;
}
