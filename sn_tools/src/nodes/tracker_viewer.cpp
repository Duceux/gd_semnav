#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/Tracker.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));

    std::string filename = "/home/robotic/Desktop/datasets/trackers/end_tracker.bag";
    std::vector<sn_msgs::Tracker> trackers;

    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("/tracking/ended_tracker"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(auto m: view){
            boost::shared_ptr<sn_msgs::Tracker> g = m.instantiate<sn_msgs::Tracker>();
            if (g != NULL){
                trackers.push_back(*g);
            }
        }
        bag.close();
    }
    catch(const std::exception& e){
        ROS_ERROR("%s", e.what());
    }
    ROS_INFO("nb trackers loaded: %u ", trackers.size());



}
