#include <ros/ros.h>
#include <sn_msgs/DetectionArray.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "laser_feature");
    ros::NodeHandle handle(std::string("~"));

    std::string input;
    ros::param::param<std::string>("~input", input, "/laser_segmentation/detections");

    std::string output;
    ros::param::param<std::string>("~ouput", output, input);



    ros::spin();

    return 0;
}
