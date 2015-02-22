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
#include <sn_features/histogram_distance.h>

void draw_descriptor(cv::Mat& surface,
          const sn::PolarHistogram& descriptor,
                     double resolution){

    for(int i=0; i<surface.rows; ++i)
        for(int j=0; j<surface.cols; ++j){
            sn::point_t p = sn::create((j-surface.cols/2)*resolution,
                                       (i-surface.rows/2)*resolution,
                                       0.0);
            double val = descriptor.get(p);
            surface.at<unsigned char>(i, j) = val*255;

        }
}

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
    ROS_INFO("nb trackers loaded: %lu ", trackers.size());

    sn::TriangleLaserExtractor extractor;
    extractor.sampling_resolution = 0.001;
    extractor.smoothing_factor = 50;
    extractor.downsampling_factor = 50;
    extractor.theta_bin_size = M_PI/12.0;
    extractor.rho_bin_size = 0.1;
    bool first = true;
    std::vector<double> prev;
    for(sn_msgs::TrackerPtr tk: trackers){
        std::cout << tk->uid << std::endl;
        std::cout << tk->header << std::endl;
        for(sn_msgs::Detection det: tk->detections){
            std::vector<double> feature = extractor(det.points);

            if(first){
                prev = feature;
                first = false;
            }else{
//                for(auto d: feature)
//                    std::cout << d << " ";
//                std::cout << std::endl;
//                for(auto d: prev)
//                    std::cout << d << " ";
//                std::cout << std::endl;

                double distance = sn::symmetric_chi2_distance(prev, feature);
                std::cout << distance << std::endl;
                prev = feature;
            }


            cv::Mat LDTB = cv::Mat::zeros(500, 500, CV_8UC1);
            draw_descriptor(LDTB, extractor.get_histogram(), 0.001);
            cv::imshow("descriptor", LDTB);
            int key = cv::waitKey(50);
            if(key==1048603)return 1;
        }
        int key = cv::waitKey();
        if(key==1048603)return 1;
    }


    return 0;
}
