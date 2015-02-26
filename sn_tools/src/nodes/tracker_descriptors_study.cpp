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

double compute_resolution(sn::vector_pts_t const& pts){
    double resolution = std::numeric_limits<double>::max();
    for(uint i=0; i<pts.size()-1; ++i)
        if(sn::l2_norm(pts[i+1]-pts[i]) > 0.0001)
            resolution = std::min(resolution, sn::l2_norm(pts[i+1]-pts[i])/10.0);
    return resolution;
}

double compute_radius(sn::vector_pts_t const& pts){
    double radius = 0.0;
    for(auto pt: pts)
        radius = std::max(radius, std::max(std::abs(pt.x), std::abs(pt.y)));
    return radius;
}

void draw(cv::Mat& surface,
          const sn::vector_pts_t& view,
          const cv::Scalar& color,
          double resolution){

    //display point
    for(uint j=0; j<view.size(); ++j){
        sn::point_t p = view[j];
        p /= resolution;
        cv::Point pp(p.x + surface.rows/2, p.y + surface.rows/2);
        cv::circle(surface, pp, 1, color, -1);
    }
    for(int i=0; i<surface.rows; ++i)
        for(int j=0; j<surface.cols; ++j)
            if(i==surface.rows/2 || j==surface.cols/2)
                surface.at<unsigned char>(i, j, 0) = 254;
}


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

void draw_descriptor(cv::Mat& surface,
                     const sn::Histogram2D& descriptor,
                     double resolution){

    for(int i=0; i<surface.rows; ++i)
        for(int j=0; j<surface.cols; ++j){
            sn::point_t p = sn::create((j-100.0)*resolution,
                                       (i-100.0)*resolution,
                                       0.0);
            double val = descriptor.get(p);
            surface.at<unsigned char>(i, j) = val*255;

        }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));

    std::string filename = "/home/robotic/Desktop/datasets/trackers/all.bag";
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

    sn::ColorTriangleExtractor extractor;
//    extractor.sampling_resolution = 0.001;
//    extractor.smoothing_factor = 1;
//    extractor.downsampling_factor = 50;
    extractor.theta_bin_size = M_PI*2.0/36.0;
    extractor.nb_triangles = 10000;
    extractor.type = "color";
    bool first = true;
    std::vector<double> prev;
    double cumul_mean = 0.0;
    double cumul_square = 0.0;
    double count = 0.0;
    double cumul_min = 0.0;
    double cumul_max = 0.0;
    double count_min = 0.0;
    double count_max = 0.0;
    for(sn_msgs::TrackerPtr tk: trackers){
        std::cout << tk->uid << std::endl;
        std::cout << tk->name << std::endl;
        int tk_count = 0.0;
        for(sn_msgs::Detection det: tk->detections){
            if(!extractor.is_valid(det))continue;
//            if(tk_count++>40)break;
            sn::descriptor_t feature = extractor(det);

            if(first){
                prev = feature.data;
                first = false;
            }else{
                double distance = sn::symmetric_chi2_distance(prev, feature.data);
                std::cout << distance << std::endl;
                if(std::isnan(distance))continue;
                prev = feature.data;

                cumul_mean += distance;
                cumul_square += distance*distance;
                count+=1.0;
                double mean = cumul_mean/count;
                double square = cumul_mean/count;
                double std = std::sqrt(square-mean*mean);
                if(distance < mean*0.5){
                    cumul_min+=distance;
                    count_min+=1.0;
                }else if(distance > mean*2){
                    cumul_max+=distance;
                    count_max+=1.0;
                }
                std::cout << mean << "\t"
                          << std << "\t"
                          << cumul_min/count_min << "\t"
                          << cumul_max/count_max << "\t"
                          << (cumul_max/count_max)/(cumul_min/count_min) << "\t"
                          << std::endl;

            }
/*
            //INIT
            sn::vector_pts_t initial = sn::transform(det.points, det.robot);
            sn::compute_centered_oriented(det.points, initial);
            double radius = compute_radius(initial);
            double resolution = compute_resolution(initial);
            int size = 500;
            resolution = radius*2/size;
            cv::Mat surface = cv::Mat::zeros(size, size, CV_8UC3);
            draw(surface,
                 initial, cv::Scalar(0,0,255), resolution);
            cv::imshow("points", surface);

*/
            cv::Mat LDTB = cv::Mat::zeros(600, 600, CV_8UC1);
            draw_descriptor(LDTB, extractor.get_histogram(), 400.0/600.0);
            cv::imshow("descriptor", LDTB);
            int key = cv::waitKey();
            if(key==1048603)return 1;
        }
        int key = cv::waitKey();
        if(key==1048603)return 1;
    }


    return 0;
}
