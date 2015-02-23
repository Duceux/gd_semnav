#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/DetectionArray.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>

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

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));


    std::string filename = "/home/robotic/Desktop/datasets/detections/process.bag";
    sn_msgs::DetectionArray detections;

    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        rosbag::View view(bag);
        for(auto m: view){
            boost::shared_ptr<sn_msgs::DetectionArray> g = m.instantiate<sn_msgs::DetectionArray>();
            if (g != NULL){
                detections.detections.insert(detections.detections.end(),
                                             g->detections.begin(),
                                             g->detections.end());
            }
        }
        bag.close();
    }
    catch(const std::exception& e){
        ROS_ERROR("%s", e.what());
    }
    ROS_INFO("nb detections loaded: %lu ", detections.detections.size());

    for(auto det: detections.detections){

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

        //SAMPLING
        sn::vector_pts_t resampled = initial;
        sn::up_sampling(initial, resampled, 0.001);
        sn::vector_pts_t smoothed = resampled;
        sn::smooth(resampled, smoothed, 50);
        radius = compute_radius(smoothed);
        resolution = compute_resolution(smoothed);
        size = 500;
        resolution = radius*2/size;
        cv::Mat sampling = cv::Mat::zeros(size, size, CV_8UC3);
        draw(sampling,
             smoothed, cv::Scalar(0,255,0), resolution);

        // Down sampled
        sn::vector_pts_t centered = smoothed;
        sn::down_sampling_nb(smoothed, centered, 50);
        radius = compute_radius(centered);
        resolution = compute_resolution(centered);
        size = 500;
        resolution = radius*2/size;
        cv::Mat oriented = cv::Mat::zeros(size, size, CV_8UC3);
        draw(oriented,
             centered, cv::Scalar(0,128,128), resolution);

        //PPTS
        sn::vector_pts_t ppts;
        sn::triangle_points(centered, ppts);
        radius = compute_radius(ppts);
        resolution = compute_resolution(ppts);
        size = 500;
        resolution = radius*2/size;
        cv::Mat pairofpt = cv::Mat::zeros(size, size, CV_8UC3);
        draw(pairofpt,
             ppts, cv::Scalar(255, 0, 0), resolution);

        //Descriptor
        sn::PolarHistogram descriptor(M_PI/36.0, 0.01);
        for(auto p: ppts)
            descriptor.add(p);
        descriptor.inf_normalize();
//        descriptor.print();

        cv::Mat LDTB = cv::Mat::zeros(size, size, CV_8UC1);
        draw_descriptor(LDTB, descriptor, resolution);


        cv::imshow("surface", surface);
        cv::imshow("sampling", sampling);
        cv::imshow("oriented", oriented);
        cv::imshow("pair of points", pairofpt);
        cv::imshow("descriptor", LDTB);
        int key = cv::waitKey(50);
        if(key==1048603)return 1;



    }


    return 0;
}
