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
      if(i==surface.rows/2 || j==surface.cols/2){
        surface.at<cv::Vec3b>(i, j) = cv::Vec3b(255,0,0);
      }
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
      surface.at<unsigned char>(i, j) = 255-val*255;

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
    //TLD
    /*
        //INIT
        sn::vector_pts_t initial = sn::transform(det.points, det.robot);
        sn::compute_centered_oriented(det.points, initial);
        double radius = compute_radius(initial);
        double resolution = compute_resolution(initial);
        int size = 500;
        resolution = radius*2/size;
        cv::Mat surface = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
        draw(surface,
             initial, cv::Scalar(0,0,0), resolution);

        //SAMPLING
        sn::vector_pts_t resampled = initial;
        sn::up_sampling(initial, resampled, 0.001);
        sn::vector_pts_t smoothed = resampled;
        sn::smooth(resampled, smoothed, 50);
        radius = compute_radius(smoothed);
        resolution = compute_resolution(smoothed);
        size = 500;
        resolution = radius*2/size;
        cv::Mat smoothing = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
        draw(smoothing,
             smoothed, cv::Scalar(0,0,0), resolution);
        cv::Mat sampling = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
        draw(sampling,
             resampled, cv::Scalar(0,0,0), resolution);

        // Down sampled
        sn::vector_pts_t centered = smoothed;
        sn::down_sampling_nb(smoothed, centered, 50);
        radius = compute_radius(centered);
        resolution = compute_resolution(centered);
        size = 500;
        resolution = radius*2/size;
        cv::Mat oriented = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
        draw(oriented,
             centered, cv::Scalar(0,0,0), resolution);

        //PPTS
        sn::vector_pts_t ppts;
        sn::triangle_points(centered, ppts);
        radius = compute_radius(ppts);
        resolution = compute_resolution(ppts);
        size = 500;
        resolution = radius*2/size;
        cv::Mat pairofpt = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
        draw(pairofpt,
             ppts, cv::Scalar(0, 0, 0), resolution);

        //Descriptor
        sn::PolarHistogram descriptor(M_PI/36.0, 0.01);
        for(auto p: ppts)
            descriptor.add(p);
        descriptor.inf_normalize();
//        descriptor.print();

        cv::Mat LDTB = cv::Mat(size, size, CV_8UC1, cv::Scalar(255));
        draw_descriptor(LDTB, descriptor, resolution);

        cv::imshow("surface", surface);
        cv::imshow("sampling", sampling);
        cv::imshow("smoothing", smoothing);
        cv::imshow("oriented", oriented);
        cv::imshow("pair of points", pairofpt);
        cv::imshow("descriptor", LDTB);
        int key = cv::waitKey();
        if(key==1048603)return 1;

*/
    /*
      // PPLD
      //INIT
      sn::vector_pts_t initial = sn::transform(det.points, det.robot);
      sn::compute_centered_oriented(det.points, initial);
      double radius = compute_radius(initial);
      double resolution = compute_resolution(initial);
      int size = 500;
      resolution = radius*2/size;
      cv::Mat surface = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
      draw(surface,
           initial, cv::Scalar(0,0,0), resolution);

      //SAMPLING
      sn::vector_pts_t resampled = initial;
      sn::up_sampling(initial, resampled, 0.001);
      radius = compute_radius(resampled);
      resolution = compute_resolution(resampled);
      size = 500;
      resolution = radius*2/size;
      cv::Mat sampling = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
      draw(sampling,
           resampled, cv::Scalar(0,0,0), resolution);

      //PPTS
      sn::vector_pts_t ppts;
      sn::pair_of_points(resampled, ppts);
      radius = compute_radius(ppts);
      resolution = compute_resolution(ppts);
      size = 500;
      resolution = radius*2/size;
      cv::Mat pairofpt = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
      draw(pairofpt,
           ppts, cv::Scalar(0, 0, 0), resolution);

      // Angle ref
      sn::vector_pts_t ppts_reoriented;
      double ref = sn::compute_angle_ref(ppts);
      ppts_reoriented = sn::transform(ppts, sn::create_pose(0,0,ref));
      radius = compute_radius(ppts_reoriented);
      resolution = compute_resolution(ppts_reoriented);
      size = 500;
      resolution = radius*2/size;
      cv::Mat pairofptangle = cv::Mat(size, size, CV_8UC3, cv::Scalar(255,255,255));
      draw(pairofptangle,
           ppts_reoriented, cv::Scalar(0, 0, 0), resolution);

      //Descriptor
      sn::PolarHistogram descriptor(M_PI/36.0, 0.01);
      for(auto p: ppts_reoriented)
          descriptor.add(p);
      descriptor.inf_normalize();
//        descriptor.print();

      cv::Mat LDTB = cv::Mat(size, size, CV_8UC1, cv::Scalar(255));
      draw_descriptor(LDTB, descriptor, resolution);

      cv::imshow("surface", surface);
      cv::imshow("sampling", sampling);
      cv::imshow("pair of points", pairofpt);
      cv::imshow("oriented", pairofptangle);
      cv::imshow("descriptor", LDTB);
      int key = cv::waitKey();
      if(key==1048603)return 1;
*/

    // COMP
    std::map<std::string, std::string> laser_ex_params;
    ros::param::get("laser_ex", laser_ex_params);
    for(auto it: laser_ex_params)
      std::cout << it.first << " " << it.second << std::endl;

    std::map<std::string, std::string> ppld_ex_params;
    ros::param::get("ppld_ex", ppld_ex_params);
    for(auto it: ppld_ex_params)
      std::cout << it.first << " " << it.second << std::endl;

    double size = 500.0;
    double resolution = 1.0*2.0/size;
    // TLD
    std::shared_ptr<sn::TriangleLaserExtractor> tld_extractor(new sn::TriangleLaserExtractor);
    tld_extractor->set_params(laser_ex_params);
    ros::Time tld_tic = ros::Time::now();
    (*tld_extractor)(det);
    ros::Time tld_toc = ros::Time::now();
    std::cout << "TLD: " << (tld_toc - tld_tic).toSec() << std::endl;
    cv::Mat TLD = cv::Mat(size, size, CV_8UC1, cv::Scalar(255));
    draw_descriptor(TLD, tld_extractor->get_histogram(), resolution);

    // PPLD
    std::shared_ptr<sn::PairOfPointsLaserExtractor> ppld_extractor(new sn::PairOfPointsLaserExtractor);
    ppld_extractor->set_params(ppld_ex_params);
    ros::Time ppld_tic = ros::Time::now();
    (*ppld_extractor)(det);
    ros::Time ppld_toc = ros::Time::now();
    std::cout << "PPLD: " << (ppld_toc - ppld_tic).toSec() << std::endl;

    cv::Mat PPLD = cv::Mat(size, size, CV_8UC1, cv::Scalar(255));
    draw_descriptor(PPLD, ppld_extractor->get_histogram(), resolution);

    cv::imshow("TLD", TLD);
    cv::imshow("PPLD", PPLD);
    int key = cv::waitKey();
    if(key==1048603)return 1;

  }
  return 0;
}
