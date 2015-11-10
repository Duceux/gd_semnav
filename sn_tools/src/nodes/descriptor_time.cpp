#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/DetectionArray.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>


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

  std::map<std::string, std::string> laser_ex_params;
  ros::param::get("laser_ex", laser_ex_params);
  for(auto it: laser_ex_params)
    std::cout << it.first << " " << it.second << std::endl;

  std::map<std::string, std::string> ppld_ex_params;
  ros::param::get("ppld_ex", ppld_ex_params);
  for(auto it: ppld_ex_params)
    std::cout << it.first << " " << it.second << std::endl;

  // TLD
  double tld_average;
  double ppl_average;
  {
    sn::Extractor::Ptr tld_extractor(new sn::TriangleLaserExtractor);
    tld_extractor->set_params(laser_ex_params);
    std::vector<sn::descriptor_t> desciptors;
    desciptors.reserve(detections.detections.size());
    ros::Time tld_tic = ros::Time::now();
    for(auto det: detections.detections){
      if(tld_extractor->is_valid(det)){
        desciptors.push_back((*tld_extractor)(det));
      }
    }
    ros::Time tld_toc = ros::Time::now();
    tld_average = (tld_toc - tld_tic).toSec()/desciptors.size();
  }


  // PPLD
  {
    sn::Extractor::Ptr ppld_extractor(new sn::PairOfPointsLaserExtractor);
    ppld_extractor->set_params(ppld_ex_params);
    std::vector<sn::descriptor_t> desciptors;
    desciptors.reserve(detections.detections.size());
    ros::Time ppld_tic = ros::Time::now();
    int count = 0;
    for(auto det: detections.detections){
      if(ppld_extractor->is_valid(det)){
        desciptors.push_back((*ppld_extractor)(det));
      }
    }
    ros::Time ppld_toc = ros::Time::now();
    ppl_average = (ppld_toc - ppld_tic).toSec()/desciptors.size();
  }

  std::cout << "TLD: " << tld_average << std::endl;
  std::cout << "PPLD: " << ppl_average << std::endl;


  return 0;
}

