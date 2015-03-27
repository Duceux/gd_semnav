#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Core>
#include <sn_msgs/DetectionArray.h>
#include <sn_geometry/sn_geometry.h>

struct LaserSegmentation{
  ros::Subscriber scanSub;
  ros::Publisher detectionPub;
  ros::Publisher filteredPub;

  //parameters
  double distance_th;
  double angle_th;

  void callback(const sensor_msgs::LaserScanConstPtr& input);

};

void LaserSegmentation::callback(const sensor_msgs::LaserScanConstPtr &input){
  Eigen::MatrixXd pair_pts(input->ranges.size()-1, 2);
  Eigen::MatrixXd pts(input->ranges.size(), 2);


  for(size_t i=0; i<input->ranges.size()-1; ++i){
    Eigen::Vector2d p1(input->ranges[i]*std::cos(i*input->angle_increment), input->ranges[i]*std::sin(i*input->angle_increment));
    Eigen::Vector2d p2(input->ranges[i+1]*std::cos((i+1)*input->angle_increment), input->ranges[i+1]*std::sin((i+1)*input->angle_increment));
    pair_pts(i, 0) = p2(0) - p1(0);
    pair_pts(i, 1) = p2(1) - p1(1);
  }
  for(size_t i=0; i<input->ranges.size(); ++i){
    pts(i, 0) = input->ranges[i]*std::cos(i*input->angle_increment+input->angle_min);
    pts(i, 1) = input->ranges[i]*std::sin(i*input->angle_increment+input->angle_min);
  }

  //  std::cout << points << std::endl;

  sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan(*input));
  output->ranges.clear();
  output->ranges.resize(input->ranges.size(), 0.0);

  sensor_msgs::LaserScanPtr angles(new sensor_msgs::LaserScan(*input));
  angles->ranges.clear();
  angles->ranges.resize(input->ranges.size(), 0.0);

  // for vizualization it's inverted
  // zeros are the one we wants
  for(size_t i=0; i<input->ranges.size()-1; ++i){
    Eigen::Vector2d p( pair_pts(i,0), pair_pts(i,1));
    double distance = p.norm();
    //    std::cout << distance << std::endl;
    if(distance > distance_th){
      output->ranges[i+1] = input->ranges[i+1];
      output->ranges[i] = input->ranges[i];
    }

  }
  output->ranges[0] = input->ranges[0];
  output->ranges[output->ranges.size()-1] = input->ranges[input->ranges.size()-1];

  std::vector<std::pair<size_t, size_t> > indexes;
  for(size_t i=0; i<output->ranges.size();){
    // look for a zeros
    if(output->ranges[i] == 0.0){
      int k=i, l=i;
      // find the last one
      for(size_t j=i; j<output->ranges.size(); ++j){
        if(output->ranges[j] != 0.0){
          l = j-1;
          break;
        }
      }
      indexes.push_back(std::pair<size_t, size_t>(k,l));
      i = l+1;
      //      std::cout << k << " " << l << std::endl;
    }
    else
      ++i;
  }

  // look at the series of zeros and if the series is too small remove it
  for(size_t i=1; i<indexes.size()-1; ++i)
    if(indexes[i].second - indexes[i].first < 10){// fixme use distance to filter
      //remove
      for(size_t k=indexes[i].first; k<=indexes[i].second; ++k)
        output->ranges[k] = input->ranges[k];
      //      indexes.erase(indexes.begin()+i);
      //      --i;
    }

  for(size_t i=1; i<indexes.size()-1; ++i){
    Eigen::Vector2d p1( pts(indexes[i].first,0), pts(indexes[i].first,1));
    Eigen::Vector2d p2( pts(indexes[i].second,0), pts(indexes[i].second,1));
    if((p1-p2).norm() > 1.0 || (p1-p2).norm() < 0.1){// fixme use distance to filter
      //remove
      for(size_t k=indexes[i].first; k<=indexes[i].second; ++k)
        output->ranges[k] = input->ranges[k];
      //      indexes.erase(indexes.begin()+i);
      //      --i;
    }
  }

  //remove first series of zeros
  for(size_t i=indexes.front().first; i<=indexes.front().second; ++i)
    output->ranges[i] = input->ranges[i];

  // lasts zero
  for(size_t i=indexes.back().first; i<=indexes.back().second; ++i)
    output->ranges[i] = input->ranges[i];

  // remove series that starts behind the previous one
  for(size_t i=1; i<indexes.size()-1; ++i){
    double r1 = input->ranges[indexes[i].second];
    double r2 = input->ranges[indexes[i+1].first];
    if(r1 < r2){
      // remove
      for(size_t k=indexes[i+1].first; k<=indexes[i+1].second; ++k)
        output->ranges[k] = input->ranges[k];
    }
    double r3 = input->ranges[indexes[i].first];
    double r4 = input->ranges[indexes[i-1].second];
    if(r3 < r4){
      // remove
      for(size_t k=indexes[i-1].first; k<=indexes[i-1].second; ++k)
        output->ranges[k] = input->ranges[k];
    }
  }

  for(size_t i=0; i<output->ranges.size(); ++i)
    if(output->ranges[i] == 0.0)
      angles->ranges[i] = input->ranges[i];
    else angles->ranges[i] = 0.0;

  sn_msgs::DetectionArrayPtr res(new sn_msgs::DetectionArray);
  res->header = input->header;
  for(uint i=0; i<angles->ranges.size(); ++i){
    sn_msgs::Detection det;
    while(angles->ranges[i] > 0 && i<angles->ranges.size()){
      geometry_msgs::Point pt;
      pt.x = pts(i, 0);
      pt.y = pts(i, 1);
      det.points.push_back(pt);
      ++i;
    }
    det.header = input->header;
    det.bbox.center = sn::compute_center(det.points);
    if(det.points.size() > 0)
      res->detections.push_back(det);
  }
  detectionPub.publish(res);
  filteredPub.publish(angles);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "laser_segmentation");
  ros::NodeHandle handle(std::string("~"));

  LaserSegmentation segmentation;

  std::string laser_topic;
  ros::param::param<std::string>("~laser_topic",laser_topic,"/scan");
  segmentation.scanSub = handle.subscribe(laser_topic, 1, &LaserSegmentation::callback, &segmentation);
  segmentation.detectionPub = handle.advertise<sn_msgs::DetectionArray>("/detections", 1);
  segmentation.filteredPub = handle.advertise<sensor_msgs::LaserScan>("detection_scan", 1);

  ros::param::param<double>("~distance", segmentation.distance_th, 0.001);
  ROS_INFO("distance: %f", segmentation.distance_th);


  ros::spin();

  return 0;
}
