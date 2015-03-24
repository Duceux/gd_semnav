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
#include <boost/filesystem.hpp>
#include <sn_msgs/DescriptorSequence.h>
#include <visualization_msgs/MarkerArray.h>
#include <sn_dictionary/dico.h>
#include <sn_features/histogram_distance.h>
#include <sn_features/pfh_extractor.h>
#include <sn_features/image_extractor.h>

namespace fs = boost::filesystem;

struct compare_func{ bool operator()(sn_msgs::TrackerPtr const& l, sn_msgs::TrackerPtr const& r){return l->uid<r->uid;} };
typedef std::set<sn_msgs::TrackerPtr, compare_func> TrackersSet;

typedef sn_msgs::DescriptorSequence Seq;
typedef sn_msgs::Descriptor Des;
typedef std::vector<Seq> VSeq;

void load(const std::string& filename, TrackersSet& trackers)
{
  std::cout << "opening: " << filename << std::endl;
  rosbag::Bag bag;
  try{
    bag.open(filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/tracking/ended_tracker"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(auto m: view){
      sn_msgs::TrackerPtr g = m.instantiate<sn_msgs::Tracker>();
      if (g != NULL){
        trackers.insert(g);
        if(g->name.size() != 0)
          (*trackers.find(g))->name = g->name;
      }
    }
    bag.close();
  }
  catch(const std::exception& e){
    ROS_ERROR("%s", e.what());
  }
  ROS_INFO("nb trackers loaded: %lu ", trackers.size());

}

void save(const std::string& filename, VSeq const& trackers)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  long unsigned int total = 0;
  for(auto tck: trackers){
    try{
      bag.write("sequence", ros::Time::now(), tck);
      total++;
    }catch(const std::exception& e){
      ROS_ERROR("%s", e.what());
      bag.close();
      return;
    }
  }
  bag.close();

  ROS_INFO("Saved %lu sequences to %s", total, filename.c_str());
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "sequence_viewer");
  ros::NodeHandle handle(std::string("~"));


  TrackersSet trackers;
  std::string filename;
  ros::param::param<std::string>("~filename", filename,
                                 "/home/robotic/Desktop/datasets/trackers/all.bag");
  load(filename, trackers);

  std::map<std::string, std::string> pfh_ex_params;
  ros::param::get("pfh_ex", pfh_ex_params);
  for(auto it: pfh_ex_params)
    std::cout << it.first << " " << it.second << std::endl;

  std::map<std::string, std::string> laser_ex_params;
  ros::param::get("laser_ex", laser_ex_params);
  for(auto it: laser_ex_params)
    std::cout << it.first << " " << it.second << std::endl;

  std::map<std::string, std::string> tbgr_ex_params;
  ros::param::get("tbgr_ex", tbgr_ex_params);
  for(auto it: tbgr_ex_params)
    std::cout << it.first << " " << it.second << std::endl;

  std::map<sn::Word, std::array<float, 3>> mColors;
  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.2, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("pfh", 0.05, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("size", 0.01, sn::Distance(sn::euclidean_distance));
  dicos.set("color", 3.0, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("tbgr", 0.1, sn::Distance(sn::symmetric_chi2_distance));

  sn::TriangleLaserExtractor::Ptr extractor(new sn::TriangleLaserExtractor);
  extractor->set_params(laser_ex_params);

  sn::PFHExtractor::Ptr pfh_ex(new sn::PFHExtractor);
  pfh_ex->set_params(pfh_ex_params);

  sn::SizeExtractor::Ptr size_ex(new sn::SizeExtractor);
  size_ex->type = "size";

  sn::ColorTriangleExtractor tmp;
  tmp.type = "color";
  sn::ColorTriangleExtractor::Ptr color_ex(new sn::ColorTriangleExtractor(tmp));

  sn::ImageDescriptorExtractor::Ptr tbgr_ex(new sn::ImageDescriptorExtractor);
  tbgr_ex->set_params(tbgr_ex_params);

  std::vector<sn::Extractor::Ptr> extractors;
  extractors.push_back(extractor);
  extractors.push_back(pfh_ex);
  extractors.push_back(size_ex);
  extractors.push_back(color_ex);
  extractors.push_back(tbgr_ex);

  std::map<std::string, double> height_map;
  height_map["laser"] = 0.0;
  height_map["pfh"] = 0.0;
  height_map["size"] = 0.02;
  height_map["color"] = 0.02;
  height_map["tbgr"] = 0.04;


  ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/path",1,true);
  ros::Publisher cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud",1,true);
  ros::Publisher kinect_pub = handle.advertise<sensor_msgs::PointCloud2>("/kinect_cloud",1,true);
  ros::Publisher poses_pub = handle.advertise<geometry_msgs::PoseArray>("/poses", 1, true);
  ros::Publisher marker_pub = handle.advertise<visualization_msgs::MarkerArray>("/words",1000,true);
  VSeq vseq;

  for(sn_msgs::TrackerPtr tk: trackers){
    nav_msgs::Path path;
    std::cout << tk->uid << std::endl;
    std::cout << tk->name << std::endl;
    std::cout << tk->detections.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> ptcld;
    pcl::PointCloud<pcl::PointXYZRGB> ptcld2;
    geometry_msgs::PoseArray parray;
    bool has_laser = false;
    bool has_kinect = false;
    for(auto det: tk->detections){
      //Path
      geometry_msgs::PoseStamped pose;
      tf::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, det.robot.theta);
      quaternion.normalize();
      tf::quaternionTFToMsg( quaternion, pose.pose.orientation);
      pose.pose.position.x = det.robot.x;
      pose.pose.position.y = det.robot.y;
      pose.pose.position.z = 0.0;
      pose.header = det.header;
      path.poses.push_back(pose);

      //cloud
      for(sn::point_t p: det.points){
        pcl::PointXYZRGB tmp;
        tmp.x = p.x;
        tmp.y = p.y;
        tmp.z = p.z;
        ptcld.push_back(tmp);
        has_laser = true;
      }

      //pose array
      parray.poses.push_back(pose.pose);

      //kinect
      if(det.cloud.data.size() > 0){
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        pcl::fromROSMsg(det.cloud, tmp);
        ptcld2.insert(ptcld2.end(), tmp.begin(), tmp.end());
        has_kinect = true;
      }

    }
    path.header = tk->header;
    sensor_msgs::PointCloud2 cloud, cloud2;
    pcl::toROSMsg(ptcld, cloud);
    cloud.header = tk->header;
    parray.header = tk->header;
    ptcld2.insert(ptcld2.end(), ptcld.begin(), ptcld.end());
    pcl::toROSMsg(ptcld2, cloud2);
    cloud2.header = tk->header;


    Seq seq;
    seq.uid = tk->uid;
    seq.header = tk->header;
    seq.name = tk->name;
    pcl::toROSMsg(ptcld2, seq.cloud);
    seq.cloud.header = tk->header;
    //descriptors
    for(sn_msgs::Detection det: tk->detections){
      for(sn::Extractor::Ptr ex: extractors){
        if(ex->is_valid(det)){
          seq.descriptors.push_back((*ex)(det));
        }
      }
    }

    //marker
    static visualization_msgs::MarkerArray marray;
    for(int i=0; i<marray.markers.size(); ++i)
    {
      marray.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_pub.publish(marray);
    if(marray.markers.size() < seq.descriptors.size()*2)
      marray.markers.resize(seq.descriptors.size()*2);
    for(int d=0; d<seq.descriptors.size(); d++)
    {
      const sn_msgs::Descriptor& des = seq.descriptors[d];
      sn::Word w = dicos.get(des);

      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      auto label = w;
      if(mColors.count( label ) == 0){
        static std::default_random_engine generator;
        static std::uniform_real_distribution<float> distribution(0.3f,0.7f);
        static auto random = std::bind ( distribution, generator );
        mColors[label][0] = random();
        mColors[label][1] = random();
        mColors[label][2] = random();
      }
      marker.pose.position.x = des.robot.x;
      marker.pose.position.y = des.robot.y;
      marker.pose.position.z = height_map[des.type];
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.01;
      marker.color.r = mColors[label][0];
      marker.color.g = mColors[label][1];
      marker.color.b = mColors[label][2];
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();
      marker.header.frame_id = tk->header.frame_id;
      marker.ns = "all";
      marker.id = d*2;
      marray.markers[d*2] = marker;
      marker.ns = des.type;
      marker.id = d*2+1;
      marray.markers[d*2+1] = marker;
    }


    marker_pub.publish(marray);
    cloud_pub.publish(cloud);
    path_pub.publish(path);
    poses_pub.publish(parray);
    kinect_pub.publish(cloud2);

    std::string mystr;
    std::cout << "Press enter to continue\n";
    std::getline (std::cin, mystr);

    //        vseq.push_back(seq);
    //        save("/home/robotic/Desktop/datasets/sequences/viewer.bag", vseq);

  }

  return 0;
}
