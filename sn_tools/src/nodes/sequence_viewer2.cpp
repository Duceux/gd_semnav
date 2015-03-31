#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/DescriptorSequence.h>
#include <sn_dictionary/dico.h>
#include <unordered_map>
#include <sn_features/histogram_distance.h>
#include <unordered_set>
#include <sn_models/bag_of_word.h>
#include <sn_tools/confusion_matrix.h>
#include <sn_models/graph_of_word.h>
#include <sn_models/comparison.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <sn_graph_learning/classifier.h>
#include <sn_models/comparison_cluster.h>
#include <sn_graph_learning/cluster_holder.h>
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


typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;
namespace fs = boost::filesystem;
namespace sngl = sn::graph_learning;

void load(const std::string& filename, VSeq& trackers)
{
  std::cout << "opening: " << filename << std::endl;
  rosbag::Bag bag;
  boost::smatch match;
  boost::regex e ("([^0-9]+)_");
  try{
    bag.open(filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("sequence"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(auto m: view){
      Sequence::Ptr g = m.instantiate<sn_msgs::DescriptorSequence>();
      if (g != NULL){
        trackers.push_back(g);
      }
    }
    bag.close();
  }
  catch(const std::exception& e){
    ROS_ERROR("%s", e.what());
  }
  ROS_INFO("nb sequences loaded: %lu ", trackers.size());

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "sequence_viewer2");
  ros::NodeHandle handle(std::string("~"));

  std::string source;
  ros::param::param<std::string>("~source_dir", source,
                                 "/home/duceux/Desktop/phd-dataset/trackers/");
  fs::path sourceDir(source);
  fs::directory_iterator end_iter;
  typedef std::set<fs::path> result_set_t;
  result_set_t result_set;

  if ( fs::exists(sourceDir) && fs::is_directory(sourceDir))
  {
    for( fs::directory_iterator dir_iter(sourceDir) ; dir_iter != end_iter ; ++dir_iter)
    {
      if (fs::is_regular_file(dir_iter->status()) )
      {
        result_set.insert(dir_iter->path().filename());
      }
    }
  }else if( fs::exists(sourceDir) && fs::is_regular_file(sourceDir))
    result_set.insert(sourceDir.filename());

  VSeq trackers;

  for(fs::path it: result_set){
    load(source+it.string(), trackers);
    std::cout << "reading: " << it.filename().string() << std::endl;
    std::string filename = it.filename().string();
  }

  ROS_INFO("nb sequences loaded: %lu ", trackers.size());

  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.1, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("pfh", 0.05, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("size", 0.01, sn::Distance(sn::euclidean_distance));
  dicos.set("color", 1.5, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("tbgr", 0.1, sn::Distance(sn::symmetric_chi2_distance));

  std::vector<Descriptor> descriptors;
  descriptors.reserve(100000);
  for(auto& it: trackers)
    for(auto& des: it->descriptors){
      descriptors.push_back(des);
    }
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(descriptors.begin(), descriptors.end(), g);
  std::cout << descriptors.size() << std::endl;
  if(descriptors.size() > 2000)
    descriptors.resize(2000);
  for(auto des: descriptors){
    dicos.get(des);
  }

  ros::Publisher marker_pub = handle.advertise<visualization_msgs::MarkerArray>("/words",1000,true);
  ros::Publisher kinect_pub = handle.advertise<sensor_msgs::PointCloud2>("/kinect_cloud",1,true);

  std::map<std::string, double> height_map;
  height_map["laser"] = 0.0;
  height_map["pfh"] = 0.0;
  height_map["size"] = 0.02;
  height_map["color"] = 0.02;
  height_map["tbgr"] = 0.04;

  std::map<sn::Word, std::array<float, 3>> mColors;

  for(Sequence::Ptr seq: trackers){
    std::cout << seq->uid << std::endl;
    std::cout << seq->name << std::endl;
    std::cout << seq->descriptors.size() << std::endl;
    //marker
    static visualization_msgs::MarkerArray marray;
    for(int i=0; i<marray.markers.size(); ++i)
    {
      marray.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_pub.publish(marray);
    if(marray.markers.size() < seq->descriptors.size()*2)
      marray.markers.resize(seq->descriptors.size()*2);
    for(int d=0; d<seq->descriptors.size(); d++)
    {
      const sn_msgs::Descriptor& des = seq->descriptors[d];
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
      marker.header.frame_id = seq->header.frame_id;
      marker.ns = "all";
      marker.id = d*2;
      marray.markers[d*2] = marker;
      marker.ns = des.type;
      marker.id = d*2+1;
      marray.markers[d*2+1] = marker;
    }


    marker_pub.publish(marray);
    kinect_pub.publish(seq->cloud);

    std::string mystr;
    std::cout << "Press enter to continue\n";
    std::getline (std::cin, mystr);
  }
  return 0;

}
