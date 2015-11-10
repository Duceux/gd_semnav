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
#include <sn_tools/io.h>
#include <sn_models/bag_of_word.h>
#include <sn_models/graph_of_word.h>
#include <sn_graph/graphiz.hh>

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;
namespace fs = boost::filesystem;
namespace sngl = sn::graph_learning;

std::string print_node(const sn::node_t &node){
  std::stringstream str;
  str << std::setprecision(2);
  str << "[ label = \"" <<  node.key.type << "\n" << node.key.label
      << "\" ];";
  return str.str();
}

std::string print_edge(const sn::edge_t &e){
  return "";
}

std::string print_graph(sn::GraphOfWordData const& graph) {
  std::stringstream str;
  str << "start = rand;\n";
  str << "overlap=false;\n";
  str << "splines=true;\n";
  str << "edge [fontsize=8];\n";
  str << "node [fontsize=10];\n";
  return str.str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "sequence_viewer2");
  ros::NodeHandle handle(std::string("~"));

  std::string source;
  ros::param::param<std::string>("~source_dir", source,
                                 "/home/robotic/Desktop/phd-dataset/sequence/");
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
    result_set.insert("");

  VSeq trackers;

  for(fs::path it: result_set){
    sn::tools::load(source+it.string(), trackers, "sequence");
    std::cout << "reading: " << it.filename().string() << std::endl;
  }

  ROS_INFO("nb sequences loaded: %lu ", trackers.size());

  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.2, sn::Distance(sn::symmetric_chi2_distance));
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
  height_map["pfh"] = 0.20;
  height_map["size"] = 0.40;
  height_map["color"] = 0.60;
  height_map["tbgr"] = 0.80;

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

    sn::BagOfWord::Ptr bow = sn::BagOfWord::create();
    sn::GraphOfWord::Ptr gow = sn::GraphOfWord::create();
    for(int d=0; d<seq->descriptors.size(); d++)
    {
      const sn_msgs::Descriptor& des = seq->descriptors[d];
      sn::Word w = dicos.get(des);

      bow->add(w);
      gow->add(w);

      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      auto label = w;
      if(mColors.count( label ) == 0){
        static std::default_random_engine generator;
        static std::uniform_real_distribution<float> distribution(0.0f,0.5f);
        static auto random = std::bind ( distribution, generator );
        mColors[label][0] = random();
        mColors[label][1] = random();
        mColors[label][2] = random();
      }
      marker.pose.position.x = des.robot.x;
      marker.pose.position.y = des.robot.y;
      marker.pose.position.z = 0.0 + height_map[des.type];
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
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

    bow->print();
    gow->print();

    sn::graph::graphiz_save(gow->get_graph(), "/home/robotic/Dropbox/phd_duceux/images/captures/gow.gv",
                            print_node, print_edge, print_graph);

    std::string mystr;
    std::cout << "Press enter to continue\n";
    std::getline (std::cin, mystr);
  }
  return 0;

}
