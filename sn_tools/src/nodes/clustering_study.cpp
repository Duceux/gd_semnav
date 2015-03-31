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
#include <sn_graph_learning/clustering.h>
#include <sn_graph/sn_graph.hh>

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
        boost::regex_search(g->name,match,e);
        g->name = match[0];
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

struct Color{
  float r, g, b;

  Color(){
    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> distribution(0.5, 1.0);

    r = distribution(generator);
    g = distribution(generator);
    b = distribution(generator);
  }

  std::string toString()const{
    std::stringstream str;
    str << '\"' << r << ", " << g << ", " << b << '\"';
    return str.str();
  }
};

typedef sn::Graph<sn::SimpleNode<sn::GraphOfWordPtr>,
sn::Edge<sn::GraphOfWordPtr, double, true>> MyGraph;


std::string print_node(const typename MyGraph::node_type& node){
  static std::map<std::string, Color> color_map;
  std::stringstream str;
  str << node.key->uid.toNSec() << "[ label = \"" <<  node.key->name
      << "\", color="<< color_map[node.key->name].toString() <<", style=filled" << " ];";
  return str.str();
}


std::string print_edge(const typename MyGraph::edge_type& e){
  std::stringstream str;
  str << e.parent->uid.toNSec();
  if(e.directed)
    str << "->";
  else
    str << "--";
  str << e.child->uid.toNSec();
  str  << "[ label = \"" <<  e.val << "\" ];\n";
  return str.str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "full_reco");
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

  std::vector<sn::GraphOfWord::Ptr> graphs;
  for(sn_msgs::DescriptorSequencePtr seq_ptr: trackers){
    auto graph_ptr = sn::GraphOfWord::create();
    graph_ptr->name = seq_ptr->name;
    graph_ptr->uid = seq_ptr->uid;
    for(sn::descriptor_t des: seq_ptr->descriptors){
      graph_ptr->add(dicos.get(des));
    }
    graph_ptr->print();
    graphs.push_back(graph_ptr);
  }
  std::shuffle(graphs.begin(), graphs.end(), g);

  MyGraph graph_of_models;

  for(auto candidate: graphs){
    graph_of_models.insert_node(candidate);
  }

  auto func = sn::comparison::inclusion(sn::no_type());
  graph_of_models = sn::MaximumSpanningTree(graph_of_models,
                                            func,
                                            false);
  for(auto e: graph_of_models.get_edges())
    if(e.val < 0.2)
      graph_of_models.remove(e);

  std::stringstream str;
  str << "/home/robotic/Desktop/results/graphs/total.gv";
  sn::graph::graphiz_save(graph_of_models, str.str(), print_node, print_edge);

  return 0;
}
