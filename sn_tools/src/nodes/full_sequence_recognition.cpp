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

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;

void load(const std::string& filename, VSeq& trackers)
{
  std::cout << "opening: " << filename << std::endl;
  rosbag::Bag bag;
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

typedef std::unordered_set<sn::Word> BinaryBagOfWord;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "full_reco");
  ros::NodeHandle handle(std::string("~"));


  VSeq trackers;
  load("/home/robotic/Desktop/datasets/sequences/viewer.bag", trackers);

  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.05, sn::Distance(sn::symmetric_chi2_distance));
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
  for(auto des: descriptors){
    dicos.get(des);
  }
  std::cout << descriptors.size() << std::endl;
  std::cout << dicos.size("laser") << std::endl;

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

  std::vector<sn::GraphOfWord::Ptr> truth;
  std::vector<sn::GraphOfWord::Ptr> test;
  std::map<std::string, int> labels;
  for(sn::GraphOfWord::Ptr g: graphs){
    if(labels[g->name] < 2){
      truth.push_back(g);
      labels[g->name]++;
    }
    else
      test.push_back(g);
  }

  std::cout << "Intersection\n";
  auto func = sn::comparison::intersection_size(sn::no_type());
  auto matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  std::cout << "IntersectionVsUnion\n";
  func = sn::comparison::intersection_union_size(sn::no_type());
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

//  std::cout << "NodeIntersection\n";
//  func = sn::comparison::node_intersection(sn::no_type());
//  matrix = sn::confusion_matrix(truth, test, func);
//  std::cout << sn::get_precision(matrix) << std::endl;
//  sn::print(matrix);

//  std::cout << "EdgeIntersection\n";
//  func = sn::comparison::edge_intersection(sn::no_type());
//  matrix = sn::confusion_matrix(truth, test, func);
//  std::cout << sn::get_precision(matrix) << std::endl;
//  sn::print(matrix);

//  std::cout << "MaxComponent\n";
//  func = sn::comparison::max_component_size(sn::no_type());
//  matrix = sn::confusion_matrix(truth, test, func);
//  std::cout << sn::get_precision(matrix) << std::endl;
//  sn::print(matrix);

  std::cout << "LaserIntersection\n";
  func = sn::comparison::intersection_size(sn::single_type("laser", true));
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  std::cout << "PfhIntersection\n";
  func = sn::comparison::intersection_size(sn::single_type("pfh", true));
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  std::cout << "TbgrIntersection\n";
  func = sn::comparison::intersection_size(sn::single_type("tbgr", true));
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  std::cout << "SizeIntersection\n";
  func = sn::comparison::intersection_size(sn::single_type("size", true));
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  std::cout << "ColorIntersection\n";
  func = sn::comparison::intersection_size(sn::single_type("color", true));
  matrix = sn::confusion_matrix(truth, test, func);
  std::cout << sn::get_precision(matrix) << std::endl;
  sn::print(matrix);

  return 0;
}
