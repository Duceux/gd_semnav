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

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;
namespace fs = boost::filesystem;

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

typedef std::unordered_set<sn::Word> BinaryBagOfWord;
typedef std::vector<sn::GraphOfWord::Ptr> GraphPtrVector;
template <typename ModelContainer>
void split_truth_is_biggest(ModelContainer const& source, ModelContainer& truth, ModelContainer& test){

  std::unordered_map<std::string, typename ModelContainer::value_type> tmp_truth;
  for(auto g: source){
    if(tmp_truth.count(g->name) == 0){
      tmp_truth[g->name] = g;
    }
    else if(tmp_truth[g->name]->nb_words() < g->nb_words())
      tmp_truth[g->name] = g;
  }

  for(auto it: tmp_truth)
    truth.push_back(it.second);

  for(auto g: source)
    if(tmp_truth[g->name]->uid != g->uid)
      test.push_back(g);
}

typedef std::unordered_set<sn::Word> BinaryBagOfWord;
typedef std::vector<sn::GraphOfWord::Ptr> GraphPtrVector;
template <typename ModelContainer>
void split_truth_test_are_biggest(ModelContainer const& source, ModelContainer& truth, ModelContainer& test){

  std::unordered_map<std::string, typename ModelContainer::value_type> tmp_truth;
  for(auto g: source){
    if(tmp_truth.count(g->name) == 0){
      tmp_truth[g->name] = g;
    }
    else if(tmp_truth[g->name]->nb_words() < g->nb_words())
      tmp_truth[g->name] = g;
  }

  std::unordered_map<std::string, typename ModelContainer::value_type> tmp_test;
  for(auto g: source){
    if(tmp_truth[g->name]->uid == g->uid)
      continue;
    if(tmp_test.count(g->name) == 0){
      tmp_test[g->name] = g;
    }
    else if(tmp_test[g->name]->nb_words() < g->nb_words())
      tmp_test[g->name] = g;
  }

  for(auto it: tmp_truth)
    truth.push_back(it.second);

  for(auto it: tmp_test)
    test.push_back(it.second);
}

struct PrintFunc{

  std::string operator ()(sn::GraphOfWordData::node_type const& node){
    return "";
  }
  std::string operator ()(sn::GraphOfWordData::edge_type const& edge){
    return "";
  }

};

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
  }

  ROS_INFO("nb sequences loaded: %lu ", trackers.size());


  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.05, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("pfh", 0.04, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("size", 0.0025, sn::Distance(sn::euclidean_distance));
  dicos.set("color", 0.5, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("tbgr", 0.05, sn::Distance(sn::symmetric_chi2_distance));

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
//    std::stringstream graphname;
//    graphname << "/home/duceux/Desktop/results/gow_study/graphs/" << graph_ptr->name << graph_ptr->uid.toNSec()<< ".gv";
//    sn::graph::graphiz_save(*graph_ptr->graph, graphname.str(), PrintFunc(), PrintFunc());
    graphs.push_back(graph_ptr);
  }

/*
  for(auto g:graphs)
    for(auto b:graphs){
      std::stringstream graphname;
      graphname << "/home/duceux/Desktop/results/gow_study/graphs/" << g->name << g->uid.toNSec() << "-" << b->name << b->uid.toNSec() << ".gv";
      sn::graph::graphiz_save(sn::intersection(*g->graph,*b->graph), graphname.str(), PrintFunc(), PrintFunc());
    }
*/

  std::vector<sn::GraphOfWord::Ptr> gtruth;
  std::vector<sn::GraphOfWord::Ptr> gtest;
  split_truth_is_biggest(graphs, gtruth, gtest);

  std::string filename = "/home/duceux/Desktop/results/metric_study/score.txt";
  std::ofstream file(filename, std::ofstream::app);
  std::string folder = "/home/duceux/Desktop/results/metric_study/";
  file << "dico_laser\t" << dicos.size("laser") << std::endl;
  file << "dico_pfh\t" << dicos.size("pfh") << std::endl;
  file << "dico_color\t" << dicos.size("color") << std::endl;
  file << "dico_tbgr\t" << dicos.size("tbgr") << std::endl;
  file << "dico_size\t" << dicos.size("size") << std::endl;
  file << "th_laser\t" << dicos.thresholds["laser"] << std::endl;
  file << "th_pfh\t" << dicos.thresholds["pfh"] << std::endl;
  file << "th_color\t" << dicos.thresholds["color"] << std::endl;
  file << "th_tbgr\t" << dicos.thresholds["tbgr"] << std::endl;
  file << "th_size\t" << dicos.thresholds["size"] << std::endl;

  std::map<std::string, int> type_histo;
  for(auto g: graphs){
    auto cur = g->type_map();
    for(auto it: cur)
      type_histo[it.first]+=it.second;
  }
  for(auto it: type_histo){
    file << "average:\t" << it.first << "\t" << (double)it.second/graphs.size() << std::endl;
  }

  {
    std::cout << "intersection_size";
    auto gfunc = sn::comparison::intersection_size(sn::no_type());
    auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
    std::cout << sn::get_precision(matrix) << std::endl;
    sn::print(matrix);
    file << "intersection_size\t" << sn::get_precision(matrix) << "\t"
         << sn::get_precision_ignoring_missed(matrix)<< std::endl;
    sn::save_confusion_matrix(matrix, folder+"intersection_size.txt");
  }
  {
    std::cout << "intersection_union_size";
    auto gfunc = sn::comparison::intersection_union_size(sn::no_type());
    auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
    std::cout << sn::get_precision(matrix) << std::endl;
    sn::print(matrix);
    file << "intersection_union_size\t" << sn::get_precision(matrix)<< "\t"
         << sn::get_precision_ignoring_missed(matrix) << std::endl;
    sn::save_confusion_matrix(matrix, folder+"intersection_union_size.txt");
  }
  {
    std::cout << "max_component_size";
    auto gfunc = sn::comparison::max_component_size(sn::no_type());
    auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
    std::cout << sn::get_precision(matrix) << std::endl;
    sn::print(matrix);
    file << "max_component_size\t" << sn::get_precision(matrix) << "\t"
         << sn::get_precision_ignoring_missed(matrix)<< std::endl;
    sn::save_confusion_matrix(matrix, folder+"max_component_size.txt");
  }
  return 0;
}
