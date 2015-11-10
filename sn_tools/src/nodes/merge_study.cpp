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
#include <sn_tools/io.h>

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;
typedef sn::GraphOfWord::Ptr GoW;
typedef std::vector<sn::GraphOfWord::Ptr> GraphPtrVector;

void split_truth(GraphPtrVector const& graphs,
                 GraphPtrVector& truth, GraphPtrVector& test, int nb)
{
  std::map<std::string, int> count_map;
  std::map<std::string, std::vector<GoW>> gow_map;

  truth.clear();
  test.clear();

  for(auto gow: graphs){
    int count = count_map[gow->name];
    if(count < nb){
      count_map[gow->name]++;
      gow_map[gow->name].push_back(gow);
    }
    else
      test.push_back(gow);
  }

  for(auto it: gow_map){
    GoW base(new sn::GraphOfWord(*it.second.front()));
    for(auto g: it.second)
      *base = sn::GraphOfWord::merge(*base, *g);
    truth.push_back(base);
  }

}

void split_truth2(GraphPtrVector const& graphs,
                 GraphPtrVector& truth, GraphPtrVector& test, int nb)
{
  std::map<std::string, int> count_map;

  truth.clear();
  test.clear();

  for(auto gow: graphs){
    int count = count_map[gow->name];
    if(count < nb){
      count_map[gow->name]++;
      truth.push_back(gow);
    }
    else
      test.push_back(gow);
  }
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
    sn::tools::load(source+it.string(), trackers, "sequence");
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


  GraphPtrVector graphs;
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

  std::string filename = "/home/robotic/Desktop/results/merge_study/score.txt";
  std::ofstream file(filename, std::ofstream::app);
  std::string folder = "/home/robotic/Desktop/results/merge_study/";
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

  for(int i=1; i<=5; ++i){

    GraphPtrVector gtruth;
    GraphPtrVector gtest;
    split_truth2(graphs, gtruth, gtest, i);

    {
      std::cout << "tf_idf" << std::endl;
      auto gfunc = sn::comparison::tf_idf(sn::no_type(), gtruth);
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "tf_idf"<<"_not_"<< "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "tf_idf_" <<"_not_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "consensus" << std::endl;
      auto gfunc = sn::comparison::consensus(sn::no_type(), gtruth);
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "consensus" <<"_not_"<< "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "consensus_" <<"_not_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "inclusion" << std::endl;
      auto gfunc = sn::comparison::inclusion(sn::no_type());
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "inclusion" <<"_not_" << "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "inclusion_" <<"_not_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "normalized_joint" << std::endl;
      auto gfunc = sn::comparison::normalized_joint(sn::no_type());
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "normalized_joint" <<"_not_" << "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix) << std::endl;
      std::stringstream str;
      str << folder << "normalized_joint_" <<"_not_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
  }


  for(int i=1; i<=5; ++i){

    GraphPtrVector gtruth;
    GraphPtrVector gtest;
    split_truth(graphs, gtruth, gtest, i);

    {
      std::cout << "tf_idf" << std::endl;
      auto gfunc = sn::comparison::tf_idf(sn::no_type(), gtruth);
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "tf_idf"<< "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "tf_idf_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "consensus" << std::endl;
      auto gfunc = sn::comparison::consensus(sn::no_type(), gtruth);
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "consensus"<< "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "consensus_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "inclusion" << std::endl;
      auto gfunc = sn::comparison::inclusion(sn::no_type());
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "inclusion" << "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "inclusion_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
    {
      std::cout << "normalized_joint" << std::endl;
      auto gfunc = sn::comparison::normalized_joint(sn::no_type());
      auto matrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);
      file << i << " " << "normalized_joint" << "\t" << sn::get_precision(matrix) << "\t"
           << sn::get_precision_ignoring_missed(matrix)<< std::endl;
      std::stringstream str;
      str << folder << "normalized_joint_" << i << ".txt";
      sn::save_confusion_matrix(matrix, str.str());
    }
  }

  return 0;
}
