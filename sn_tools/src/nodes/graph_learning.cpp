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
#include <sn_tools/io.h>
#include <sn_tools/stringstream.h>
#include <sn_graph_learning/graph_clustering_engine.h>

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;
namespace fs = boost::filesystem;
namespace sngl = sn::graph_learning;

template<typename ClusterHolder>
int corruption(ClusterHolder const& holder){
  int total = 0;
  for(auto proto: holder)
    if(proto->name == "Unknown")
      total++;
  return total;
}

template<typename ClusterHolder>
int solid(ClusterHolder const& holder){
  int total = 0;
  for(auto proto: holder)
    if(proto->name != "Unknown" && proto->merged > 1)
      total++;
  return total;
}

template<typename ClusterHolder>
int duplicate(ClusterHolder const& holder){
  std::map<std::string, int> dup_map;
  for(auto proto: holder)
    if(proto->name != "Unknown")
      dup_map[proto->name]++;
  int total = 0;
  for(auto it: dup_map)
    total+=it.second-1;
  return total;
}

template<typename ClusterHolder>
int singleton(ClusterHolder const& holder){
  int total = 0;
  for(auto proto: holder)
    if(proto->merged == 1)
      total++;
  return total;
}

template<typename ClusterHolder>
int category(ClusterHolder const& holder){
  std::map<std::string, int> dup_map;
  for(auto proto: holder)
    if(proto->name != "Unknown")
      dup_map[proto->name]++;
  return dup_map.size();
}


struct lessfunc{
  bool operator()(sn::GraphOfWordPtr const& l, sn::GraphOfWordPtr const& r){
    return l->name < r->name;
  }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "full_reco");

  VSeq trackers;

  {
    typedef std::set<fs::path> result_set_t;
    result_set_t result_set;
    std::string source;
    ros::param::param<std::string>("~source_dir", source,
                                   "/home/duceux/Desktop/phd-dataset/trackers/");
    fs::path sourceDir(source);
    fs::directory_iterator end_iter;


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

    for(fs::path it: result_set){
      sn::tools::load(source+it.string(), trackers, "sequence");
      std::cout << "reading: " << it.filename().string() << std::endl;
      std::string filename = it.filename().string();
    }

    ROS_INFO("nb sequences loaded: %lu ", trackers.size());
  }

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
  if(descriptors.size() > 2000)
    descriptors.resize(2000);
  for(auto des: descriptors){
    dicos.get(des);
  }

  sngl::GraphClusterEngine engine;
  {
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
    for(const auto& g: graphs)
      engine.push(g);
    trackers.clear();
    graphs.clear();
  }
  std::string folder = "/home/robotic/Desktop/results/graph-clustering/";


  std::ofstream seqfile(sn::StringStream() << folder << "seqres"  << ".txt");
  std::set<sn::GraphOfWordPtr, lessfunc> reality_check;

  unsigned count = 0;
  double error = 0;
  double total = 0;
  while(engine.queue_size() > 0) {
    auto candidate = engine.pop();
    //    if(count > 20) break;
    std::cout << ++count << std::endl;
    std::cout << "candidate\n";
    candidate->print();
    std::string result = engine.learn(candidate);

    auto const& classifier = engine.classifier();
    auto const& clusters = engine.clusters();

    reality_check.insert(candidate);
    total++;

    if(result != candidate->name)
      error++;

    std::cout << "total queries: " << total << std::endl;
    std::cout << "nb clusters: " << clusters.size() << std::endl;
    std::cout << "solid: " << solid(clusters) << std::endl;
    std::cout << "corruption: " << corruption(clusters) << std::endl;
    std::cout << "real nb: " << reality_check.size() << std::endl;
    std::cout << "error rate: " << (error-reality_check.size()) << std::endl;
    std::cout << "duplicate: " << duplicate(clusters) << std::endl;
    std::cout << "singleton: " << singleton(clusters) << std::endl;
    std::cout << "label: " << category(clusters) << std::endl;
    std::cout << "nb_models: " << engine.nb_models() << std::endl;

    seqfile << total << "\t"
            << clusters.size() << "\t"
            << solid(clusters) << "\t"
            << corruption(clusters)  << "\t"
            << reality_check.size() << "\t"
            << (error-reality_check.size()) << "\t"
            << duplicate(clusters) << "\t"
            << singleton(clusters) << "\t"
            << category(clusters) << "\t"
            << engine.nb_models() << "\t";
    seqfile << std::endl;

    std::stringstream str;
    str << folder << "step/" << std::setw(5) << std::setfill('0') << total << ".gv";
    engine.save_complete_graph(str.str());
  }
  seqfile.close();

  std::stringstream str;
  str << folder << "complete.gv";
  engine.save_complete_graph(str.str());

  // produce learning matrix
  auto clusters = engine.clusters();
  std::ofstream learningmatfile(sn::StringStream() << folder << "learningmat" << ".txt");
  clusters.rearrange_cluters();
  std::map<std::string, std::map<int, int>> matrix;
  const auto& C = clusters.get_clusters();
  for(unsigned i=0; i<C.size(); ++i){
    for(auto it: C[i])
      matrix[it.first][i] = it.second;
  }

  // print learning matrix
  unsigned label = 0;
  for(auto it: matrix){
    for(unsigned i=0; i<C.size(); ++i)
      learningmatfile << label << " "
                      << i << " "
                      << it.second[i] << std::endl;
    label++;
    learningmatfile << std::endl;
  }

  return 0;
}
