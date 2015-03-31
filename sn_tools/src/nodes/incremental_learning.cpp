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

template<typename ClusterHolder>
int corruption(ClusterHolder const& holder){
  int total = 0;
  for(auto proto: holder)
    if(proto->name == "Unknown")
      total++;
  return total;
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


  // SEQUENTIAL ENGINE
  {
    auto mergefunc =  [](sn::GraphOfWordPtr const& l, sn::GraphOfWordPtr const& r){
      return std::make_shared<sn::GraphOfWord>(sn::GraphOfWord::merge(*l, *r));
    };

    struct lessfunc{
      bool operator()(sn::GraphOfWordPtr const& l, sn::GraphOfWordPtr const& r){
        return l->name < r->name;
      }
    };
    std::set<sn::GraphOfWordPtr, lessfunc> reality_check;
    sngl::VectorHolder<sn::GraphOfWordPtr, decltype(mergefunc)> clusters(mergefunc);
    auto comfunc = sn::comparison::intersection_union_size(sn::no_type());
    typedef sngl::Classifier<decltype(clusters), decltype(comfunc)> MyClassifier;
    MyClassifier classifier(comfunc);
    typedef sngl::SequentialClustering<MyClassifier> MyClustering;
    MyClustering clustering(0.1);
    double error = 0;
    double total = 0;

    std::ofstream seqfile("/home/robotic/Desktop/seqres.txt");
    std::cout <<"Nb of models: " << graphs.size() << std::endl;
    for(auto candidate: graphs){
      std::cout << "candidate\n";
      candidate->print();
      reality_check.insert(candidate);
      total++;

      classifier.classify(candidate, clusters);
      clustering.cluster(classifier, clusters, candidate);

      if(classifier.size() > 0)
        if(clusters[classifier.front().second]->name != candidate->name)
          error++;


      std::cout << "nb clusters: " << clusters.size() << std::endl;
      std::cout << "corruption: " << corruption(clusters) << std::endl;
      std::cout << "real nb: " << reality_check.size() << std::endl;
      std::cout << "error rate: " << error/total << std::endl;
      if(classifier.size() > 0)
        std::cout << "score: " << classifier.front().first << std::endl;

      seqfile << clusters.size() << "\t"
              << corruption(clusters)  << "\t"
              << reality_check.size() << "\t"
              << error/total << "\t";
      if(classifier.size() > 0)
        seqfile << classifier.front().first << "\t";
      else
        seqfile << 0.0 << "\t";
      seqfile << std::endl;
    }
    seqfile.close();
  }
  return 0;
}
