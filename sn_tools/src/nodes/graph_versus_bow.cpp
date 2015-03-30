
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

//  for(double threshold=0.1; threshold<=0.4; threshold+=0.01)
  double threshold = 0.1;
  {

    double mean_score = 0;
    double mean_gscore = 0;
    double mean_size = 0;
    double mean_size_dico = 0;
    sn::ConfusionMatrix total;
    sn::ConfusionMatrix gtotal;

    for(int i=0; i<10; i++){

      sn::Dictionary<sn::FastGetter> dicos;
      dicos.set("laser", threshold, sn::Distance(sn::symmetric_chi2_distance));
      dicos.set("pfh", 0.05, sn::Distance(sn::symmetric_chi2_distance));
      dicos.set("size", 0.01, sn::Distance(sn::euclidean_distance));
      dicos.set("color", 1.5, sn::Distance(sn::symmetric_chi2_distance));
      dicos.set("tbgr", 0.1, sn::Distance(sn::symmetric_chi2_distance));

      std::vector<Descriptor> descriptors;
      descriptors.reserve(100000);
      for(auto& it: trackers)
        for(auto& des: it->descriptors){
          if(des.type != "laser")
            continue;
          else
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
      std::cout << descriptors.size() << std::endl;
      std::cout << dicos.size("laser") << std::endl;


      std::vector<sn::GraphOfWord::Ptr> graphs;
      for(sn_msgs::DescriptorSequencePtr seq_ptr: trackers){
        auto graph_ptr = sn::GraphOfWord::create();
        graph_ptr->name = seq_ptr->name;
        graph_ptr->uid = seq_ptr->uid;
        for(sn::descriptor_t des: seq_ptr->descriptors){
          if(des.type == "laser")
            graph_ptr->add(dicos.get(des));
        }
        graph_ptr->print();
        graphs.push_back(graph_ptr);
      }

      std::vector<sn::GraphOfWord::Ptr> gtruth;
      std::vector<sn::GraphOfWord::Ptr> gtest;
      split_truth_is_biggest(graphs, gtruth, gtest);


      double mean_sizel = 0.0;
      std::vector<sn::BagOfWord::Ptr> bows;
      for(sn_msgs::DescriptorSequencePtr seq_ptr: trackers){
        auto graph_ptr = sn::BagOfWord::create();
        graph_ptr->name = seq_ptr->name;
        graph_ptr->uid = seq_ptr->uid;
        for(sn::descriptor_t des: seq_ptr->descriptors){
          if(des.type == "laser")
            graph_ptr->add(dicos.get(des));
        }
        graph_ptr->print();
        mean_sizel += graph_ptr->nb_words();
        bows.push_back(graph_ptr);
      }
      mean_sizel /= graphs.size();

      std::vector<sn::BagOfWord::Ptr> truth;
      std::vector<sn::BagOfWord::Ptr> test;
      split_truth_is_biggest(bows, truth, test);


      std::cout << "Bow Intersection\n";
      auto func = sn::BowIntersection();
      auto matrix = sn::confusion_matrix(truth, test, func);
      std::cout << sn::get_precision(matrix) << std::endl;
      sn::print(matrix);



      std::cout << "Graph Intersection\n";
      auto gfunc = sn::comparison::intersection_size(sn::no_type());
      auto gmatrix = sn::confusion_matrix(gtruth, gtest, gfunc);
      std::cout << sn::get_precision(gmatrix) << std::endl;
      sn::print(gmatrix);

      std::string filename = "/home/duceux/Desktop/results/bow_study/bvsg.txt";
      std::ofstream file(filename, std::ofstream::app);
      file << dicos.size("laser") << "\t"
           << dicos.thresholds["laser"] << "\t"
           << sn::get_precision(matrix) <<  "\t"
           << sn::get_precision(gmatrix) << "\t"
           << mean_sizel << std::endl;


      mean_score += sn::get_precision(matrix);
      mean_gscore += sn::get_precision(gmatrix);
      mean_size += mean_sizel;
      mean_size_dico += dicos.size("laser");
      total += matrix;
      gtotal += gmatrix;
    }

    std::string filename = "/home/duceux/Desktop/results/bow_study/gstat2.txt";
    std::ofstream file(filename, std::ofstream::app);
    file << threshold << "\t"
         << sn::get_precision_ignoring_missed(total) << "\t"
         << sn::get_precision_ignoring_missed(gtotal) << "\t"
         << mean_size/5 <<  "\t"
         << mean_size_dico/5 << std::endl;
    std::stringstream mfile;
    mfile << "/home/duceux/Desktop/results/bow_study/matrix_bow.txt";
    sn::save_confusion_matrix(total, mfile.str());
    std::stringstream mfile2;
    mfile2 << "/home/duceux/Desktop/results/bow_study/gmatrix_gow.txt";
    sn::save_confusion_matrix(gtotal, mfile2.str());
  }
  return 0;
}
