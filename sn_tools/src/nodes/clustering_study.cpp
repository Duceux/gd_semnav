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

struct Color{
  float r, g, b;

  Color(){
    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> distribution(0.2, 1.0);

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
sn::Edge<sn::GraphOfWordPtr, double, false>> MyGraph;


std::string print_node(const typename MyGraph::node_type& node){
  static std::map<std::string, Color> color_map;
  std::stringstream str;
  str << "[ label = \"" <<  node.key->name
      << "\", color="<< color_map[node.key->name].toString() <<", style=filled" << " ];";
  return str.str();
}


std::string print_edge(const typename MyGraph::edge_type& e){
  std::stringstream str;
  str  << "[ label = \"" <<  e.val << "\" ];\n";
  return str.str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "full_reco");

  std::string source;
  ros::param::param<std::string>("~source_dir", source,
                                 "/home/robotic/Desktop/new_phd_dataset/sequence/");
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
    std::string filename = it.filename().string();
  }

  ROS_INFO("nb sequences loaded: %lu ", trackers.size());

  //  for(int iter=0; iter<1; ++iter){
  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.1, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("pfh", 0.05, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("size", 0.01, sn::Distance(sn::euclidean_distance));
  dicos.set("color", 1.5, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("tbgr", 0.1, sn::Distance(sn::symmetric_chi2_distance));

  /*
  sn::Dictionary<sn::FastGetter> dicos;
  dicos.set("laser", 0.05, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("pfh", 0.04, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("size", 0.0025, sn::Distance(sn::euclidean_distance));
  dicos.set("color", 0.5, sn::Distance(sn::symmetric_chi2_distance));
  dicos.set("tbgr", 0.05, sn::Distance(sn::symmetric_chi2_distance));
*/
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

  int count = 0;
  for(auto candidate: graphs){
    graph_of_models.insert_node(candidate);
    //    candidate->print();
    count++;
  }
  static auto func = sn::comparison::joint_modalities_intersection(sn::no_type());
//  graph_of_models = sn::MaximumSpanningTree(graph_of_models, func, true);
/*

  // Remove edges
  MyGraph tmp;
  for(const auto& node: graph_of_models.get_nodes())
    tmp.insert(node);
  for(const auto& edge: graph_of_models.get_edges()) {
    MyGraph::edge_type e = edge;
    e.val/=std::max(edge.parent->type_map().size(), edge.child->type_map().size());
    e.val/=std::min(edge.parent->size(), edge.child->size());
    tmp.insert(e);
  }
  graph_of_models = tmp;
*/


    {
      int nb_edg = 0;
      double min_sim = 0;
      int nb_mod = 0;
      int nb_modl = 0;
      int nb_cat = 0;
      int nb_clu = 0;
      int nb_cor = 0;
      int nb_dup = 0;
      int nb_dis = 0;
      int nb_sing = 0;
      int nb_good = 0;

      //params
      nb_edg = 2;
      min_sim = 10;

      // calculations
      // big brother
      //    static sn::GraphOfWord::Ptr big_brother(new sn::GraphOfWord);
      //    double inclusion = func(candidate, big_brother);
      //    std::cout << inclusion << std::endl;
      //    big_brother->get_graph() = sn::graph::addition(big_brother->get_graph(),
      //                                                   candidate->get_graph());

      // clusters
      graph_of_models = sn::MaximumOutEdgeGraph(graph_of_models,
                                                func, nb_edg, min_sim);

      // reduce clusters


      // reduce graph

      //truth
      nb_mod = graphs.size();
      std::set<std::string> cat_set;
      for(auto candidate: graphs){
        cat_set.insert(candidate->name);
      }
      nb_cat = cat_set.size();

      // results
      auto components = sn::get_connected_components(graph_of_models);
      nb_clu = components.size();
      std::map<std::string, int> dup_map;
      for(auto const& subg: components){
        if(subg.nb_nodes() == 1)
          nb_sing+=1;
        int corrompu = 0;
        std::string name = subg.get_nodes().begin()->key->name;
        for(auto const& model: subg.get_nodes())
          if(name != model.key->name){
            corrompu = 1;
            break;
          }

        nb_cor += corrompu;
        if(corrompu == 0){
          dup_map[name]++;
          if(subg.nb_nodes() > 1)
            nb_good++;
        }
      }
      for(auto it: dup_map)
        nb_dup+=it.second-1;
      nb_dis = dup_map.size();
      nb_modl = graph_of_models.nb_nodes();
      std::ofstream results("/home/robotic/Desktop/results/clustering_study.txt", std::ofstream::app);
      results << nb_edg << "\t"
              << min_sim << "\t"
              << nb_mod << "\t"
              << nb_cat << "\t"
              << nb_modl << "\t"
              << nb_clu << "\t"
              << nb_cor << "\t"
              << nb_dup << "\t"
              << nb_dis << "\t"
              << nb_sing << "\t"
              << nb_good << "\t"
              << std::endl;
      results.close();

      std::cout << "nb_edg" << "\t"
                << "min_sim" << "\t"
                << "nb_mod" << "\t"
                << "nb_cat" << "\t"
                << "nb_modl" << "\t"
                << "nb_clu" << "\t"
                << "nb_cor" << "\t"
                << "nb_dup" << "\t"
                << "nb_dis" << "\t"
                << "nb_sing" << "\t"
                << "nb_good" << "\t"
                << std::endl;

      std::cout << nb_edg << "\t"
                << min_sim << "\t"
                << nb_mod << "\t"
                << nb_cat << "\t"
                << nb_modl << "\t"
                << nb_clu << "\t"
                << nb_cor << "\t"
                << nb_dup << "\t"
                << nb_dis << "\t"
                << nb_sing << "\t"
                << nb_good << "\t"
                << std::endl;

//      std::stringstream str;
//      str << "/home/robotic/Desktop/results/graphs/"<< iter << th << ".gv";
//      sn::graph::graphiz_save(graph_of_models, str.str(), print_node, print_edge, sn::graph::SimplePrintGraph());


    }
//  }

  // Extra cluster similarities
  unsigned extra_nb = 0;
  double extra_min = std::numeric_limits<double>::max();
  double extra_max = 0;
  double extra_mean = 0;
  double extra_var = 0;
  unsigned intra_nb = 0;
  double intra_min = std::numeric_limits<double>::max();
  double intra_max = 0;
  double intra_mean = 0;
  double intra_var = 0;
  for(auto edge: graph_of_models.get_edges()) {
    if(edge.parent->name != edge.child->name){
      std::cout << edge.parent->name << " " << edge.child->name << std::endl;
      extra_nb++;
      extra_min = std::min(extra_min, edge.val);
      extra_max = std::max(extra_max, edge.val);
      extra_mean+=edge.val;
      extra_var+= edge.val*edge.val;
    }
    else{
      intra_nb++;
      intra_min = std::min(intra_min, edge.val);
      intra_max = std::max(intra_max, edge.val);
      intra_mean+=edge.val;
      intra_var+= edge.val*edge.val;
    }
  }
  extra_mean = extra_mean/extra_nb;
  extra_var = std::sqrt(extra_var/extra_nb - extra_mean*extra_mean);
  intra_mean = intra_mean/intra_nb;
  intra_var = std::sqrt(intra_var/intra_nb - intra_mean*intra_mean);
  std::cout << "Extra: " << extra_nb << " "
            << extra_min << " "
            << extra_mean << " "
            << extra_max << " "
            << extra_var << std::endl;
  std::cout << "Intra: " << intra_nb << " "
            << intra_min << " "
            << intra_mean << " "
            << intra_max << " "
            << intra_var << std::endl;

  std::stringstream str;
  str << "/home/robotic/Desktop/results/graph-clustering/final.gv";
  sn::graph::graphiz_save(graph_of_models, str.str(), print_node, print_edge, sn::graph::SimplePrintGraph());

  return 0;
}
