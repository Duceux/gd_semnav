#ifndef GRAPH_CLUSTERING_ENGINE_H
#define GRAPH_CLUSTERING_ENGINE_H

#include <sn_models/graph_of_word.h>
#include <sn_models/comparison.h>
#include <sn_graph_learning/classifier.h>
#include <sn_graph/wise_operators.hh>
#include <queue>

namespace sn {
namespace graph_learning {

// Merge functor
struct MergeFunctor{
  sn::GraphOfWordPtr operator()(sn::GraphOfWordPtr const& l, sn::GraphOfWordPtr const& r) const{
    return std::make_shared<sn::GraphOfWord>(sn::GraphOfWord::merge(*l, *r));
  }
};


// TYPE STUFF
typedef sn::GraphOfWord::Ptr model_t;
typedef sn::GraphOfWord GoW;
typedef sn::SimpleNode<model_t> node_t;
typedef sn::Edge<model_t, std::pair<double, double>, true> edge_t;
typedef sn::Edge<model_t, std::pair<double, double>, false> graphiz_edge_t;
typedef sn::Graph<node_t, edge_t> graph_t;
typedef sn::Graph<node_t, graphiz_edge_t> graphiz_graph_t;
typedef LearningMatrix<model_t, MergeFunctor> cluster_holder_t;
typedef Classifier2<cluster_holder_t> classifier_t;
typedef std::queue<model_t> queue_t;

// PRINTING STUFF
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


std::string print_node(const node_t& node){
  static std::map<std::string, Color> color_map;
  std::stringstream str;
  str << std::setprecision(2);
  str << "[ label = \"" <<  node.key->name
      << "\", color="<< color_map[node.key->name].toString() <<", style=filled" << " ];";
  return str.str();
}

template<typename E>
std::string print_edge(const E& e){
  std::stringstream str;
  str << std::setprecision(4);
  str  << "[ label = \"" << e.val.first << "\n" << e.val.second << "\" ];";
  return str.str();
}

template<typename T>
std::string print_nothing(const T&){
  return "";
}

template<typename G>
std::string print_graph(G const& graph) {
  std::stringstream str;
  str << "start = rand;\n";
  str << "overlap=false;\n";
  str << "splines=true;\n";
  str << "edge [fontsize=8];\n";
  str << "node [fontsize=10];\n";
  return str.str();
}

bool edge_sup(const edge_t& l, const edge_t& r) {
  if(l.val.first == r.val.first)
    return l.val.second > r.val.second;
  return l.val.first > r.val.first;
}

bool edge_inf(const edge_t& l, const edge_t& r) {
  if(l.val.first == r.val.first)
    return l.val.second < r.val.second;
  return l.val.first < r.val.first;
}


// POLICIES STUFF
struct MinEdgePol{
  bool operator()(const edge_t& l,
                  const edge_t& r)const{
    return edge_sup(l, r);
  }
};

struct MaxEdgePol{
  bool operator()(const edge_t& l,
                  const edge_t& r)const{
    return edge_inf(l, r);
  }
};


struct MinSizePol{
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    return v1.key->size()>v2.key->size();
  }
};

struct MinScorePol{
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    return v1.key->score>v2.key->score;
  }
};

struct MinVisPol{
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    return v1.key->visited>v2.key->visited;
  }
};

struct MinOutEdgePol{
  MinOutEdgePol(const graph_t& g):graph(g){}
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    if(graph.nb_out_edges_of(v1) == 0 && graph.nb_out_edges_of(v2) == 0)return false;
    if(graph.nb_out_edges_of(v1) == 0 && graph.nb_out_edges_of(v2) != 0)return false;
    if(graph.nb_out_edges_of(v1) != 0 && graph.nb_out_edges_of(v2) == 0)return true;
    edge_t l = best_out_edge(graph, v1.key, MinEdgePol());
    edge_t r = best_out_edge(graph, v2.key, MinEdgePol());
    return edge_sup(l, r);
  }
  graph_t graph;
};

struct LeafPol{
  LeafPol(const graph_t& g):graph(g){}
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    if(graph.nb_in_edges_of(v1) > graph.nb_in_edges_of(v2)) return true;
    if(graph.nb_in_edges_of(v1) < graph.nb_in_edges_of(v2)) return false;
    if(graph.nb_out_edges_of(v1) == 0 && graph.nb_out_edges_of(v2) == 0)return false;
    if(graph.nb_out_edges_of(v1) == 0 && graph.nb_out_edges_of(v2) != 0)return false;
    if(graph.nb_out_edges_of(v1) != 0 && graph.nb_out_edges_of(v2) == 0)return true;
    edge_t l = best_out_edge(graph, v1.key, MaxEdgePol());
    edge_t r = best_out_edge(graph, v2.key, MaxEdgePol());
    return edge_sup(l, r);
  }
  graph_t graph;
};

struct MinInEdgePol{
  MinInEdgePol(const graph_t& g):graph(g){}
  bool operator()(const node_t& v1,
                  const node_t& v2)const{
    if(graph.nb_in_edges_of(v1) == 0 && graph.nb_in_edges_of(v2) == 0)return false;
    if(graph.nb_in_edges_of(v1) == 0 && graph.nb_in_edges_of(v2) != 0)return false;
    if(graph.nb_in_edges_of(v1) != 0 && graph.nb_in_edges_of(v2) == 0)return true;
    edge_t l = best_out_edge(graph, v1.key, MinEdgePol());
    edge_t r = best_out_edge(graph, v2.key, MinEdgePol());
    return edge_sup(l, r);
  }
  graph_t graph;
};

class GraphClusterEngine {
private:
  int max_nodes;

public:
  GraphClusterEngine():
    holder_(MergeFunctor()),
    visited_th_(0)
  {
    max_nodes = 5;
  }

  std::string learn(model_t& model) {
    last_ = model;
    std::string res = "";
    model->visited++;

    // Classify
    if(holder_.size() > 2) {
      classifier_.classify(model, holder_);
      res = holder_[classifier_.front().id]->name;
      double size = model->size();
      double types = GraphOfWord::type_map(model->get_graph()).size();
      std::cout << res << " "
                << classifier_.front().mod/types << " "
                << classifier_.front().inter/size << "\n"
                << holder_[(++classifier_.begin())->id]->name << " "
                << (++classifier_.begin())->mod/types << " "
                << (++classifier_.begin())->inter/size << std::endl;
      model->score = classifier_.front().inter;

      /*
      double merge_sim = 1500;
      double min_mod = 18.0;
      if(classifier_.front().mod > min_mod && classifier_.front().inter > merge_sim)
        return res;
        */
    }

    if(holder_.size() > 2 && !model->labeled) {
      // First two subgraphs
      int first = classifier_.front().id;
      int second = (++classifier_.begin())->id;
      // Update graph
      UpdateGraph(model, clusters_[first]);
      UpdateGraph(model, clusters_[second]);

      // And try to collect singletons
      for(auto const& subg: clusters_)
        if(SizeCluster(subg) == 1)
          UpdateGraph(model, subg);

    }else
      UpdateGraph(model, complete_graph_);

    // Merge nodes
//    MergeModels();

    // Reduce clusters
    ReduceClusters();

    // Reduce graph
    ReduceSingletons();

    // Compute clusters
    ComputeClusters();

    return res;
  }


  std::pair<double, double> Compare(const model_t& l, const model_t& r) {
    static auto func = sn::comparison::joint_modalities_intersection(sn::no_type());
    static auto mod = sn::comparison::modalities_match(sn::no_type());


    if(l->labeled && r->labeled) {
      if(l->name == r->name)
        return std::pair<double, double>(100, 10000);
      else
        return std::pair<double, double>(0, 0);
    }

    std::pair<double, double> val;
    val.first = mod(l, r);
    val.second = func(l, r);

    return val;
  }

  int SizeCluster(const graph_t& g) {
    int size = 0;
    for(auto const& n: g.get_nodes())
      size+=n.key->merged;
    return size;
  }

  void UpdateGraph(const model_t& model, const graph_t& subg) {
    double min_edge_sim = 20.0;
//    double min_edge_sim = 200.0;
    double min_mod = 5.0;
//    double min_mod = 5.0;
    complete_graph_.insert_node(model);
    for(const auto& node: subg.get_nodes()){
      if(node.key->uid != model->uid){
        auto e = Compare(model, node);
        if(e.first >= min_mod && e.second >= min_edge_sim){
          complete_graph_.insert_edge(model, node, e);
        }
        e = Compare(node, model);
        if(e.first > min_mod && e.second > min_edge_sim){
          complete_graph_.insert_edge(node, model, e);
        }
      }
    }
    // reduce nodes
    for(const auto& node: subg.get_nodes())
      ReduceNode(node);

    ReduceNode(complete_graph_.at(model));
  }

  void ReduceSingletons() {
//    if(!queue_.empty())return;
    clusters_ = get_connected_components(complete_graph_);
    int max_single = 10;
    graph_t singletons;
    for(const graph_t& sg: clusters_){
      if(SizeCluster(sg) == 1 && !sg.get_nodes().begin()->key->labeled) {
        singletons.insert_node(*sg.get_nodes().begin());
      }
    }

    while(singletons.size() > max_single) {
      auto node = best_node(singletons, MinScorePol());
      singletons.remove(node);
      complete_graph_.remove(node);
    }
  }

  void ReduceNode(const node_t& key){
    double max_out_edge = max_nodes - 1;
    double max_in_edge = max_out_edge;
    while(complete_graph_.nb_out_edges_of(key) > max_out_edge){
      auto edge = best_out_edge(complete_graph_, key, MinEdgePol());
      complete_graph_.remove(edge);
    }
    while(complete_graph_.nb_in_edges_of(key) > max_in_edge)
      complete_graph_.remove(best_in_edge(complete_graph_, key, MinEdgePol()));
    assert(complete_graph_.nb_out_edges_of(key) <= max_out_edge);
    assert(complete_graph_.nb_in_edges_of(key) <= max_in_edge);
  }


  void ReduceClusters(){
    bool need_reducing = true;
    while (need_reducing) {
      clusters_ = get_connected_components(complete_graph_);
      need_reducing = false;
      for(auto subg: clusters_)
        need_reducing = need_reducing || ReduceCluster(subg);
    }
  }

  bool ReduceCluster(graph_t& subgrah){
    if(SizeCluster(subgrah) <= max_nodes) return false;

    auto e = best_edge(subgrah, MinEdgePol());
    subgrah.remove(e);
    complete_graph_.remove(e);

    return true;
    /*
    while(SizeCluster(subgrah) > max_nodes && subgrah.nb_nodes() > 1){
      auto n = best_node(subgrah, LeafPol(subgrah));
      subgrah.remove(n);
      complete_graph_.detach(n);
      //      push(node.key);
    }
    */
    /*
    for(auto const& n: subgrah.get_nodes()) {
      UpdateGraph(n);
    }
    */
  }



  void ComputeClusters(){
    clusters_ = get_connected_components(complete_graph_);
    holder_.clear();
    for(int i=0; i<clusters_.size(); ++i) {
      if(clusters_[i].size() == 0)continue;
      const graph_t& cluster = clusters_[i];
      auto it = cluster.get_nodes().begin();
      auto itEnd = cluster.get_nodes().end();
      holder_.new_cluster(*it);
      ++it;
      for(;it!=itEnd; ++it) {
        holder_.add_to_cluster(i, *it);
      }
    }
  }

  void MergeModels() {
    double merge_sim = 1900;
    double min_mod = 16.0;
    bool needs_update = true;
    while(needs_update) {
      needs_update = false;
      model_t parent;
      model_t child;
      for(const edge_t& e: complete_graph_.get_edges()) {
        if(e.val.first >= min_mod && e.val.second >= merge_sim)
          if(e.parent->labeled == e.child->labeled)
          {
            needs_update = true;
            parent = e.parent;
            child = e.child;
            break;
          }
      }
      if(needs_update) {
        complete_graph_.remove(parent);
        complete_graph_.remove(child);
        model_t newm(new GoW(GoW::merge(*parent, *child)));
        UpdateGraph(newm, complete_graph_);
        //        push(newm);
      }
    }
  }

  size_t nb_models() { return complete_graph_.nb_nodes(); }
  size_t nb_merging() {
    size_t count = 0;
    for(const node_t& m: complete_graph_.get_nodes())
      count += m.key->merged-1;
    return count;
  }
  size_t nb_edges() { return complete_graph_.nb_edges(); }
  size_t nb_clusters() { return clusters_.size(); }

  // PRINTING STUFF
  void save_complete_graph(const std::string& filename) {
    // cvt graph to prettier
    graphiz_graph_t to_save;
    for(auto const& n: complete_graph_.get_nodes())
      to_save.insert(n);
    for(auto const& e: complete_graph_.get_edges())
      to_save.insert_edge(e.parent, e.child, e.val);


    std::ofstream file(filename);
    file << std::setprecision(2);
    file << std::fixed;
    sn::graph::graphiz_save(to_save, file, print_node, print_edge<graphiz_edge_t>, print_graph<graphiz_graph_t>);
    file.close();
  }

  // Getters
  const classifier_t& classifier() const { return classifier_; }
  const cluster_holder_t& clusters() const { return holder_; }

  // Queue
  void push(const model_t& m) {
    if(m->visited <= visited_th_)
      queue_.push(m);
  }

  int queue_size() const {
    return queue_.size();
  }

  model_t pop() {
    model_t r = queue_.front();
    queue_.pop();
    while(r->visited > visited_th_){
      r = queue_.front();
      queue_.pop();
    }
    return r;
  }

private:
  graph_t complete_graph_;
  std::vector<graph_t> clusters_;
  cluster_holder_t holder_;
  classifier_t classifier_;
  queue_t queue_;
  model_t last_;
  int visited_th_;
};

}
}

#endif // GRAPH_CLUSTERING_ENGINE_H
