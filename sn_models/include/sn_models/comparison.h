#ifndef COMPARISON_H
#define COMPARISON_H

#include <sn_models/functor.h>
#include <memory>

namespace sn{
namespace comparison {

const double NUMBER_OF_MODALITIES = 20;

struct base{
  std::shared_ptr<no_type> m_filter;
  template<typename F>
  base(F const& f):m_filter(new functor_wrapper<F>(f)){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const = 0;

};


struct intersection_size: base{
  template<typename F>
  intersection_size(F const& f):base(f){}

  GraphOfWordData intersection(const GraphOfWord::Ptr &l,
                               const GraphOfWord::Ptr &r)const{
    return filter(graph::intersection(*l->graph, *r->graph), *m_filter, *m_filter);
  }

  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return intersection(l, r).size();
  }

  template<class Container>
  void setModels(Container const& m){
  }

};

struct intersection_union_size: public intersection_size{
  template<typename F>
  intersection_union_size(F const& f):intersection_size(f){}

  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    auto gunion = filter(graph::addition(*l->graph, *r->graph),*m_filter, *m_filter);
    return (double)res.size()/gunion.size();
  }
};

struct edge_intersection: public intersection_size{
  template<typename F>
  edge_intersection(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return intersection(l, r).nb_edges();
  }
};


struct node_intersection: public intersection_size{
  template<typename F>
  node_intersection(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return intersection(l, r).nb_nodes();
  }
};

struct max_component_size: public intersection_size{
  template<typename F>
  max_component_size(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return get_max_connected_components(intersection(l, r)).size();
  }
};

struct inclusion: public intersection_size{
  template<typename F>
  inclusion(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return (double)intersection(l, r).size()/l->size();
  }
};

struct modalities_match: public intersection_size{
  template<typename F>
  modalities_match(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    return (double)GraphOfWord::type_map(res).size();
  }
};

struct joint_modalities_intersection: public intersection_size{
  template<typename F>
  joint_modalities_intersection(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    return GraphOfWord::type_map(res).size()*res.size();
  }
};

struct normalized_joint: public intersection_size{
  template<typename F>
  normalized_joint(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    double type_inc = (double)GraphOfWord::type_map(res).size()/NUMBER_OF_MODALITIES;
    double inc = (double)res.size()/l->size();
    return type_inc*inc;
  }
};

struct per_modalities_inclusion: public intersection_size{
  template<typename F>
  per_modalities_inclusion(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    auto map = GraphOfWord::type_map(res);
    auto size_map = GraphOfWord::type_map(*l->graph);
    double sum = 0;
    for(auto it:map)
      sum+=(double)it.second/size_map[it.first];
    return sum;
  }
};

struct tf_idf: public intersection_size{
  template<typename F>
  tf_idf(F const& f, std::vector<GraphOfWordPtr> const& ref):intersection_size(f){
    setModels(ref);
  }
  template<typename F>
  tf_idf(F const& f):intersection_size(f){}


  double get_idf(edge_t const& e)const{
    return std::log(N/(double)edge_idf.at(e).size());
  }
  double get_idf(node_t const& e)const{
    return std::log(N/(double)node_idf.at(e).size());
  }

  int N;
  std::unordered_map<edge_t, std::set<ros::Time>, HashEdge<edge_t>> edge_idf;
  std::unordered_map<node_t, std::set<ros::Time>, HashNode<node_t>> node_idf;


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    double score = 0;
    for(node_t const& node: res.get_nodes())
      score += get_idf(node);
    for(edge_t const& edge: res.get_edges())
      score += get_idf(edge);
    return score;
  }

  template<class Container>
  void setModels(const Container &ref){
    N = ref.size();
    for(GraphOfWordPtr const& model: ref){
      for(node_t const& node: model->graph->get_nodes())
        node_idf[node].insert(model->uid);
      for(edge_t const& edge: model->graph->get_edges())
        edge_idf[edge].insert(model->uid);
    }
  }

};

struct consensus: public intersection_size{
  template<typename F>
  consensus(F const& f, std::vector<GraphOfWordPtr> const& ref):intersection_size(f){
    models = ref;
  }
  template<typename F>
  consensus(F const& f):intersection_size(f){}

  template<class Container>
  void setModels(const Container &ref){
    models.clear();
    for(GraphOfWordPtr const& model: ref)
      models.push_back(model);
  }

  std::vector<GraphOfWordPtr> models;

  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    std::map<ros::Time, std::map<std::string, double>> mat;
    std::map<std::string, double> norm;
    for(GraphOfWordPtr const& model: models){
      GraphOfWordData inter = intersection(l, model);
      auto map = GraphOfWord::type_map(inter);
      for(auto it: map){
        mat[model->uid][it.first] = it.second;
        norm[it.first]+=it.second*it.second;
      }
    }
    for(auto it: norm){
      norm[it.first] = std::sqrt(it.second);
    }
    double score = 0;
    if(mat.count(r->uid) == 0){
      return 0.0;
    }
    for(auto it: mat.at(r->uid)){
      score += it.second/norm[it.first];
    }
    return score;
  }

};


}

}

#endif // COMPARISON_H
