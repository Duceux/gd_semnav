#ifndef COMPARISON_H
#define COMPARISON_H

#include <sn_models/functor.h>
#include <memory>

namespace sn{
namespace comparison {

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

};

struct intersection_union_size: public intersection_size{
  template<typename F>
  intersection_union_size(F const& f):intersection_size(f){}

  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(l, r);
    auto gunion = graph::addition(filter(*l->graph, *m_filter, *m_filter),
                           filter(*r->graph, *m_filter, *m_filter));
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
    return (double)intersection(l, r).size()/r->size();
  }
};

struct modalities_match: public intersection_size{
  template<typename F>
  modalities_match(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return GraphOfWord::type_map(intersection(l, r)).size();
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



}

}

#endif // COMPARISON_H
