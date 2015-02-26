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
                            const GraphOfWord::Ptr &r)const{return 0.0;}
};


struct intersection_size: public base{
  template<typename F>
  intersection_size(F const& f):base(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(filter(*l->graph, *m_filter, *m_filter),
                            filter(*r->graph, *m_filter, *m_filter));
    return res.size();
  }
};

struct intersection_union_size: public intersection_size{
  template<typename F>
  intersection_union_size(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(filter(*l->graph, *m_filter, *m_filter),
                            filter(*r->graph, *m_filter, *m_filter));
    auto gunion = addition(filter(*l->graph, *m_filter, *m_filter),
                           filter(*r->graph, *m_filter, *m_filter));
    return (double)res.size()/gunion.size();
  }
};

struct edge_intersection: public intersection_size{
  template<typename F>
  edge_intersection(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(filter(*l->graph, *m_filter, *m_filter),
                            filter(*r->graph, *m_filter, *m_filter));
    return res.nb_edges();
  }
};


struct node_intersection: public intersection_size{
  template<typename F>
  node_intersection(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    auto res = intersection(filter(*l->graph, *m_filter, *m_filter),
                            filter(*r->graph, *m_filter, *m_filter));
    return res.nb_nodes();
  }
};

struct max_component_size: public intersection_size{
  template<typename F>
  max_component_size(F const& f):intersection_size(f){}


  virtual double operator()(const GraphOfWord::Ptr &l,
                            const GraphOfWord::Ptr &r)const{
    return get_max_connected_components(intersection(filter(*l->graph, *m_filter, *m_filter),
                        filter(*r->graph, *m_filter, *m_filter))).size();
  }
};


}

}

#endif // COMPARISON_H
