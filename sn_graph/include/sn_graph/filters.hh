#ifndef FILTERS_HH
#define FILTERS_HH

#include <sn_graph/graph.hh>

namespace sn{

template<typename N, typename E, typename LetN, typename LetE>
Graph<N,E> filter(Graph<N,E> const& g, LetN const& ln, LetE const& le){
  Graph<N,E> res;
  for(N n: g.get_nodes())
    if(ln(n))
      res.insert(n);
  for(E e: g.get_edges())
    if(le(e)){
      res.insert(g.at(e.parent));
      res.insert(g.at(e.child));
      res.insert(e);
    }
  return res;
}

}

#endif // FILTERS_HH
