#ifndef BEST_SPANNING_TREE_HH
#define BEST_SPANNING_TREE_HH

#include <sn_graph/graph.hh>

namespace sn {

template <typename Graph, typename DistF>
Graph MaximumSpanningTree(const Graph& input, DistF func, bool sym=true){
  Graph output;

  std::set<typename Graph::key_type> visited;

  for(auto n1: input.get_nodes()){
    if(visited.count(n1.key)>0)
      continue;
    visited.insert(n1.key);
    output.insert(n1);

    auto argb = input.get_nodes().begin();
    double best = -1.0;
    bool dir = true;
    for(auto it=input.get_nodes().begin();
        it!=input.get_nodes().end();
        ++it)
    {
      auto n2 = *it;
      if(visited.count(n2.key)>0)
        continue;
      double dist = func(n1, n2);
      if(best < dist){
        best = dist;
        argb = it;
        dir = true;
      }
      if(!sym){
        dist = func(n2, n1);
        if(best < dist){
          best = dist;
          argb = it;
          dir = false;
        }
      }
    }
    if(best>-1.0){
      output.insert(*argb);
      if(dir)
        output.insert_edge(n1.key, argb->key, best);
      else
        output.insert_edge(argb->key, n1.key, best);
    }
  }

  assert(output.nb_nodes() == input.nb_nodes());
  assert(output.nb_edges() == input.nb_nodes()-1);
  return output;
}


}

#endif // BEST_SPANNING_TREE_HH
