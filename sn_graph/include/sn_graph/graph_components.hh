#ifndef GRAPH_COMPONENTS_HH
#define GRAPH_COMPONENTS_HH

#include <sn_graph/graph.hh>
#include <sn_graph/wise_operators.hh>

namespace sn{


template<typename N, typename E>
std::vector<Graph<N, E>> get_connected_components(Graph<N, E> const& g)
{
  std::set<N> visited;

  std::function<void(Graph<N, E>&, N const&)> dfs = [&](Graph<N, E>& graph, N const& node){
    if(visited.insert(node).second == false)return;
    graph.insert(node);
    if(g.nb_out_edges_of(node) > 0)
      for(const auto& edge: g.get_out_edges(node)){
            dfs(graph, g.at(edge.child));
            graph.insert(edge);
        }
    if(g.nb_in_edges_of(node) > 0)
      for(const auto& edge: g.get_in_edges(node)){
            dfs(graph, g.at(edge.parent));
            graph.insert(edge);
        }
  };

  std::vector<Graph<N, E>> result;

  for(const auto& node: g.get_nodes()){
    if(visited.count(node) == 0){
      Graph<N, E> comp;
      dfs(comp, node);
      result.push_back(comp);
    }
  }

  if(result.size() == 0)
    result.push_back(g);

  return result;
}

template<typename N, typename E>
Graph<N, E> get_max_connected_components(Graph<N, E> const& g){
  std::vector<Graph<N, E>> result = get_connected_components(g);

  auto comp = [](Graph<N, E> best, Graph<N, E> current)->bool{
    return best.size() < current.size();};
  auto res = best_element(result.begin(), result.end(), comp);
  return *res;
}

}


#endif // GRAPH_COMPONENTS_HH
