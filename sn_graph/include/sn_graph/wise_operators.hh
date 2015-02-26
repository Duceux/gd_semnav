#ifndef WISE_OPERATORS_HH
#define WISE_OPERATORS_HH

namespace sn {

template <class ForwardIterator, typename Comp>
ForwardIterator best_element ( ForwardIterator first, ForwardIterator last, Comp&& func )
{
  if (first==last) return first;
  ForwardIterator largest = first;

  while (++first!=last)
    if (func(*largest, *first))
      largest=first;
  return largest;
}

template<typename Graph, typename Comp>
typename Graph::node_type best_node(const Graph& g, Comp&& func){
  auto res = best_element(g.get_nodes().begin(),
                          g.get_nodes().end(),
                          func);
  return *res;
}

template<typename Graph, typename Comp>
typename Graph::edge_type best_out_edge(const Graph& g, const typename Graph::key_type& key, Comp&& func){
  auto res = best_element(g.get_out_edges(key).begin(),
                          g.get_out_edges(key).end(),
                          func);
  return *res;
}

template<typename Graph, typename Comp>
typename Graph::edge_type best_out_edge(const Graph& g, const typename Graph::node_type& node, Comp&& func){
  return best_out_edge(g, node.key, func);
}


template<typename Graph, typename Comp>
typename Graph::edge_type best_in_edge(const Graph& g, const typename Graph::key_type& key, Comp&& func){
  auto res = best_element(g.get_in_edges(key).begin(),
                          g.get_in_edges(key).end(),
                          func);
  return *res;
}

template<typename Graph, typename Comp>
typename Graph::edge_type best_in_edge(const Graph& g, const typename Graph::node_type& node, Comp&& func){
  return best_in_edge(g, node.key, func);
}


template<typename Graph, typename Comp>
typename Graph::edge_type best_edge(const Graph& g, Comp&& func){
  auto res = best_element(g.get_edges().begin(),
                          g.get_edges().end(),
                          func);
  return *res;
}

}

#endif // WISE_OPERATORS_HH
