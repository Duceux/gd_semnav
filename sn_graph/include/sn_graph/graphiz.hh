#ifndef GRAPHIZ_HH
#define GRAPHIZ_HH

#include <fstream>
#include <sn_graph/graph_components.hh>

namespace sn {
namespace graph {

struct SimplePrintGraph{
  template<typename Graph>
  std::string operator()(Graph const& graph)const{
    std::stringstream str;
    str << "label = \"" << "nodes: " << graph.nb_nodes() << " edges: " << graph.nb_edges() << " "
         << "clusters: " << get_connected_components(graph).size() << "\";";
    return str.str();
  }
};

template <typename Graph, typename PrintNode, typename PrintEdge, typename PrintGraph = SimplePrintGraph>
void graphiz_save(const Graph& graph, std::ofstream& file, PrintNode const& op1, PrintEdge const& op2, PrintGraph const& op3)
{

  if(graph.is_directed()){
    file << "digraph G {\n";
  }
  else
    file << "graph G {\n";

  file << op3(graph) << "\n";

  // get the hash function to have the id
  auto hashf = HashNode<typename Graph::node_type>();
  for(const auto& node: graph.get_nodes())
    file << "\"" << hashf(node) << "\"" <<  op1(node) << "\n";

  for(const auto& edge: graph.get_edges()){
    if(graph.is_directed())
      file << "\"" << hashf(edge.parent) << "\"" << "->" << "\""<<  hashf(edge.child) << "\"" << op2(edge) << "\n";
    else
      file << "\"" << hashf(edge.parent) << "\"" << "--"<< "\"" << hashf(edge.child) << "\"" << op2(edge) << "\n";
  }

  file << "\n";
  file << "}\n";
}


template <typename Graph, typename PrintNode, typename PrintEdge, typename PrintGraph>
void graphiz_save(const Graph& graph, const std::string& filename, PrintNode const& op1, PrintEdge const& op2, PrintGraph const& op3)
{
  std::ofstream file(filename);
  graphiz_save(graph, file, op1, op2, op3);
  file.close();
}

template <typename Graph, typename PrintNode, typename PrintEdge, typename PrintGraph>
void graphiz_save_by_components(const Graph& graph, const std::string& filename, PrintNode const& op1, PrintEdge const& op2, PrintGraph const& op3)
{
  std::ofstream file(filename);
  auto components = get_connected_components(graph);
  for(auto const& subg: components)
    graphiz_save(subg, file, op1, op2, op3);
  file.close();
}

}//sn
}//graph


#endif // GRAPHIZ_HH
