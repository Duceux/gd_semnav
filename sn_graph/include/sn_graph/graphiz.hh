#ifndef GRAPHIZ_HH
#define GRAPHIZ_HH

#include <fstream>

namespace sn {
namespace graph {

template <typename Graph, typename PrintNode, typename PrintEdge>
void graphiz_save(const Graph& graph, const std::string& filename, PrintNode op1, PrintEdge op2)
{
  std::ofstream file(filename);

  if(graph.is_directed()){
    file << "digraph G {\n";
  }
  else
    file << "graph G {\n";

  file << "label = \"" << "nodes: " << graph.nb_nodes() << " edges: " << graph.nb_edges() << " "
       << "clusters: " << get_connected_components(graph).size() << "\";\n";

  for(const auto& node: graph.get_nodes())
    file << op1(node) << "\n";

  for(const auto& edge: graph.get_edges()){
      file << op2(edge) << "\n";
  }

  file << "\n";
  file << "}\n";

  file.close();
}

template <typename Graph, typename PrintNode, typename PrintEdge, typename PrintGraph>
void graphiz_save(const Graph& graph, const std::string& filename, PrintNode op1, PrintEdge op2, PrintGraph op3)
{
  std::ofstream file(filename);

  if(graph.is_directed()){
    file << "digraph G {\n";
  }
  else
    file << "graph G {\n";

  file << op3(graph) << "\n";

  for(const auto& node: graph.get_nodes())
    file << "\"" << node.key << "\"" <<  op1(node) << "\n";

  for(const auto& edge: graph.get_edges()){
    if(graph.is_directed())
      file << "\"" << edge.parent << "\"" << "->" << "\""<<  edge.child<< "\"" << op2(edge) << "\n";
    else
      file << "\"" << edge.parent << "\"" << "--"<< "\"" << edge.child<< "\"" << op2(edge) << "\n";
  }

  file << "\n";
  file << "}\n";

  file.close();
}

}//sn
}//graph


#endif // GRAPHIZ_HH
