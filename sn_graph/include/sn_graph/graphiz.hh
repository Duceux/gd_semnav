#ifndef GRAPHIZ_HH
#define GRAPHIZ_HH

#include <fstream>

namespace graph_clustering {

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
       << "clusters: " << graph.get_connected_components().size() << "\";\n";

  for(const auto& node: graph.get_nodes())
    if(graph.nb_edges_of(node)>0)
        file << node.key <<  op1(node.val) << "\n";

  for(const auto& edge: graph.get_edges()){
    if(graph.is_directed())
      file << edge.parent << "->" << edge.child << "[ label = \"" << op2(edge.val) << "\", color = black ];\n";
    else
      file << edge.parent << "--" << edge.child << "[ label = \"" << op2(edge.val) << "\", color = black ];\n";
  }

  file << "\n";
  file << "}\n";

  file.close();
}


}

#endif // GRAPHIZ_HH
