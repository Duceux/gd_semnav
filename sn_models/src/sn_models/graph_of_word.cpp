#include <sn_models/graph_of_word.h>

namespace sn {

GraphOfWord::GraphOfWord():
  graph(new GraphOfWordData)
{}

void GraphOfWord::add(const Word &w)
{
  graph->insert(w);

  ros::Duration dmin = ros::DURATION_MAX;
  for(auto n: graph->get_nodes()){
    auto duration =  w.descriptor.header.stamp - n.key.descriptor.header.stamp;
    if(duration > ros::Duration() && duration < dmin)
      dmin = duration;
  }
  for(auto n: graph->get_nodes()){
    auto duration =  w.descriptor.header.stamp - n.key.descriptor.header.stamp;
    if(duration <= dmin && dmin < ros::DURATION_MAX &&
       n != w)
      graph->insert_edge(n.key, w);
  }
}

GraphOfWord::Ptr GraphOfWord::create()
{
  return Ptr(new Ptr::element_type);
}

void GraphOfWord::print() const
{
  std::cout << name << " "
            << uid << std::endl;
  std::cout << graph->nb_nodes() << " "
            << graph->nb_edges() << std::endl;
//  for(auto n: graph->get_nodes())
//    std::cout << n << std::endl;
//  for(auto n: graph->get_edges())
  //    std::cout << n << std::endl;
}

std::map<std::string, int> GraphOfWord::type_map() const
{
  std::map<std::string, int> result;
  for(node_t w: graph->get_nodes()){
    result["node"]++;
    result[type_of(w)]++;
  }
  for(edge_t w: graph->get_edges()){
    result["edge"]++;
    result[type_of(w)]++;
  }
  return result;
}

std::string GraphOfWord::type_of(const edge_t &e)
{
  if(e.parent.type < e.child.type)
    return e.parent.type+"-"+e.child.type;
  else
    return e.child.type+"-"+e.parent.type;
}

std::string GraphOfWord::type_of(const node_t &e)
{
  return e.key.type;
}

std::map<std::string, int> GraphOfWord::type_map(const GraphOfWordData &graph)
{
  std::map<std::string, int> result;
  for(node_t w: graph.get_nodes()){
    result[type_of(w)]++;
  }
  for(edge_t w: graph.get_edges()){
    result[type_of(w)]++;
  }
  return result;
}

GraphOfWord GraphOfWord::merge(const GraphOfWord &l, const GraphOfWord &r)
{
  GraphOfWord gow;
  gow.uid = ros::Time(ros::WallTime::now().toSec());
  if(l.name == r.name)
    gow.name = l.name;
  else
    gow.name = "Unknown";

  *gow.graph = graph::addition(*l.graph, *r.graph);
  return gow;
}

}//sn
