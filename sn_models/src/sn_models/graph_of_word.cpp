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
//  std::cout << dmin << std::endl;

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

}
