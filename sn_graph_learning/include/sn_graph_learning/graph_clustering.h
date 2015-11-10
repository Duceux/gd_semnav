#ifndef GRAPH_CLUSTERING_H
#define GRAPH_CLUSTERING_H

#include <sn_models/graph_of_word.h>

namespace sn {
namespace graph_clustering {

class GraphClustering {
public:
  typedef GraphOfWordPtr Model;
  typedef Graph<SimpleNode<Model>, Edge<Model, double, true>> ModelGraph;

public:
  // Give a label to a cluster
  void LabelCluster(const Model& cluster, const std::string& name);

  // Give a label to a model
  void LabelModel(const Model& model, const std::string& name);

  // Put model in the relationship graph
  void AddToKnowledgeGraph(const Model& m);

  // PerformClustering
  void ExtractClusters();

private:
  ModelGraph knowledge_;


};

}  // graph_clustering
}  // sn

#endif // GRAPH_CLUSTERING_H
