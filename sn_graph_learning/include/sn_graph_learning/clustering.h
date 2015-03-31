#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <sn_graph_learning/classifier.h>

namespace sn{
namespace graph_learning{

template<typename Classifier>
class SequentialClustering{
private:
  double threshold;

public:
  SequentialClustering(double th):threshold(th){}

  void cluster(Classifier const& classification,
          typename Classifier::holder_t& holder,
          typename Classifier::model_t const& cand){
    if(classification.size() == 0){
      holder.new_cluster(cand);
      return;
    }
    if(classification.front().first > threshold)
      holder.add_to_cluster(classification.front().second, cand);
    else
      holder.new_cluster(cand);
  }
};


}
}

#endif // CLUSTERING_H
