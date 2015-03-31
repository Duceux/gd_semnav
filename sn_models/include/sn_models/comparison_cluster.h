#ifndef COMPARISON_CLUSTER_H
#define COMPARISON_CLUSTER_H

#include <sn_models/functor.h>

namespace sn {
namespace models {

struct comparison_to_cluster{
  struct score_t{
    double score;
    GraphOfWordPtr model;
  };

  typedef std::vector<score_t> result_t;
  result_t operator ()(const GraphOfWordPtr& cand,
                       const std::vector<GraphOfWordPtr>& models);
};

}
}


#endif // COMPARISON_CLUSTER_H
