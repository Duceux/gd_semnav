#ifndef CLUSTER_HOLDER_H
#define CLUSTER_HOLDER_H

#include <vector>

namespace sn{
namespace graph_learning{

template<typename Model, typename MergeFunc>
class VectorHolder{
public:
  typedef Model model_t;
  typedef Model proto_t;

  VectorHolder(MergeFunc const& func):mergefunc_(func){}

  int size()const{return prototypes_.size();}
  model_t const& operator[](int i)const{return prototypes_[i];}

  void new_cluster(model_t const& m){
    prototypes_.push_back(m);
  }

  void add_to_cluster(int i, model_t const& m){
    prototypes_[i] = mergefunc_(prototypes_[i], m);
  }

  typename std::vector<model_t>::const_iterator begin()const{return prototypes_.begin();}
  typename std::vector<model_t>::const_iterator end()const{return prototypes_.end();}

private:
  std::vector<model_t> prototypes_;
  MergeFunc mergefunc_;
};

}
}

#endif // CLUSTER_HOLDER_H
