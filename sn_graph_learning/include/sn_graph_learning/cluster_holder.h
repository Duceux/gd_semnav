#ifndef CLUSTER_HOLDER_H
#define CLUSTER_HOLDER_H

#include <vector>
#include <map>
#include <string>

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

  void clear() {
    prototypes_.clear();
  }

  typename std::vector<model_t>::const_iterator begin()const{return prototypes_.begin();}
  typename std::vector<model_t>::const_iterator end()const{return prototypes_.end();}

private:
  std::vector<model_t> prototypes_;
  MergeFunc mergefunc_;
};

template<typename Model, typename MergeFunc>
class LearningMatrix{
public:
  typedef Model model_t;
  typedef Model proto_t;
  typedef std::map<std::string, int> cluster_info_t;
  LearningMatrix(MergeFunc const& func):mergefunc_(func){}

  int size()const{return prototypes_.size();}
  model_t const& operator[](int i)const{return prototypes_[i];}

  void clear() {
    prototypes_.clear();
    supports_.clear();
  }

  void new_cluster(model_t const& m){
    std::map<std::string, int> new_cluster;
    new_cluster[m->name]+=m->merged;
    supports_.push_back(new_cluster);
    prototypes_.push_back(m);
  }

  void add_to_cluster(int i, model_t const& m){
    supports_[i][m->name]+=m->merged;
    prototypes_[i] = mergefunc_(prototypes_[i], m);
  }

  typename std::vector<model_t>::const_iterator begin()const{return prototypes_.begin();}
  typename std::vector<model_t>::const_iterator end()const{return prototypes_.end();}

  const std::vector<cluster_info_t>& get_clusters() const { return supports_; }

  void rearrange_cluters() {
    for(unsigned i=0; i<supports_.size()-1; ++i)
      for(unsigned j=i+1; j<supports_.size(); ++j){
        cluster_info_t& ci = supports_[i];
        cluster_info_t& cj = supports_[j];
        // push the most corrupted to the end
        if(ci.size() > cj.size() && cj.size() >= 1){
         cj.swap(ci);
         prototypes_[j].swap(prototypes_[i]);
         continue;
        }
        // if ci is not corrupted but cj is do nothing
        if(ci.size() == 1 && cj.size() > 1)
          continue;

        // here no corruption
        // check for the name
        if(ci.begin()->first > cj.begin()->first){
          cj.swap(ci);
          prototypes_[j].swap(prototypes_[i]);
          continue;
        }
        // check for size
        if(ci.begin()->first == cj.begin()->first && ci.begin()->second < cj.begin()->second){
          cj.swap(ci);
          prototypes_[j].swap(prototypes_[i]);
          continue;
        }
    }
  }

private:
  std::vector<cluster_info_t> supports_;
  std::vector<model_t> prototypes_;
  MergeFunc mergefunc_;
};


}
}

#endif // CLUSTER_HOLDER_H
