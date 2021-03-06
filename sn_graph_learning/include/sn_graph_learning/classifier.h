#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <sn_models/comparison_cluster.h>
#include <sn_graph_learning/cluster_holder.h>
#include <sn_models/comparison.h>

namespace sn{
namespace graph_learning{


template<typename ClusterHolder, typename CompFunc>
class Classifier{
private:
  typedef std::pair<double, int> score_t;

  CompFunc comp_func_;
  std::vector<score_t> classification_;

public:
  typedef typename std::vector<score_t>::const_reverse_iterator const_iterator;
  typedef typename ClusterHolder::model_t model_t;
  typedef ClusterHolder holder_t;

  Classifier(CompFunc const& func):comp_func_(func){}

  void classify(model_t const& cand,
                ClusterHolder const& models);

  int size()const{return classification_.size();}

  const_iterator begin()const{return classification_.rbegin();}
  const_iterator end()const{return classification_.rend();}

  const score_t& front()const{return classification_.back();}
  const score_t& back()const{return classification_.front();}
  const score_t& operator [](int pos)const{
    return classification_[classification_.size()-1-pos];}
  void pop(){classification_.pop_back();}

  std::vector<double> compare(const model_t& cand,
                       const holder_t& models)const{
    std::vector<double> results(models.size(), 0.0);
    for(int i=0; i<models.size(); ++i){
      results[i] = comp_func_(cand, models[i]);
    }
    return results;
  }
};

template<typename ClusterHolder, typename CompFunc>
void Classifier<ClusterHolder, CompFunc>::classify(typename ClusterHolder::model_t const& cand,
                                                    const ClusterHolder &models)
{
  if(models.size() == 0)
    return;
  comp_func_.setModels(models);

  classification_.clear();
  std::vector<double> results = compare(cand, models);
  assert(results.size() == models.size());
  for(int i=0; i<models.size(); ++i){
    classification_.push_back(score_t(results[i], i));
  }
  auto sort_func = [](score_t const& l, score_t const& r){
    return l.first < r.first;
  };
  std::sort(classification_.begin(), classification_.end(), sort_func);
  assert(classification_.size() == results.size());
}

template<typename ClusterHolder>
class Classifier2{
public:
  struct score_t {
    double mod;
    double inter;
    int id;
  };

  std::vector<score_t> classification_;

public:
  typedef typename std::vector<score_t>::const_reverse_iterator const_iterator;
  typedef typename ClusterHolder::model_t model_t;
  typedef ClusterHolder holder_t;

  void classify(model_t const& cand,
                ClusterHolder const& models);

  int size()const{return classification_.size();}

  const_iterator begin()const{return classification_.rbegin();}
  const_iterator end()const{return classification_.rend();}

  const score_t& front()const{return classification_.back();}
  const score_t& back()const{return classification_.front();}
  const score_t& operator [](int pos)const{
    return classification_[classification_.size()-1-pos];}
  void pop(){classification_.pop_back();}

  std::vector<score_t> compare(const model_t& cand,
                       const holder_t& models)const{
    static auto func = sn::comparison::intersection_size(sn::no_type());
    static auto mod = sn::comparison::modalities_match(sn::no_type());
    std::vector<score_t> results(models.size());
    for(int i=0; i<models.size(); ++i){
      results[i].mod = mod(cand, models[i]);
      results[i].inter = func(cand, models[i]);
      results[i].id = i;
    }
    return results;
  }
};

template<typename ClusterHolder>
void Classifier2<ClusterHolder>::classify(typename ClusterHolder::model_t const& cand,
                                                    const ClusterHolder &models)
{
  if(models.size() == 0)
    return;

  classification_.clear();
  classification_ = compare(cand, models);
  auto edge_inf = [](score_t const& l, score_t const& r){
    if(l.mod == r.mod)
      return l.inter < r.inter;
    return l.mod < r.mod;
  };
  std::sort(classification_.begin(), classification_.end(), edge_inf);
}

}
}

#endif // CLASSIFIER_H
