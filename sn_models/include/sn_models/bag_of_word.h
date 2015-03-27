#ifndef BAG_OF_WORD_H
#define BAG_OF_WORD_H

#include <sn_dictionary/dico.h>
#include <unordered_map>
#include <sn_msgs/DescriptorSequence.h>


namespace sn {

struct BagOfWord{
  typedef std::unordered_map<Word, unsigned int> Bag;
  typedef std::shared_ptr<BagOfWord> Ptr;

  std::string name;
  ros::Time uid;
  std::shared_ptr<Bag> bag;

  BagOfWord():bag(new Bag){}
  static Ptr create(){return Ptr(new BagOfWord);}
  void add(Word const& w){
    //FIXME: check distance with previous word
    (*bag)[w]++;
  }

  void print()const{
    std::cout << name << " "
              << uid << std::endl;
    std::cout << nb_words() << std::endl;
  }

  int nb_words()const{return bag->size();}

};

double intersection(BagOfWord::Ptr const& l, BagOfWord::Ptr const& r){
  double res = 0.0;
  for(auto it: *l->bag)
    if(r->bag->count(it.first) > 0)
      res+=std::min(it.second, r->bag->at(it.first));
  return res;
}

double binary_intersection(BagOfWord::Ptr const& l, BagOfWord::Ptr const& r){
  double res = 0.0;
  for(auto it: *l->bag)
    if(r->bag->count(it.first) > 0)
      res+=1.0;
  return res;
}

struct BowIntersection{
  virtual double operator()(const BagOfWord::Ptr &l,
                            const BagOfWord::Ptr &r)const{
    return intersection(l, r);
  }
};

}

#endif // BAG_OF_WORD_H
