#ifndef BAG_OF_WORD_H
#define BAG_OF_WORD_H

#include <sn_dictionary/dico.h>
#include <unordered_map>
#include <sn_msgs/DescriptorSequence.h>

template<typename T>
void
hash_combine(std::size_t &seed, T const &key) {
  std::hash<T> hasher;
  seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
  template<>
  struct hash<sn::Word> {
    std::size_t operator()(sn::Word const &p) const {
      std::size_t seed(0);
      ::hash_combine(seed, p.label);
      ::hash_combine(seed, p.type);
      return seed;
    }
  };
}


namespace sn {

struct BagOfWord{
    std::string name;
    std::unordered_map<sn::Word, unsigned int> bag;

    typedef std::shared_ptr<BagOfWord> Ptr;

    template<typename T>
    static Ptr create(sn_msgs::DescriptorSequence const& seq, sn::Dictionary<T> & dico){
        Ptr res(new BagOfWord);
        for(const sn_msgs::Descriptor& des: seq.descriptors)
            res->bag[dico.get(des)]++;
        res->name = seq.name;
        return res;
    }
};



double intersection(BagOfWord::Ptr const& l, BagOfWord::Ptr const& r){
        double res = 0.0;
        for(auto it:l->bag)
            if(r->bag.count(it.first) > 0)
                res+=std::min(it.second, r->bag.at(it.first));
        return res;
}

double binary_intersection(BagOfWord::Ptr const& l, BagOfWord::Ptr const& r){
        double res = 0.0;
        for(auto it:l->bag)
            if(r->bag.count(it.first) > 0)
                res+=1.0;
        return res;
}

}

#endif // BAG_OF_WORD_H
