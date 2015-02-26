#ifndef DICO_H
#define DICO_H

#include <memory>
#include <unordered_map>
#include <vector>
#include <sn_msgs/Descriptor.h>
#include <functional>


namespace sn{

typedef sn_msgs::Descriptor descriptor_t;


struct Word{
    std::string type;
    int label;
    bool operator ==(Word const& other)const{
        return type == other.type && label == other.label;
    }
    bool operator !=(Word const& other)const{
        return !(*this == other);
    }
    bool operator <(Word const& other)const{
        if(type == other.type)
            return label < other.label;
        return type < other.type;
    }

    descriptor_t descriptor;
};

inline std::ostream& operator<< (std::ostream &out,  Word const&e)
{
  out <<  e.type << " " << e.label;
  return out;
}

typedef std::function<double(std::vector<double> const&, std::vector<double> const& )> DistFunc;
struct Distance{
    Distance(DistFunc f):func(f){}
    DistFunc func;

    template<class...Args>
    DistFunc::result_type operator()(Args... args){
        return func(args...);
    }
};

struct FastGetter{
    int operator ()(std::vector<double> const& des, std::vector<std::vector<double>> & words, double th, Distance func);
};


template<typename Getter>
struct Dictionary{
    Getter getter;
    std::unordered_map<std::string, double> thresholds;
    std::unordered_map<std::string, Distance> distances;
    std::unordered_map<std::string, std::vector<std::vector<double>>> words;

    void set(std::string const& type, double th, Distance dist){
        thresholds[type] = th;
        distances.insert(std::pair<std::string, Distance>(type, dist));
        words[type];
    }

    Word get(sn_msgs::Descriptor const& des){
        Word res;
        res.label =  getter(des.data,
                      words[des.type],
                      thresholds.at(des.type),
                      distances.at(des.type));
        res.type = des.type;
        res.descriptor = des;
        return res;
    }

    std::size_t size(std::string const& type){return words[type].size();}
};

}// sn


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

#endif // DICO_H
