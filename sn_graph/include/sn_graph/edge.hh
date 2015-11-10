#ifndef EDGE_HH
#define EDGE_HH

namespace sn {

template <typename T>
struct HashEdge{
  void hash_combine(std::size_t &seed, typename T::key_type const &key)const{
    std::hash<typename T::key_type> hasher;
    seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }

  std::size_t operator()(const T& t)const{
    if(t.directed){
      std::size_t seed(0);
      hash_combine(seed, t.parent);
      hash_combine(seed, t.child);
      return seed;
    }else{
      std::size_t seed(0), seed2(0);
      hash_combine(seed, t.parent);
      hash_combine(seed, t.child);
      hash_combine(seed2, t.child);
      hash_combine(seed2, t.parent);
      return std::min(seed, seed2);
    }
  }
};

template<typename E>
struct LessEdge{
  bool operator ()(E const& l, E const& r)const{
    // TODO: undirected version
    //if(l.directed)
    {
      if(l.parent < r.parent)
        return true;
      if(l.parent == r.parent && l.child < r.child)
        return true;
      return false;
    }
  }
};


template<typename Key, bool Directed = true>
struct SimpleEdge{
  typedef Key key_type;
  typedef Key value_type;
  typedef SimpleEdge<Key, Directed> self_type;


  Key parent;
  Key child;
  static const bool directed = Directed;

  SimpleEdge(Key const& p, Key const& c):
    parent(p), child(c)
  {}

  bool operator ==(self_type const& other)const{
    if(directed)
      return parent == other.parent && child == other.child;
    else{
      return (parent == other.parent && child == other.child) ||
          (parent == other.child && child == other.parent);
    }
  }

  bool operator !=(self_type const& other)const{
    return !(*this == other);
  }
};

template<typename Key, bool D>
std::ostream& operator<< (std::ostream &out, const SimpleEdge<Key, D> &e)
{
  if(e.directed)
    out <<  e.parent << "->" << e.child;
  else
    out <<  e.parent << "--" << e.child;
  return out;
}

template<typename Edge>
Edge make_opposed(Edge const& e){
  Edge res(e);
  std::swap(res.child, res.parent);
  return res;
}

template<typename Key, typename EdgeData, bool Directed = true>
struct Edge: public SimpleEdge<Key, Directed>
{
  typedef EdgeData value_type;

  EdgeData val;

  Edge(const Key& p, const Key& c, EdgeData v = EdgeData()):
    SimpleEdge<Key, Directed>(p, c), val(v)
  {}
};

template<typename Key, typename EdgeData, bool Directed = true>
std::ostream& operator<< (std::ostream &out, const Edge<Key, EdgeData, Directed> &e)
{
  out << "("
      << e.parent << ", "
      << e.child << ", "
      << e.val << ")";
  return out;
}

template<typename Edge>
using EdgeSet = std::unordered_set<Edge, HashEdge<Edge>>;

template<typename Edge>
using EdgeMap = std::unordered_map<typename Edge::key_type, EdgeSet<Edge>>;


}


#endif // EDGE_HH
