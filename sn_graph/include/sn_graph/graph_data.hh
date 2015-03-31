#ifndef GRAPH_DATA_HH
#define GRAPH_DATA_HH

#include <unordered_map>
#include <unordered_set>


namespace sn {

template <typename T>
struct HashNode{
  std::size_t operator()(const T& t)const{
    return std::hash<typename T::key_type>()(t.key);
  }
};


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

template<typename Key>
struct SimpleNode{
  typedef Key key_type;
  typedef Key value_type;
  typedef SimpleNode<Key> self_type;

  const Key key;

  SimpleNode(const Key& key_):
    key(key_)
  {}

  bool operator ==(self_type const& other)const{
    return key == other.key;
  }

  bool operator !=(self_type const& other)const{
    return !(*this == other);
  }

  operator Key()const{
    return key;
  }

  bool operator <(self_type const& other)const{
    return key < other.key;
  }

};

template<typename Key>
std::ostream& operator<< (std::ostream &out, const SimpleNode<Key> &e)
{
  out <<  e.key;
  return out;
}

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

template<typename Edge>
Edge make_opposed(Edge const& e){
  Edge res(e);
  std::swap(res.child, res.parent);
  return res;
}

template<typename Key, bool D>
std::ostream& operator<< (std::ostream &out, const SimpleEdge<Key, D> &e)
{
  if(e.directed)
    out <<  e.parent << "->" << e.child;
  else
    out <<  e.parent << "--" << e.child;
  return out;
}


template<typename Key, typename NodeData>
struct Node: public SimpleNode<Key>
{
  const Key key;
  NodeData val;

  Node(const Key& key_, const NodeData& val_ = NodeData()):
    key(key_), val(val_)
  {}

  typedef Key key_type;
  typedef NodeData value_type;
};

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
  out << "(" << e.key << ", "
      << e.parent << ", "
      << e.child << ", "
      << e.val << ")";
  return out;
}

template<typename Node>
using NodeSet = std::unordered_set<Node, HashNode<Node>>;

template<typename Edge>
using EdgeSet = std::unordered_set<Edge, HashEdge<Edge>>;

template<typename Edge>
using EdgeMap = std::unordered_map<typename Edge::key_type, EdgeSet<Edge>>;


template<typename Node,
         typename Edge>
struct GraphData{
  typedef typename Node::key_type key_type;
  typedef Node node_type;
  typedef Edge edge_type;
  typedef NodeSet<node_type> node_set_type;
  typedef EdgeSet<edge_type> edge_set_type;
  typedef EdgeMap<edge_type> edge_map_type;
  typedef GraphData<node_type, edge_type> self_type;

  static const bool directed = Edge::directed;
  node_set_type nodes;
  edge_map_type in_edges;
  edge_map_type out_edges;
  edge_set_type edges;
};

}


#endif // GRAPH_DATA_HH
