#ifndef GRAPH_DATA_HH
#define GRAPH_DATA_HH

#include <unordered_map>
#include <unordered_set>
#include <sn_graph/edge.hh>

namespace sn {

template <typename T>
struct HashNode{
  std::size_t operator()(const T& t)const{
    return std::hash<typename T::key_type>()(t.key);
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

template<typename Key, typename NodeData>
struct Node: public SimpleNode<Key>
{
  NodeData val;

  Node(const Key& key_, const NodeData& val_ = NodeData()):
    SimpleNode<Key>(key_), val(val_)
  {}

  typedef Key key_type;
  typedef NodeData value_type;
};

template<typename Node>
using NodeSet = std::unordered_set<Node, HashNode<Node>>;


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
