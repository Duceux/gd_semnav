#ifndef GRAPH_HH
#define GRAPH_HH

#include <vector>
#include <memory>
#include <sstream>
#include <iostream>
#include <sn_graph/graph_data.hh>

namespace sn {

template<typename Node,
         typename Edge>
class Graph{
  static_assert(std::is_same<typename Node::key_type,
                typename Edge::key_type>::value,
                "Node and Edge must have the same key type.");

public:
  typedef typename Node::key_type key_type;
  typedef Node node_type;
  typedef Edge edge_type;
  typedef NodeSet<node_type> node_set_type;
  typedef EdgeSet<edge_type> edge_set_type;
  typedef Graph<node_type, edge_type> self_type;
  typedef GraphData<node_type, edge_type> data_type;

  //constructors
  Graph():ptr_(new data_type){}
  Graph(const data_type& data):
    ptr_(new data_type(data)){}

  self_type copy()const{
    return self_type(*ptr_);
  }

  void swap(self_type& oth){
    std::swap(ptr_, oth.ptr_);
  }

  // Graph
  void clear();

  //operators
  const node_type& operator()(const key_type& key)const{return at(key);}
  const node_type& operator()(const key_type& p, const key_type& c)const{return at(p, c);}

  //SIZE STUFF
  std::size_t nb_nodes()const;
  std::size_t nb_edges()const;

  std::size_t nb_edges_of(const key_type& key)const;
  std::size_t nb_edges_of(const node_type& m)const;

  std::size_t nb_out_edges_of(const key_type& key)const;
  std::size_t nb_out_edges_of(const node_type& m)const;

  std::size_t nb_in_edges_of(const key_type& key)const;
  std::size_t nb_in_edges_of(const node_type& m)const;

  std::size_t size()const{return nb_nodes()+nb_edges();}
  bool empty()const{return size()==0;}

  //NODES

  template<typename... Args>
  bool insert_node(const key_type& key,  Args const&... args);
  bool insert(const node_type& node);

  void remove(const key_type& key);
  void remove(const node_type& node);

  void detach(const key_type& key);
  void detach(const node_type& node);

  bool has(const key_type& k1)const;
  bool has(const node_type& n)const;

  const node_type& at(const key_type& key)const;

  node_type min_in_edges_node()const;

  // EDGES
  template<typename... Args>
  bool insert_edge(const key_type& k1, const key_type& k2, Args const&... args);
  bool insert(const edge_type& e);

  bool remove(const key_type& k1, const key_type& k2);
  bool remove(const edge_type& e);

  bool has(const key_type& k1, const key_type& k2)const;
  bool has(const edge_type& e)const;

  const edge_type& at(const key_type& p, const key_type& c)const;

  // GETTERS
  bool is_directed()const{return ptr_->directed;}

  node_set_type& get_nodes(){return ptr_->nodes;}
  const node_set_type& get_nodes()const{return ptr_->nodes;}

  edge_set_type& get_edges(){return ptr_->edges;}
  const edge_set_type& get_edges()const{return ptr_->edges;}

  edge_set_type& get_out_edges(const key_type& key);
  const edge_set_type& get_out_edges(const key_type& key)const;

  edge_set_type& get_in_edges(const key_type& key);
  const edge_set_type& get_in_edges(const key_type& key)const;

private:
  std::shared_ptr<data_type> ptr_;
};

}

#include <sn_graph/graph.hpp>

#endif // GRAPH_H
