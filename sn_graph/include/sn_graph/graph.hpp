#include <fstream>
#include <set>
#include <iostream>
#include <cassert>
#include <stdexcept>

namespace sn{

template<typename Key>
void throw_out_of_range(const Key& key){
  std::stringstream str;
  str << key;
  throw std::out_of_range (str.str());
}

template<typename Node, typename Edge>
void Graph<Node, Edge>::clear() {
  ptr_ = std::make_shared<data_type>();
}


template<typename Node, typename Edge>
template<typename... Args>
bool Graph<Node, Edge>::insert_node(const key_type& key,  Args const&... args){
  return insert(Node(key, args...));
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::insert(const node_type& node)
{
  return ptr_->nodes.insert(node).second;
}

template<typename Node, typename Edge>
void Graph<Node, Edge>::remove(const key_type& key){
  ptr_->nodes.erase(key);
  detach(key);
}

template<typename Node, typename Edge>
void Graph<Node, Edge>::detach(const key_type& key){
  edge_set_type to_remove;
  if(nb_out_edges_of(key) > 0) {
    const auto& out = get_out_edges(key);
    to_remove.insert(out.begin(), out.end());
  }
  if(nb_in_edges_of(key) > 0) {
    const auto& in = get_in_edges(key);
    to_remove.insert(in.begin(), in.end());
  }
  ptr_->out_edges.erase(key);
  ptr_->in_edges.erase(key);
  for(Edge e: to_remove){
    ptr_->edges.erase(e);
    if(ptr_->out_edges.count(e.child) > 0)
      ptr_->out_edges.at(e.child).erase(e);
    if(ptr_->in_edges.count(e.parent) > 0)
      ptr_->in_edges.at(e.parent).erase(e);
    if(ptr_->out_edges.count(e.parent) > 0)
      ptr_->out_edges.at(e.parent).erase(e);
    if(ptr_->in_edges.count(e.child) > 0)
      ptr_->in_edges.at(e.child).erase(e);
  }
}

template<typename Node, typename Edge>
void Graph<Node, Edge>::detach(const node_type& node){
  detach(node.key);
}

template<typename Node, typename Edge>
const typename Graph<Node, Edge>::node_type&
Graph<Node, Edge>::at(const key_type& key)const{
  Node n(key);
  if(ptr_->nodes.count(n) == 0)
    throw_out_of_range(key);
  return *ptr_->nodes.find(n);
}


template<typename Node, typename Edge>
const typename Graph<Node, Edge>::edge_type&
Graph<Node, Edge>::at(const key_type& p, const key_type& c)const{
  Edge e(p, c);
  if(ptr_->edges.count(e) == 0)
    throw_out_of_range(e);
  return *ptr_->edges.find(e);
}


template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_nodes()const{
  return ptr_->nodes.size();
}


template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_edges_of(const key_type& key)const{
  return nb_in_edges_of(key)+nb_out_edges_of(key);
}

template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_edges_of(const node_type& node)const{
  return nb_edges_of(node.key);
}

template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_out_edges_of(const node_type& node)const{
  return nb_out_edges_of(node.key);
}

template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_out_edges_of(const key_type& key)const{
  auto it = ptr_->out_edges.find(key);
  if(it != ptr_->out_edges.end())
    return it->second.size();
  return 0;
}

template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_in_edges_of(const key_type& key)const{
  auto it = ptr_->in_edges.find(key);
  if(it != ptr_->in_edges.end())
    return it->second.size();
  return 0;
}

template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_in_edges_of(const node_type& node)const{
  return nb_in_edges_of(node.key);
}

template<typename Node, typename Edge>
void Graph<Node, Edge>::remove(const node_type& node){
  remove(node.key);
}

template<typename Node, typename Edge>
template<typename... Args>
bool Graph<Node, Edge>::insert_edge(const key_type& k1, const key_type& k2, Args const&... args)
{
  return insert(Edge(k1, k2, args...));
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::insert(const edge_type& e){
  if(ptr_->nodes.count(e.parent) == 0)
    throw_out_of_range(e.parent);
  if(ptr_->nodes.count(e.child) == 0)
    throw_out_of_range(e.child);
  ptr_->edges.insert(e);
  ptr_->out_edges[e.parent].insert(e);
  ptr_->in_edges[e.child].insert(e);
  if(!ptr_->directed){
    ptr_->edges.insert(make_opposed(e));
    ptr_->out_edges[e.child].insert(make_opposed(e));
    ptr_->in_edges[e.parent].insert(make_opposed(e));
  }
  return true;
}


template<typename Node, typename Edge>
std::size_t Graph<Node, Edge>::nb_edges()const
{
  return ptr_->edges.size();
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::remove(const key_type& k1, const key_type& k2)
{
  Edge e(k1, k2);
  ptr_->edges.erase(e);
  auto it1 = ptr_->out_edges.find(k1);
  if(it1 != ptr_->out_edges.end()){
    it1->second.erase(e);
    if(it1->second.size() == 0)
      ptr_->out_edges.erase(it1);
  }
  auto it2 = ptr_->in_edges.find(k2);
  if(it2 != ptr_->in_edges.end()){
    it2->second.erase(e);
    if(it2->second.size() == 0)
      ptr_->in_edges.erase(it2);
  }
  return true;
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::remove(const edge_type& e)
{
  return remove(e.parent, e.child);
}


template<typename Node, typename Edge>
typename Graph<Node, Edge>::node_type Graph<Node, Edge>::min_in_edges_node()const{
  auto arg_min = ptr_->nodes.begin()->first;
  if(ptr_->in_edges.count(arg_min) == 0)return node_at(arg_min);
  auto min = ptr_->in_edges.at(arg_min).size();
  for(auto node: ptr_->nodes){
    if(ptr_->in_edges.count(node.first) == 0)return node_at(node.first);
    auto nb = ptr_->in_edges.at(node.first).size();
    if(min > nb){
      arg_min = node.first;
      min = nb;
    }
  }
  return node_at(arg_min);
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::has(const key_type& p, const key_type& c)const{
  return ptr_->edges.count(Edge(p, c));
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::has(const key_type& k1)const{
  return ptr_->nodes.count(Node(k1)) > 0;
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::has(const node_type& n)const{
  return has(n.key);
}

template<typename Node, typename Edge>
bool Graph<Node, Edge>::has(const edge_type& e)const{
  return has(e.parent, e.child);
}

template<typename Node, typename Edge>
typename Graph<Node, Edge>::edge_set_type const&
Graph<Node, Edge>::get_out_edges(const key_type& key)const{
  return ptr_->out_edges.at(key);
}

template<typename Node, typename Edge>
typename Graph<Node, Edge>::edge_set_type &
Graph<Node, Edge>::get_out_edges(const key_type& key){
  return ptr_->out_edges.at(key);
}

template<typename Node, typename Edge>
typename Graph<Node, Edge>::edge_set_type const&
Graph<Node, Edge>::get_in_edges(const key_type& key)const{
  return ptr_->in_edges.at(key);
}

template<typename Node, typename Edge>
typename Graph<Node, Edge>::edge_set_type &
Graph<Node, Edge>::get_in_edges(const key_type& key){
  return ptr_->in_edges.at(key);
}

}
