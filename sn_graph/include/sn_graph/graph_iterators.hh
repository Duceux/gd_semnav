#ifndef GRAPH_ITERATORS_HH
#define GRAPH_ITERATORS_HH

#include <iterator>
#include <sn_graph/graph_data.hh>

namespace sn {

template<typename OuterIt, typename InnerIt, typename EdgeType>
struct EdgeIterator: std::iterator<std::forward_iterator_tag, EdgeType>
{
public:
  typedef EdgeType edge_type;
  typedef InnerIt inner_iterator;
  typedef OuterIt outer_iterator;

  EdgeIterator(const outer_iterator& begin,
               const outer_iterator& end):
    current_map(begin), map_end(end)
  {
    if(is_end())
      return;
    else{
      current_edge = current_map->second.begin();
      edge_end = current_map->second.end();
    }
    while(!is_valid())advance();
  }

  const edge_type& operator*()const{
    return *current_edge;
  }
  const edge_type* operator->() const{
    return current_edge;
  }


  EdgeIterator &operator++(){
    if(is_end())
      return *this;
    do{
      advance();
    }
    while(!is_valid());
    return *this;
  }

  EdgeIterator& operator++(int){
    const EdgeIterator old(*this);
    ++(*this);
    return old;
  }


  bool operator!= (const EdgeIterator& other) const {
    return !(*this == other);
  }

  bool operator== (const EdgeIterator& other) const {
//    std::cout << "==: " << other.is_end() << " " << is_end() << " " << is_valid() << " " << (current_edge == other.current_edge) << std::endl;
    if(other.is_end())
      return is_end();
    else
      return current_edge == other.current_edge && current_map == other.current_map;
  }

private:
  inner_iterator current_edge;
  outer_iterator current_map;
  outer_iterator map_end;
  inner_iterator edge_end;

  bool is_end()const{
    return current_map == map_end;
  }

  bool is_valid()const{
    if(is_end())
      return true;
    else
      return current_edge != edge_end;
  }

  void advance(){
    if(is_end())
      return;
    if(current_edge != edge_end){
      ++current_edge;
      return;
    }
    ++current_map;
    if(!is_end()){
      current_edge = current_map->second.begin();
      edge_end = current_map->second.end();
    }
  }

};

template<class GraphData>
class EdgeIteratorWrapper{
public:
  typedef EdgeIterator<
  typename GraphData::edge_map_type::iterator,
  typename GraphData::edge_map_type::mapped_type::iterator,
  typename GraphData::edge_type> iterator;

  typedef EdgeIterator<
  typename GraphData::edge_map_type::const_iterator,
  typename GraphData::edge_map_type::mapped_type::const_iterator,
  typename GraphData::edge_type const> const_iterator;

  EdgeIteratorWrapper(const std::shared_ptr<GraphData>& g):
    ptr_(g){}

  iterator begin(){
    return iterator(ptr_->out_edges.begin(),
                    ptr_->out_edges.end());
  }

  const_iterator begin()const{
    return const_iterator(ptr_->out_edges.cbegin(),
                          ptr_->out_edges.cend());
  }


  iterator end(){
    return iterator(ptr_->out_edges.end(),
                    ptr_->out_edges.end());
  }
  const_iterator end()const{
    return const_iterator(ptr_->out_edges.cend(),
                          ptr_->out_edges.cend());
  }

private:
  std::shared_ptr<GraphData> ptr_;
};

}


#endif // GRAPH_ITERATORS_HH
