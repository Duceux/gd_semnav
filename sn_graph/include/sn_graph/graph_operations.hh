#ifndef GRAPH_OPERATIONS_HH
#define GRAPH_OPERATIONS_HH

#include <sn_graph/graph.hh>

namespace sn {

template<typename Graph>
Graph substraction(const Graph& g1, const Graph& g2){
    Graph res = g1.copy();

    for(auto const& edge: g2.get_edges()){
        res.remove(edge);
    }

    for(auto const& node: g2.get_nodes())
        if(res.nb_edges_of(node) == 0){
            res.remove(node);
        }

    assert(res.nb_nodes() <= std::max(g1.nb_nodes(),g2.nb_nodes()));
    assert(res.nb_nodes() >= g1.nb_nodes()-g2.nb_nodes());
    assert(res.nb_edges() <= std::max(g1.nb_edges(),g2.nb_edges()));
    assert(res.nb_edges() >= g1.nb_edges()-g2.nb_edges() || res.nb_edges()==0);
    return res;
}

template<typename G, typename... Args>
G substraction(const G& first, const Args...  args){
    return substraction(first, addition(args...));
}

template<typename Graph>
Graph addition(const Graph& g1, const Graph& g2){
    Graph bigger = g1;
    Graph smaller = g2;
    if(g2.size()>g1.size())
        bigger.swap(smaller);

    Graph res = bigger.copy();

    for(auto const& node: smaller.get_nodes())
        res.insert(node);
    for(auto const& edge: smaller.get_edges())
        res.insert(edge);

    assert(res.nb_nodes() <= g1.nb_nodes()+g2.nb_nodes());
    assert(res.nb_edges() <= g1.nb_edges()+g2.nb_edges());
    return res;
}

template<typename G, typename... Args>
G addition(const G& first, const Args...  args){
    return addition(first, addition(args...));
}

template<typename Graph>
Graph intersection(const Graph& g1, const Graph& g2){
    Graph bigger = g1;
    Graph smaller = g2;
    if(g2.size()>g1.size())
        bigger.swap(smaller);

    Graph res;

    for(auto const& node: smaller.get_nodes())
        if(bigger.has(node))
            res.insert(node);
    for(auto const& edge: smaller.get_edges())
        if(bigger.has(edge))
            res.insert(edge);

    assert(res.nb_nodes() <= std::min(g1.nb_nodes(),
                                      g2.nb_nodes()));
    assert(res.nb_edges() <= std::min(g1.nb_edges(),
                                      g2.nb_edges()));
    return res;
}

template<typename G, typename... Args>
G intersection(const G& first, const Args...  args){
    return intersection(first, intersection(args...));
}

template<typename Graph>
Graph difference(const Graph& g1, const Graph& g2){
    return substraction(addition(g1, g2),
                        intersection(g1, g2));
}

}//namespace graphml

#endif // GRAPH_OPERATIONS_HH
