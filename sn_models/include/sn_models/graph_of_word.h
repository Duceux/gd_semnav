#ifndef GRAPH_OF_WORD_H
#define GRAPH_OF_WORD_H

#include <sn_graph/sn_graph.hh>
#include <ros/ros.h>
#include <sn_dictionary/dico.h>

namespace sn {

typedef SimpleNode<Word> node_t;
typedef SimpleEdge<Word, false> edge_t;
typedef Graph<node_t, edge_t> GraphOfWordData;
typedef std::shared_ptr<GraphOfWordData> GraphOfWordDataPtr;

struct GraphOfWord{
    typedef std::shared_ptr<GraphOfWord> Ptr;

    GraphOfWord();

    std::string name;
    ros::Time uid;
    uint merged;
    double score;
    int visited;
    bool labeled;

    GraphOfWordData& get_graph(){return *graph;}

    GraphOfWordDataPtr graph;

    void add(Word const& w);

    static Ptr create();

    void print()const;

    int nb_words()const{return graph->nb_nodes();}
    int size()const{return graph->size();}

    std::map<std::string, int> type_map()const;

    static std::string type_of(edge_t const& e);
    static std::string type_of(node_t const& e);
    static std::map<std::string, int> type_map(GraphOfWordData const& graph);

    static GraphOfWord merge(GraphOfWord const& l, GraphOfWord const& r);
};

typedef GraphOfWord::Ptr GraphOfWordPtr;

}

#endif // GRAPH_OF_WORD_H
