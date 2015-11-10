#include <gtest/gtest.h>
#include <sn_graph/graph.hh>
#include <iostream>

using namespace sn;

TEST(Graph, UndirectedSimpleNodeAndSimpleEdge)
{
    Graph<SimpleNode<std::string>,
            SimpleEdge<std::string, false>> graph;

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");

    EXPECT_TRUE(graph.at("hello") != graph.at("coucou"));
    EXPECT_EQ(2, graph.nb_nodes());
    EXPECT_EQ(1, graph.nb_edges());
    EXPECT_EQ(graph.at("hello", "coucou"), graph.at("coucou", "hello"));
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_TRUE(graph.has("coucou", "hello"));

    for(auto n: graph.get_nodes())
      std::cout << n << std::endl;
    for(auto n: graph.get_edges())
      std::cout << n << std::endl;

}

TEST(Graph, UndirectedDetachNode)
{
    Graph<SimpleNode<std::string>,
            SimpleEdge<std::string, false>> graph;

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");

    graph.detach("coucou");
    EXPECT_TRUE(graph.has("coucou"));
    EXPECT_EQ(graph.nb_out_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_edges_of("coucou"), 0);
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_EQ(graph.nb_out_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_edges_of("hello"), 0);
    graph.remove("coucou");
    EXPECT_FALSE(graph.has("coucou"));
    graph.remove("hello");
    graph.detach("hello");
    EXPECT_FALSE(graph.has("hello"));

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");
    graph.detach("hello");
    EXPECT_TRUE(graph.has("coucou"));
    EXPECT_EQ(graph.nb_out_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_edges_of("coucou"), 0);
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_EQ(graph.nb_out_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_edges_of("hello"), 0);
    graph.remove("coucou");
    EXPECT_FALSE(graph.has("coucou"));
    graph.remove("hello");
    graph.detach("hello");
    EXPECT_FALSE(graph.has("hello"));
}

TEST(Graph, DirectedDetachNode)
{
    Graph<SimpleNode<std::string>,
            SimpleEdge<std::string, true>> graph;

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");

    graph.detach("coucou");
    EXPECT_TRUE(graph.has("coucou"));
    EXPECT_EQ(graph.nb_out_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_edges_of("coucou"), 0);
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_EQ(graph.nb_out_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_edges_of("hello"), 0);
    graph.remove("coucou");
    EXPECT_FALSE(graph.has("coucou"));
    graph.remove("hello");
    graph.detach("hello");
    EXPECT_FALSE(graph.has("hello"));

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");
    graph.detach("hello");
    EXPECT_TRUE(graph.has("coucou"));
    EXPECT_EQ(graph.nb_out_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("coucou"), 0);
    EXPECT_EQ(graph.nb_edges_of("coucou"), 0);
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_EQ(graph.nb_out_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_in_edges_of("hello"), 0);
    EXPECT_EQ(graph.nb_edges_of("hello"), 0);
    graph.remove("coucou");
    EXPECT_FALSE(graph.has("coucou"));
    graph.remove("hello");
    graph.detach("hello");
    EXPECT_FALSE(graph.has("hello"));

}

TEST(Graph, DirectedSimpleNodeAndSimpleEdge)
{
    Graph<SimpleNode<std::string>,
            SimpleEdge<std::string, true>> graph;

    graph.insert_node("coucou");
    graph.insert_node("hello");
    graph.insert_edge("hello", "coucou");

    EXPECT_TRUE(graph.at("hello") != graph.at("coucou"));
    EXPECT_EQ(2, graph.nb_nodes());
    EXPECT_EQ(1, graph.nb_edges());
    EXPECT_ANY_THROW(graph.at("coucou", "hello"));
    EXPECT_TRUE(graph.has("hello"));
    EXPECT_FALSE(graph.has("coucou", "hello"));

    for(auto n: graph.get_nodes())
      std::cout << n << std::endl;
    for(auto n: graph.get_edges())
      std::cout << n << std::endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
