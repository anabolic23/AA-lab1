#pragma once
#include <iostream>
#include <vector>


class Graph {
protected:
    int vertices; // Number of graph vertices
    std::vector<std::vector<int>> adjacencyMatrix; // Adjacency matrix
    std::vector<std::vector<int>> adjacencyList;   // Adjacency list

public:
    Graph(int numVertices);

    // Add a vertex to the graph
    virtual void addVertex() = 0;

    // Add an edge between vertices u and v
    virtual void addEdge(int u, int v) = 0;

    // Remove a vertex from the graph
    virtual void removeVertex(int v) = 0;

    // Remove an edge between vertices u and v
    virtual void removeEdge(int u, int v) = 0;

    // Generate a random graph with the given probability of connections between vertices
    virtual void generateRandomGraph(double probability) = 0;

    // Print the adjacency matrix of the graph
    void printAdjacencyMatrix();

    // Print the adjacency list of the graph
    void printAdjacencyList();
};