#pragma once
#include "Graph.h"
#include <iostream>
#include <vector>


class UndirectedGraph : public Graph {
public:
    UndirectedGraph(int numVertices);
    void generateRandomGraph(double probability) override;
    void addVertex() override;
    void addEdge(int u, int v) override;
    void removeVertex(int v) override;
    void removeEdge(int u, int v) override;
    void convertMatrixToList();
};

class DirectedGraph : public Graph {
public:
    DirectedGraph(int numVertices);
    void generateRandomGraph(double probability) override;
    void addVertex() override;
    void addEdge(int u, int v) override;
    void removeVertex(int v) override;
    void removeEdge(int u, int v) override;
    void convertMatrixToList(); 
};

class WeightedGraph : public Graph {
protected:
    std::vector<std::vector<int>> weights;
    double minWeight;
    double maxWeight;


public:
    WeightedGraph(int numVertices, double min, double max);
    void addWeightedEdge(int u, int v, int w);
    void removeWeightedEdge(int u, int v);
    int getWeight(int u, int v) const;
    void addVertex() override;
    void addEdge(int u, int v) override;

    void removeVertex(int v) override;
    void removeEdge(int u, int v) override;
    void generateRandomGraph(double probability, int min, int max);
    void generateRandomGraph(double probability) override;
    void convertMatrixToList();
    WeightedGraph* primMST();
    WeightedGraph* kruskalMST();

};
