#include "Graph.h"
#include <iostream>


Graph::Graph(int numVertices) : vertices(numVertices) {
    // Initialization of the adjacency matrix and adjacency list with zeros
    adjacencyMatrix.resize(vertices, std::vector<int>(vertices, 0));
    adjacencyList.resize(vertices);
}

// Print the adjacency matrix of the graph
void Graph::printAdjacencyMatrix() {
    std::cout << "Adjacency Matrix:\n";
    for (int i = 0; i < vertices; ++i) {
        for (int j = 0; j < vertices; ++j) {
            std::cout << adjacencyMatrix[i][j] << " ";
        }
        std::cout << "\n";
    }
}

// Print the adjacency list of the graph
void Graph::printAdjacencyList() {
    std::cout << "Adjacency List:\n";
    for (int i = 0; i < vertices; ++i) {
        std::cout << i << " -> ";
        for (int j : adjacencyList[i]) {
            std::cout << j << " ";
        }
        std::cout << "\n";
    }
}