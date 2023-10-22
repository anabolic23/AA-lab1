#include "Graphs.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <climits>
#include <queue>
#include <utility>

using namespace std;

UndirectedGraph::UndirectedGraph(int numVertices) : Graph(numVertices) {}

void UndirectedGraph::generateRandomGraph(double probability) {
    srand(static_cast<unsigned int>(time(nullptr)));

    for (int i = 1; i < vertices; ++i) {
        int randomVertex = rand() % i;
        addEdge(i, randomVertex);
    }
    for (int i = 0; i < vertices; ++i) {
        for (int j = i; j < vertices; ++j) {
            if (i != j) {
                double randomValue = static_cast<double>(rand()) / RAND_MAX;

                if (randomValue <= probability) {
                    addEdge(i, j);

                }
            }
        }
    }
}

// Add a vertex to the undirected graph
void UndirectedGraph::addVertex() {
    ++vertices;
    adjacencyMatrix.resize(vertices, std::vector<int>(vertices, 0));
    adjacencyList.resize(vertices);
}

// Add an edge between vertices u and v
void UndirectedGraph::addEdge(int u, int v){
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = 1;
        adjacencyMatrix[v][u] = 1;

    }
}

// Remove a vertex from the undirected graph
void UndirectedGraph::removeVertex(int v) {
    if (v >= 0 && v < vertices) {
        // Removing vertex v from the adjacency matrix
        adjacencyMatrix.erase(adjacencyMatrix.begin() + v);
        for (auto& row : adjacencyMatrix) {
            row.erase(row.begin() + v);
        }

        // Removing vertex v from the adjacency list of other vertices
        adjacencyList.erase(adjacencyList.begin() + v);

        // Decreasing the number of vertices
        --vertices;
    }
}

// Remove an edge between vertices u and v
void UndirectedGraph::removeEdge(int u, int v) {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = 0;
        adjacencyMatrix[v][u] = 0;

        // Removing v from the adjacency list of vertex u 
        adjacencyList[u].erase(std::remove(adjacencyList[u].begin(), adjacencyList[u].end(), v), adjacencyList[u].end());
        adjacencyList[v].erase(std::remove(adjacencyList[v].begin(), adjacencyList[v].end(), u), adjacencyList[v].end());
    }
}
void UndirectedGraph::convertMatrixToList() {
    // Clear the adjacency list before converting the matrix
    for (int i = 0; i < vertices; ++i) {
        adjacencyList[i].clear();
    }

    // Add vertices to the adjacency list from the adjacency matrix
    for (int i = 0; i < vertices; ++i) {
        for (int j = i; j < vertices; ++j) {
            if (adjacencyMatrix[i][j] == 1) {
                adjacencyList[i].push_back(j);
                adjacencyList[j].push_back(i); // If the graph is undirected, add reverse edges as well
            }
        }
    }
}


DirectedGraph::DirectedGraph(int numVertices) : Graph(numVertices) {}

void DirectedGraph::generateRandomGraph(double probability) {
    srand(static_cast<unsigned int>(time(nullptr)));

    for (int i = 0; i < vertices; ++i) {
        for (int j = 0; j < vertices; ++j) {
            if (i != j) {
                double randomValue = static_cast<double>(rand()) / RAND_MAX;

                if (randomValue <= probability) {
                    addEdge(i, j);

                }
            }
        }
    }
}
// Add a vertex to the directed graph
void DirectedGraph::addVertex() {
    ++vertices;
    adjacencyMatrix.resize(vertices, std::vector<int>(vertices, 0));
    adjacencyList.resize(vertices);
}

// Add an edge from vertex u to vertex v
void DirectedGraph::addEdge(int u, int v) {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = 1;
    }
}

// Remove a vertex from the directed graph
void DirectedGraph::removeVertex(int v) {
    if (v >= 0 && v < vertices) {
        // Removing vertex v from the adjacency matrix
        adjacencyMatrix.erase(adjacencyMatrix.begin() + v);
        for (auto& row : adjacencyMatrix) {
            row.erase(row.begin() + v);
        }

        // Removing vertex v from the adjacency list of other vertices
        for (auto& list : adjacencyList) {
            list.erase(std::remove(list.begin(), list.end(), v), list.end());
        }

        // Decreasing the number of vertices
        --vertices;
    }
}

// Remove an edge from vertex u to vertex v
void DirectedGraph::removeEdge(int u, int v) {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = 0;
        adjacencyList[u].erase(std::remove(adjacencyList[u].begin(), adjacencyList[u].end(), v), adjacencyList[u].end());
    }
}
void DirectedGraph::convertMatrixToList() {
    // Clear the adjacency list before converting the matrix
    for (int i = 0; i < vertices; ++i) {
        adjacencyList[i].clear();
    }

    // Add vertices to the adjacency list from the adjacency matrix
    for (int i = 0; i < vertices; ++i) {
        for (int j = 0; j < vertices; ++j) {
            if (adjacencyMatrix[i][j] == 1) {
                adjacencyList[i].push_back(j);
            }
        }
    }
}

WeightedGraph::WeightedGraph(int numVertices, double min, double max) : Graph(numVertices) {
    // Initialization of the edge weight matrix with zeros
    weights.resize(vertices, std::vector<int>(vertices, INT_MAX));
    minWeight = min;
    maxWeight = max;
}
// Add an edge between vertices u and v with weight w
void WeightedGraph::addWeightedEdge(int u, int v, int w) {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = w;
        weights[u][v] = w;
    }
}

// Remove an edge between vertices u and v
void WeightedGraph::removeWeightedEdge(int u, int v) {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        adjacencyMatrix[u][v] = 0;
        weights[u][v] = 0;
    }
}

// Get the weight of the edge between vertices u and v
int WeightedGraph::getWeight(int u, int v) const {
    if (u >= 0 && u < vertices && v >= 0 && v < vertices) {
        return weights[u][v];
    }
    return 0; // Return 0 if the vertices or edge do not exist
}

// Add a vertex to the graph
void WeightedGraph::addVertex() {
    ++vertices;
    adjacencyMatrix.resize(vertices, std::vector<int>(vertices, 0));
    adjacencyList.resize(vertices);
    weights.resize(vertices, std::vector<int>(vertices, 0));
}

// Add an edge between vertices u and v
void WeightedGraph::addEdge(int u, int v) {}

// Remove a vertex from the graph
void WeightedGraph::removeVertex(int v) {}

// Remove an edge between vertices u and v
void WeightedGraph::removeEdge(int u, int v)  {}

// Generate a random graph with the specified connection probability and weight range
void WeightedGraph::generateRandomGraph(double probability, int min, int max) {
    srand(static_cast<unsigned int>(time(nullptr)));

    for (int i = 0; i < vertices; ++i) {
        for (int j = i; j < vertices; ++j) {
            if (i != j) {
                double randomValue = static_cast<double>(rand()) / RAND_MAX;
                if (randomValue <= probability) {
                    int randomWeight = min + rand() % max;
                    addWeightedEdge(i, j, randomWeight);
                    addWeightedEdge(j, i, randomWeight);
                }
            }
        }
    }
}
void WeightedGraph::generateRandomGraph(double probability) {
    srand(static_cast<unsigned int>(time(nullptr)));

    for (int i = 0; i < vertices; ++i) {
        for (int j = i; j < vertices; ++j) {
            if (i != j) {
                double randomValue = static_cast<double>(rand()) / RAND_MAX;
                if (randomValue <= probability) {
                    int randomWeight = 1 + rand() % 100;
                    addWeightedEdge(i, j, randomWeight);
                    addWeightedEdge(j, i, randomWeight);
                }
            }
        }
    }
}
void WeightedGraph::convertMatrixToList() {
    // Clear the adjacency list
    for (int i = 0; i < vertices; ++i) {
        adjacencyList[i].clear();
    }

    // Add vertices to the adjacency list from the adjacency matrix
    for (int i = 0; i < vertices; ++i) {
        for (int j = 0; j < vertices; ++j) {
            if (adjacencyMatrix[i][j] > 0) { // Only add edges with positive weights
                adjacencyList[i].push_back(j);
            }
        }
    }
}

struct VertexInfo {
    int vertex;
    int weight;
    // Custom comparison function for the priority queue
    bool operator>(const VertexInfo& other) const {
        return weight > other.weight;
    }
};

WeightedGraph* WeightedGraph::primMST() {
    WeightedGraph* mst = new WeightedGraph(vertices, minWeight, maxWeight);

    vector<bool> inMST(vertices, false);
    vector<int> minWeight(vertices, INT_MAX);
    vector<int> parent(vertices, -1);

    priority_queue<VertexInfo, vector<VertexInfo>, greater<VertexInfo>> pq;

    // Start with a random vertex
    int startVertex = rand() % vertices;
    pq.push({ startVertex, 0 });

    while (!pq.empty()) {
        VertexInfo current = pq.top();
        pq.pop();
        int u = current.vertex;

        if (inMST[u]) {
            continue;  // Skip if already in MST
        }

        inMST[u] = true;

        if (u != startVertex) {
            int weightToMST = minWeight[u];
            mst->addWeightedEdge(u, parent[u], weightToMST);
            mst->addWeightedEdge(parent[u], u, weightToMST);
        }

        for (int v = 0; v < vertices; ++v) {
            int weightUV = getWeight(u, v);
            if (!inMST[v] && weightUV < minWeight[v]) {
                minWeight[v] = weightUV;
                parent[v] = u;
                pq.push({ v, weightUV });
            }
        }
    }
    mst->convertMatrixToList();
    return mst;
}

int findParent(int vertex, vector<int>& parent) {
    if (parent[vertex] == -1)
        return vertex;
    return findParent(parent[vertex], parent);
}

WeightedGraph* WeightedGraph::kruskalMST() {
    WeightedGraph* mst = new WeightedGraph(vertices, minWeight, maxWeight);
    vector<pair<int, pair<int, int>>> edges; // Store edges with their weights

    // Populate the 'edges' vector with all edges and their weights
    for (int i = 0; i < vertices; ++i) {
        for (int j = i + 1; j < vertices; ++j) {
            int weight = getWeight(i, j);
            if (weight != 0) {
                edges.push_back({ weight, {i, j} });
            }
        }
    }

    // Sort the edges by weight in non-decreasing order
    sort(edges.begin(), edges.end());

    vector<int> parent(vertices, -1);

    for (const auto& edge : edges) {
        int weight = edge.first;
        int u = edge.second.first;
        int v = edge.second.second;

        int parentU = findParent(u, parent);
        int parentV = findParent(v, parent);

        // If including this edge doesn't create a cycle, add it to the MST
        if (parentU != parentV) {
            mst->addWeightedEdge(u, v, weight);
            mst->addWeightedEdge(v, u, weight);
            parent[parentU] = parentV;
        }
    }

    mst->convertMatrixToList();
    return mst;
}

