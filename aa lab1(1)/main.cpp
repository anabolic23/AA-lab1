#include "Graph.h"
#include "Graphs.h"
#include <iostream>
#include <chrono>

using namespace std;

int main() {
    int numVertices;
    double probability;

    // Enter the number of vertices and the probability of connection between vertices
    cout << "Enter the number of vertices: ";
    cin >> numVertices;

    if (numVertices <= 0) {
        cout << "Number of vertices should be positive." << endl;
        return 1;
    }

    cout << "Enter the probability (0 to 1): ";
    cin >> probability;

    if (probability < 0 || probability > 1) {
        cout << "Probability should be between 0 and 1." << endl;
        return 1;
    }

    DirectedGraph* directedGraph = nullptr;
    UndirectedGraph* undirectedGraph = nullptr;
    WeightedGraph* weightedGraph = nullptr;

    int choice;
    cout << "Choose the type of graph:" << endl;
    cout << "1. Undirected Graph" << endl;
    cout << "2. Directed Graph" << endl;
    cout << "3. Weighted Graph" << endl;
    cout << "Enter your choice (1/2/3): ";
    cin >> choice;

    switch (choice) {
    case 1:
        undirectedGraph = new UndirectedGraph(numVertices);
        break;
    case 2:
        directedGraph = new DirectedGraph(numVertices);
        break;
    case 3:
        weightedGraph = new WeightedGraph(numVertices, 1, 100); // Adjust min and max weights as needed
        break;
    default:
        cout << "Invalid choice. Exiting." << endl;
        return 1;
    }


    if (choice == 1) {
        undirectedGraph->generateRandomGraph(probability);
        undirectedGraph->printAdjacencyMatrix();
        cout << "To show that adjacency list is empty" << endl;
        undirectedGraph->printAdjacencyList();
        cout << "To show that function (convertMatrixToList) works currectly" << endl;
        undirectedGraph->convertMatrixToList();
        undirectedGraph->printAdjacencyList();
    }

    if (choice == 2) {
        directedGraph->generateRandomGraph(probability);
        directedGraph->printAdjacencyMatrix();
        cout << "To show that adjacency list is empty" << endl;
        directedGraph->printAdjacencyList();
        cout << "To show that function (convertMatrixToList) works currectly" << endl;
        directedGraph->convertMatrixToList();
        directedGraph->printAdjacencyList();
    }

    if (choice == 3) {
        weightedGraph->generateRandomGraph(probability);
        weightedGraph->printAdjacencyMatrix();

        auto startTime = std::chrono::high_resolution_clock::now();
        WeightedGraph* primMST = weightedGraph->primMST();
        auto endTime = std::chrono::high_resolution_clock::now();

        if (primMST) {
            cout << "Performing Prim's MST algorithm..." << endl;
            primMST->printAdjacencyMatrix();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            cout << "Prim's algorithm execution time: " << duration.count() << " microseconds" << endl;
        }

        startTime = std::chrono::high_resolution_clock::now();
        WeightedGraph* kruskalMST = weightedGraph->kruskalMST();
        endTime = std::chrono::high_resolution_clock::now();

        if (kruskalMST) {
            cout << "Performing Kruskal's MST algorithm..." << endl;
            kruskalMST->printAdjacencyMatrix();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            cout << "Kruskal's algorithm execution time: " << duration.count() << " microseconds" << endl;
        }
    }
}
