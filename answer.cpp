#include <iostream>
#include <vector>
#include <cassert>
#include <functional>
#include <queue>
#include <deque>
#include <unordered_set>
#include <set>

struct IGraph {
    virtual ~IGraph() {}

    virtual void AddEdge(int from, int to) = 0;

    virtual int VerticesCount() const = 0;

    virtual std::vector<int> GetNextVertices(int vertex) const = 0;
    virtual std::vector<int> GetPrevVertices(int vertex) const = 0;
};

struct ListGraph: public IGraph {
public:
    ListGraph(int size): adjacencyLists(size) {}

    ListGraph(const IGraph &graph) {
        adjacencyLists.resize(graph.VerticesCount());
        for (int i = 0; i < graph.VerticesCount(); ++i) {
            adjacencyLists[i] = graph.GetNextVertices(i);
        }
    }

    ~ListGraph() {}

    void AddEdge(int from, int to) override {
        assert(0 <= from && from < adjacencyLists.size());
        assert(0 <= to && to < adjacencyLists.size());
        adjacencyLists[from].push_back(to);
    }

    int VerticesCount() const override {
        return (int)adjacencyLists.size();
    }

    std::vector<int> GetNextVertices(int vertex) const override {
        assert(0 <= vertex && vertex < adjacencyLists.size());
        return adjacencyLists[vertex];
    }

    std::vector<int> GetPrevVertices(int vertex) const override {
        assert(0 <= vertex && vertex < adjacencyLists.size());
        std::vector<int> prevVertices;

        for (int from = 0; from < adjacencyLists.size(); ++from) {
            for (int to: adjacencyLists[from]) {
                if (to == vertex)
                    prevVertices.push_back(from);
            }
        }
        return prevVertices;
    }

private:
    std::vector<std::vector<int>> adjacencyLists;
};

class MatrixGraph: public IGraph {
public:
    MatrixGraph(int size): adjacencyMatrix(size, std::vector<bool>(size, false)) {}

    MatrixGraph(const IGraph &graph) {
        int size = graph.VerticesCount();
        adjacencyMatrix.resize(size, std::vector<bool>(size, false));
        for (int from = 0; from < size; ++from) {
            for (int to: graph.GetNextVertices(from)) {
                adjacencyMatrix[from][to] = true;
            }
        }
    }

    void AddEdge(int from, int to) override {
        assert(0 <= from && from < VerticesCount());
        assert(0 <= to && to < VerticesCount());
        adjacencyMatrix[from][to] = true;
    }

    int VerticesCount() const override {
        return adjacencyMatrix.size();
    }

    std::vector<int> GetNextVertices(int vertex) const override {
        assert(0 <= vertex && vertex < VerticesCount());
        std::vector<int> nextVertices;
        for (int to = 0; to < VerticesCount(); ++to) {
            if (adjacencyMatrix[vertex][to]) {
                nextVertices.push_back(to);
            }
        }
        return nextVertices;
    }

    std::vector<int> GetPrevVertices(int vertex) const override {
        assert(0 <= vertex && vertex < VerticesCount());
        std::vector<int> prevVertices;
        for (int from = 0; from < VerticesCount(); ++from) {
            if (adjacencyMatrix[from][vertex]) {
                prevVertices.push_back(from);
            }
        }
        return prevVertices;
    }

private:
    std::vector<std::vector<bool>> adjacencyMatrix;
};

class SetGraph: public IGraph {
public:
    SetGraph(int size) : adjacencySets(size) {}

    SetGraph(const IGraph& graph) {
        adjacencySets.resize(graph.VerticesCount());
        for (int from = 0; from < graph.VerticesCount(); ++from) {
            for (int to: graph.GetNextVertices(from)) {
                adjacencySets[from].insert(to);
            }
        }
    }

    void AddEdge(int from, int to) override {
        assert(0 <= from && from < VerticesCount());
        assert(0 <= to && to < VerticesCount());
        adjacencySets[from].insert(to);
    }

    int VerticesCount() const override {
        return adjacencySets.size();
    }

    std::vector<int> GetNextVertices(int vertex) const override {
        assert(0 <= vertex && vertex < VerticesCount());
        return std::vector<int>(adjacencySets[vertex].begin(), adjacencySets[vertex].end());
    }

    std::vector<int> GetPrevVertices(int vertex) const override {
        assert(0 <= vertex && vertex < VerticesCount());
        std::vector<int> prevVertices;
        for (int from = 0; from < VerticesCount(); ++from) {
            if (adjacencySets[from].count(vertex) > 0) {
                prevVertices.push_back(from);
            }
        }
        return prevVertices;
    }

private:
    std::vector<std::unordered_set<int>> adjacencySets;
};

class ArcGraph: public IGraph {
public:
    ArcGraph(int size): verticesCount(size) {}

    ArcGraph(const IGraph &graph) {
        verticesCount = graph.VerticesCount();
        for (int from = 0; from < verticesCount; ++from) {
            for (int to: graph.GetNextVertices(from)) {
                edges.emplace_back(from, to);
            }
        }
    }

    void AddEdge(int from, int to) override {
        assert(0 <= from && from < verticesCount);
        assert(0 <= to && to < verticesCount);
        edges.emplace_back(from, to);
    }

    int VerticesCount() const override {
        return verticesCount;
    }

    std::vector<int> GetNextVertices(int vertex) const override {
        assert(0 <= vertex && vertex < verticesCount);
        std::vector<int> nextVertices;
        for (const auto& edge: edges) {
            if (edge.first == vertex) {
                nextVertices.push_back(edge.second);
            }
        }
        return nextVertices;
    }

    std::vector<int> GetPrevVertices(int vertex) const override {
        assert(0 <= vertex && vertex < verticesCount);
        std::vector<int> prevVertices;
        for (const auto& edge: edges) {
            if (edge.second == vertex) {
                prevVertices.push_back(edge.first);
            }
        }
        return prevVertices;
    }

private:
    int verticesCount;
    std::vector<std::pair<int, int>> edges;
};

void BFS(const IGraph &graph, int vertex, std::vector<bool> &visited, const std::function<void(int)> &func) {
    std::queue<int> qu;
    qu.push(vertex);
    visited[vertex] = true;

    while (!qu.empty()) {
        int currentVertex = qu.front();
        qu.pop();

        func(currentVertex);

        for (int nextVertex: graph.GetNextVertices(currentVertex)) {
            if (!visited[nextVertex]) {
                visited[nextVertex] = true;
                qu.push(nextVertex);
            }
        }
    }
}

void mainBFS(const IGraph &graph, const std::function<void(int)> &func) {
    std::vector<bool> visited(graph.VerticesCount(), false);

    for (int i = 0; i < graph.VerticesCount(); ++i) {
        if (!visited[i]) {
            BFS(graph, i, visited, func);
        }
    }
}

void DFS(const IGraph &graph, int vertex, std::vector<bool> &visited, const std::function<void(int)> &func) {
    visited[vertex] = true;
    func(vertex);

    for (int nextVertex: graph.GetNextVertices(vertex)) {
        if (!visited[nextVertex]) {
            DFS(graph, nextVertex, visited, func);
        }
    }
}

void mainDFS(const IGraph &graph, const std::function<void(int)> &func) {
    std::vector<bool> visited(graph.VerticesCount(), false);

    for (int i = 0; i < graph.VerticesCount(); ++i) {
        if (!visited[i]) {
            DFS(graph, i, visited, func);
        }
    }
}

void topologicalSortInternal(const IGraph &graph, int vertex, std::vector<bool> &visited, std::deque<int> &sorted) {
    visited[vertex] = true;

    for (int nextVertex: graph.GetNextVertices(vertex)) {
        if (!visited[nextVertex]) {
            topologicalSortInternal(graph, nextVertex, visited, sorted);
        }
    }

    sorted.push_front(vertex);
}

std::deque<int> topologicalSort(const IGraph &graph) {
    std::deque<int> sorted;
    std::vector<bool> visited(graph.VerticesCount(), false);

    for (int i = 0; i < graph.VerticesCount(); ++i) {
        if (!visited[i]) {
            topologicalSortInternal(graph, i, visited, sorted);
        }
    }

    return sorted;
}

int main() {
    ListGraph listGraph(7);
    listGraph.AddEdge(0, 1);
    listGraph.AddEdge(0, 5);
    listGraph.AddEdge(1, 2);
    listGraph.AddEdge(1, 3);
    listGraph.AddEdge(1, 5);
    listGraph.AddEdge(1, 6);
    listGraph.AddEdge(3, 2);
    listGraph.AddEdge(3, 4);
    listGraph.AddEdge(3, 6);
    listGraph.AddEdge(5, 4);
    listGraph.AddEdge(5, 6);
    listGraph.AddEdge(6, 4);

    std::cout << "ListGraph:" << std::endl;
    std::cout << "BFS: ";
    mainBFS(listGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << "DFS: ";
    mainDFS(listGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << "Topological sort: ";
    for (int vertex : topologicalSort(listGraph)) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl << std::endl;

    MatrixGraph matrixGraph(listGraph);
    std::cout << "MatrixGraph:" << std::endl;
    std::cout << "BFS: ";
    mainBFS(matrixGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << "DFS: ";
    mainDFS(matrixGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << std::endl;

    ArcGraph arcGraph(matrixGraph);
    std::cout << "ArcGraph:" << std::endl;
    std::cout << "BFS: ";
    mainBFS(arcGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << "DFS: ";
    mainDFS(arcGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << std::endl;

    SetGraph setGraph(arcGraph);
    std::cout << "SetGraph:" << std::endl;
    std::cout << "BFS: ";
    mainBFS(setGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl << "DFS: ";
    mainDFS(setGraph, [](int vertex) { std::cout << vertex << " "; });
    std::cout << std::endl;

    return 0;
}
