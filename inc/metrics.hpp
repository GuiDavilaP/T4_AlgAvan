#ifndef METRICS_HPP
#define METRICS_HPP

#include <vector>
#include <chrono>

struct IterationMetrics {
    int iteration_number;
    double iteration_time_ms;
    int matching_size;              // Tamanho atual do matching
    double avg_potential_s;         // Potencial médio dos vértices em S
    double avg_potential_t;         // Potencial médio dos vértices em T
    int equality_edges;             // Número de arestas no subgrafo de igualdade
};

struct PathMetrics {
    double path_search_time_ms;  
    int path_length;                // Comprimento do caminho aumentante
    int dijkstra_iterations;        // Número de nós processados no Dijkstra
    int queue_operations;           // Operações push/pop na priority queue
    int potential_updates;          // Número de atualizações de distância
    double min_slack;               // Menor slack encontrado
    int tight_edges_count;          // Arestas no subgrafo de igualdade
};

struct ScalabilityMetrics {
    int graph_size;
    int edge_count;
    double total_time_ms;
    double avg_iteration_time_ms;   
    double time_per_vertex;         // Tempo por vértice
    double time_per_edge;           // Tempo por aresta
    int max_path_length;            // Maior caminho encontrado
    double convergence_rate;        // Taxa de convergência
    double avg_tree_depth;
    double potential_variance;
    double tight_edges_ratio;
};

struct AlgorithmMetrics {
    std::vector<IterationMetrics> iteration_metrics;
    std::vector<PathMetrics> path_metrics;
    ScalabilityMetrics scalability;
    int total_iterations;
    bool converged;
};

#endif