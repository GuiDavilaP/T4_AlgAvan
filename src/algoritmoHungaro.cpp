#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <limits>
#include <algorithm>
#include <chrono>
#include <iomanip>   // Para formatação de output
#include <sstream>
#include <fstream>
#include "../inc/readGraph.hpp"
#include "../inc/metrics.hpp"

using namespace std;

class HungarianAlgorithm {
private:
    struct Edge {
        string to;
        int weight;
        Edge(const string& t, int w) : to(t), weight(w) {}
    };
    
    // Grafo representado como lista de adjacência
    map<string, vector<Edge>> graph;
    
    // Conjuntos S e T
    set<string> S, T;
    
    // Peso máximo
    int W;
    
    // Potenciais dos vértices
    map<string, int> potential;
    
    // Emparelhamento atual
    map<string, string> matching;           // matching[u] = v
    map<string, string> reverse_matching;   // reverse_matching[v] = u
    
    // Conjuntos de vértices livres
    set<string> S0, T0;
    
    // Todas as arestas
    vector<tuple<string, string, int>> edges;

    // Métricas e tempo
    std::chrono::time_point<std::chrono::high_resolution_clock> algorithm_start;
    AlgorithmMetrics metrics;

    // Calcular slack de uma aresta
    double calculate_slack(const string& u, const string& v, int original_weight) {
        return potential[u] + potential[v] - original_weight;
    }

    // Contar arestas no subgrafo de igualdade (tight edges)
    int count_tight_edges() {
        int count = 0;
        const double EPSILON = 1e-9;
        
        for (const auto& [u, adj_list] : graph) {
            for (const Edge& edge : adj_list) {
                double slack = calculate_slack(u, edge.to, edge.weight);
                if (abs(slack) < EPSILON) {  // Aresta tight
                    count++;
                }
            }
        }
        return count;
    }

    // Encontrar menor slack entre todas as arestas
    double find_min_slack() {
        double min_slack = numeric_limits<double>::max();
        
        for (const auto& [u, adj_list] : graph) {
            for (const Edge& edge : adj_list) {
                double slack = calculate_slack(u, edge.to, edge.weight);
                min_slack = min(min_slack, slack);
            }
        }
        return min_slack;
    }

    // Calcular potencial médio para um conjunto de vértices
    double calculate_avg_potential(const set<string>& vertices) {
        if (vertices.empty()) return 0.0;
        
        double sum = 0.0;
        for (const string& v : vertices) {
            sum += potential[v];
        }
        return sum / vertices.size();
    }

    // Coletar métricas da iteração
    IterationMetrics collect_iteration_metrics(int iteration) {
        IterationMetrics im;
        im.iteration_number = iteration;
        im.matching_size = matching.size();
        im.avg_potential_s = calculate_avg_potential(S);
        im.avg_potential_t = calculate_avg_potential(T);
        im.equality_edges = count_tight_edges();
        return im;
    }

    // Coletar métricas do caminho encontrado
    PathMetrics collect_path_metrics(const vector<string>& path, int dijkstra_iters, 
                               int queue_ops, int potential_upd) {
        PathMetrics pm;
        pm.path_length = path.size();
        pm.dijkstra_iterations = dijkstra_iters;
        pm.queue_operations = queue_ops;
        pm.potential_updates = potential_upd;
        pm.min_slack = find_min_slack();
        pm.tight_edges_count = count_tight_edges();
        
        return pm;
    }

    // Print status da iteração
    // (para depuração e visualização)
    void print_iteration_status(int iteration, const vector<string>& path) {
        cout << "\n--- Iteração " << (iteration + 1) << " ---\n";
        cout << "Vértices livres em S: ";
        for (const string& s : S0) cout << s << " ";
        cout << "\nVértices livres em T: ";
        for (const string& t : T0) cout << t << " ";
        cout << "\n";
        
        if (path.empty()) {
            cout << "Nenhum caminho aumentante encontrado!\n";
        } else {
            cout << "Caminho aumentante: ";
            for (size_t i = 0; i < path.size(); i++) {
                cout << path[i];
                if (i < path.size() - 1) cout << " -> ";
            }
            cout << "\n";
        }
        
        cout << "Emparelhamento atual: ";
        for (const auto& [u, v] : matching) {
            cout << "(" << u << "," << v << ") ";
        }
        cout << "\n";
    }

    // Print arestas do emparelhamento final
    void print_pairing_edges(const vector<tuple<string, string, int>>& matching_edges) {
        cout << "\n--- Arestas do emparelhamento final ---\n";
        if (matching_edges.empty()) {
            cout << "Nenhum emparelhamento encontrado.\n";
            return;
        } 
        for (const auto& [u, v, weight] : matching_edges) {
            cout << "  " << u << " -> " << v << " (peso: " << -weight << ")\n";
        }
    }
        

public:
    HungarianAlgorithm(const map<string, vector<pair<string, int>>>& input_graph) {
        W = 0;
        
        // Converter grafo de entrada e encontrar peso máximo
        for (const auto& [u, adj_list] : input_graph) {
            S.insert(u);
            for (const auto& [v, weight] : adj_list) {
                T.insert(v);
                graph[u].emplace_back(v, weight);
                edges.emplace_back(u, v, weight);
                W = max(W, weight);
            }
        }
        
        // Inicializar conjuntos de vértices livres
        S0 = S;
        T0 = T;
    }
    
    void initialize_potentials() {
        // Para maximização com pesos negados:
        // Para todo vértice v ∈ S: p_v := 0
        for (const string& v : S) {
            potential[v] = 0;
        }
        
        // Para todo vértice v ∈ T: p_v := W (positivo, pois negamos os pesos)
        for (const string& v : T) {
            potential[v] = W;
        }
    }
    
    int get_transformed_weight(const string& u, const string& v, int original_weight) {
        // Para maximização: negamos o peso e depois aplicamos transformação
        // d'_uv = -d_uv - (p_v - p_u)
        return -original_weight - (potential[v] - potential[u]);
    }
    
    tuple<map<string, int>, map<string, string>, int, int, int> dijkstra_from_s0() {
        map<string, int> distances;
        map<string, string> predecessors;
        set<string> visited;
        int dijkstra_iterations = 0;
        int queue_operations = 0;
        int potential_updates = 0;
        
        // Inicializar distâncias
        for (const string& v : S) {
            distances[v] = numeric_limits<int>::max();
        }
        for (const string& v : T) {
            distances[v] = numeric_limits<int>::max();
        }
        
        // Priority queue: (distância, vértice)
        priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq;
        
        // Inicializar distâncias para vértices livres em S
        for (const string& s : S0) {
            distances[s] = 0;
            pq.push({0, s});
            queue_operations++;
        }
        
        while (!pq.empty()) {
            auto [current_dist, u] = pq.top();
            pq.pop();
            queue_operations++;
            dijkstra_iterations++;
            
            if (visited.count(u)) {
                continue;
            }
            visited.insert(u);

            if (current_dist > distances[u]) {
                continue;
            }
            
            // Explorar arestas do grafo original (S -> T)
            if (graph.count(u)) {
                for (const Edge& edge : graph[u]) {
                    const string& v = edge.to;
                    int original_weight = edge.weight;
                    
                    // Só considerar se v não está emparelhado OU se esta aresta não faz parte do matching
                    if (!reverse_matching.count(v) || reverse_matching[v] != u) {
                        int transformed_weight = get_transformed_weight(u, v, original_weight);
                        int new_dist = distances[u] + transformed_weight;
                        
                        if (new_dist < distances[v]) {
                            distances[v] = new_dist;
                            predecessors[v] = u;
                            pq.push({new_dist, v});
                            queue_operations++;
                            potential_updates++;
                        }
                    }
                }
            }
            
            // Explorar arestas do matching (T -> S) com peso 0
            if (reverse_matching.count(u)) {
                const string& matched_vertex = reverse_matching[u];
                int new_dist = distances[u] + 0;  // Peso 0 para arestas do matching
                
                if (new_dist < distances[matched_vertex]) {
                    distances[matched_vertex] = new_dist;
                    predecessors[matched_vertex] = u;
                    pq.push({new_dist, matched_vertex});
                    queue_operations++;
                    potential_updates++;
                }
            }
        }
        
        return {distances, predecessors, dijkstra_iterations, queue_operations, potential_updates};
    }
    
    pair<string, vector<string>> find_augmenting_path(const map<string, int>& distances, 
                                                     const map<string, string>& predecessors) {
        // Encontrar vértice livre em T com menor distância
        int min_dist = numeric_limits<int>::max();
        string target = "";
        
        for (const string& t : T0) {
            if (distances.count(t) && distances.at(t) < min_dist) {
                min_dist = distances.at(t);
                target = t;
            }
        }
        
        if (target.empty() || min_dist == numeric_limits<int>::max()) {
            return {"", {}};
        }
        
        // Reconstruir caminho
        vector<string> path;
        string current = target;
        
        while (predecessors.count(current)) {
            path.push_back(current);
            current = predecessors.at(current);
        }
        path.push_back(current);  // Adicionar vértice inicial
        reverse(path.begin(), path.end());
        
        return {target, path};
    }
    
    void augment_matching(const vector<string>& path) {
        // O caminho alterna entre vértices de S e T
        // Arestas em posições pares (0, 2, 4, ...) devem ser adicionadas ao matching
        // Arestas em posições ímpares (1, 3, 5, ...) devem ser removidas do matching
        
        for (size_t i = 0; i < path.size() - 1; i++) {
            const string& u = path[i];
            const string& v = path[i + 1];
            
            if (i % 2 == 0) {  // Posição par: adicionar ao matching
                matching[u] = v;
                reverse_matching[v] = u;
            } else {  // Posição ímpar: remover do matching (implicitamente tratado)
                if (matching.count(u)) {
                    string old_match = matching[u];
                    matching.erase(u);
                    if (reverse_matching.count(old_match)) {
                        reverse_matching.erase(old_match);
                    }
                }
            }
        }
        
        // Atualizar conjuntos de vértices livres
        const string& start_vertex = path[0];
        const string& end_vertex = path.back();
        
        S0.erase(start_vertex);
        T0.erase(end_vertex);
    }
    
    void update_potentials(const map<string, int>& distances) {
        for (const string& v : S) {
            if (distances.count(v) && distances.at(v) != numeric_limits<int>::max()) {
                potential[v] += distances.at(v);
            }
        }
        for (const string& v : T) {
            if (distances.count(v) && distances.at(v) != numeric_limits<int>::max()) {
                potential[v] += distances.at(v);
            }
        }
    }

    pair<int, vector<tuple<string, string, int>>> calculate_final_matching() {
        // Calcular peso total do emparelhamento (MÁXIMO)
        int total_weight = 0;
        vector<tuple<string, string, int>> matching_edges;
        
        for (const auto& [u, v] : matching) {
            // Encontrar peso original da aresta
            if (graph.count(u)) {
                for (const Edge& edge : graph[u]) {
                    if (edge.to == v) {
                        total_weight += edge.weight;  // Peso original (positivo)
                        matching_edges.emplace_back(u, v, edge.weight);
                        break;
                    }
                }
            }
        }
        
        return {total_weight, matching_edges};
    }
    
    pair<int, vector<tuple<string, string, int>>> solve() {
        algorithm_start = std::chrono::high_resolution_clock::now();
        initialize_potentials();
        
        metrics = AlgorithmMetrics();
        int iteration = 0;
        int max_path_length = 0;
        
        while (!S0.empty()) {
            auto iteration_start = std::chrono::high_resolution_clock::now();
            
            // Coletar métricas da iteração
            IterationMetrics im = collect_iteration_metrics(iteration);
            
            // Executar Dijkstra e coletar métricas detalhadas
            auto dijkstra_start = std::chrono::high_resolution_clock::now();
            auto [distances, predecessors, dijkstra_iters, queue_ops, potential_upd] = dijkstra_from_s0();
            
            // Encontrar caminho aumentante
            auto [target, path] = find_augmenting_path(distances, predecessors);
            
            auto iteration_end = std::chrono::high_resolution_clock::now();
            im.iteration_time_ms = std::chrono::duration<double, std::milli>(
                iteration_end - iteration_start).count();
            
            // Atualizar métricas do caminho
            PathMetrics pm = collect_path_metrics(path, dijkstra_iters, queue_ops, potential_upd);
            pm.path_search_time_ms = std::chrono::duration<double, std::milli>(
                iteration_end - dijkstra_start).count();
            
            max_path_length = max(max_path_length, pm.path_length);
            
            metrics.iteration_metrics.push_back(im);
            metrics.path_metrics.push_back(pm);
            
            if (path.empty()) break;
            
            augment_matching(path);
            update_potentials(distances);
            iteration++;
        }
        
        auto algorithm_end = std::chrono::high_resolution_clock::now();
        
        double total_iteration_time = 0.0;
        for (const auto& im : metrics.iteration_metrics) {
            total_iteration_time += im.iteration_time_ms;
        }

        double total_time = std::chrono::duration<double, std::milli>(
            algorithm_end - algorithm_start).count();

        // Calcular métricas de escalabilidade
        metrics.scalability.graph_size = S.size() + T.size();
        metrics.scalability.edge_count = edges.size();
        metrics.scalability.total_time_ms = total_time;
        metrics.scalability.avg_iteration_time_ms = iteration > 0 ? total_iteration_time / iteration : 0;
        metrics.scalability.time_per_vertex = total_time / (S.size() + T.size());
        metrics.scalability.time_per_edge = total_time / edges.size();
        metrics.scalability.max_path_length = max_path_length;
        metrics.scalability.convergence_rate = total_time > 0 ? 
    (double)matching.size() / total_time : 0;
        
        metrics.total_iterations = iteration;
        
        return calculate_final_matching();
    }
    
    void save_metrics_csv(int save_mode) const {
        // Criar diretório results se não existir
        if (system("mkdir -p results") != 0) {
            throw std::runtime_error("Failed to create results directory");
        }

        // Arquivo principal para métricas gerais
        if (save_mode == 1 || save_mode == 4) {
            ofstream main_metrics("results/main_metrics.csv", ios::trunc);
            main_metrics << "graph_size,edge_count,total_iterations,total_time_ms,avg_iteration_time_ms,"
                        << "time_per_vertex,time_per_edge,max_path_length,convergence_rate\n";
            main_metrics << metrics.scalability.graph_size << ","
                        << metrics.scalability.edge_count << ","
                        << metrics.total_iterations << ","
                        << metrics.scalability.total_time_ms << ","
                        << metrics.scalability.avg_iteration_time_ms << ","
                        << metrics.scalability.time_per_vertex << ","
                        << metrics.scalability.time_per_edge << ","
                        << metrics.scalability.max_path_length << ","
                        << metrics.scalability.convergence_rate << "\n";
        }

        // Arquivo para métricas das iterações
        if (save_mode == 2 || save_mode == 4) {
            ofstream iteration_metrics("results/iteration_metrics.csv", ios::trunc);
            iteration_metrics << "iteration,time_ms,matching_size,avg_potential_s,avg_potential_t,equality_edges\n";
            for (const auto& im : metrics.iteration_metrics) {
                iteration_metrics << im.iteration_number << ","
                                << im.iteration_time_ms << ","
                                << im.matching_size << ","
                                << im.avg_potential_s << ","
                                << im.avg_potential_t << ","
                                << im.equality_edges << "\n";
            }
        }

        // Arquivo para métricas dos caminhos
        if (save_mode == 3 || save_mode == 4) {
            ofstream path_metrics("results/path_metrics.csv", ios::trunc);
            path_metrics << "iteration,search_time_ms,"
                        << "path_length,dijkstra_iterations,queue_operations,potential_updates,min_slack,tight_edges_count\n";
            for (size_t i = 0; i < metrics.path_metrics.size(); i++) {
                const auto& pm = metrics.path_metrics[i];
                path_metrics << i << ","
                            << pm.path_search_time_ms << ","
                            << pm.path_length << ","
                            << pm.dijkstra_iterations << ","
                            << pm.queue_operations << ","
                            << pm.potential_updates << ","
                            << pm.min_slack << ","
                            << pm.tight_edges_count << "\n";
            }
        }
    }
};

int main(int argc, char* argv[]) {
    // Default save mode is 0 (no saves)
    int save_mode = 0;
    
    // Parse command line argument if provided
    if (argc > 1) {
        save_mode = std::stoi(argv[1]);
    }

    auto [graph, optimal_value] = read_input();
    
    HungarianAlgorithm hungarian(graph);
    //hungarian.print_graph();
    
    auto [total_weight, matching_edges] = hungarian.solve();
    int result = -total_weight; // Negando para obter o valor real
    
    cout << result << endl;

    // Imprimir arestas do emparelhamento final
    //print_pairing_edges(matching_edges);
    
    // Salvar métricas detalhadas
    if (save_mode >= 1 && save_mode <= 4) {
        hungarian.save_metrics_csv(save_mode);
    }

    // Verificar se o resultado está correto
    if (optimal_value.has_value()) {
        if (result != optimal_value.value()) {
            throw std::runtime_error("Resultado incorreto! Valor encontrado (" + 
                                   std::to_string(result) + 
                                   ") diferente do valor ótimo (" + 
                                   std::to_string(optimal_value.value()) + ")");
        }
        //cout << "Resultado correto! O valor encontrado corresponde ao valor ótimo.\n";
    }
    
    return 0;
}