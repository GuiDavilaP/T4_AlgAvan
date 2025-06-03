#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <limits>
#include <algorithm>
#include "../inc/readGraph.hpp"

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
    
    pair<map<string, int>, map<string, string>> dijkstra_from_s0() {
        map<string, int> distances;
        map<string, string> predecessors;
        set<string> visited;
        
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
        }
        
        while (!pq.empty()) {
            auto [current_dist, u] = pq.top();
            pq.pop();
            
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
                }
            }
        }
        
        return {distances, predecessors};
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
    
    pair<int, vector<tuple<string, string, int>>> solve() {
        initialize_potentials();
        
        int iteration = 0;
        while (!S0.empty()) {
            // Executar Dijkstra
            auto [distances, predecessors] = dijkstra_from_s0();
            
            // Encontrar caminho aumentante
            auto [target, path] = find_augmenting_path(distances, predecessors);
            
            // Imprimir status da iteração
            //print_iteration_status(iteration, path);
            
            if (path.empty()) {
                break;
            }
            
            // Aumentar emparelhamento
            augment_matching(path);
            
            // Atualizar potenciais
            update_potentials(distances);
            
            iteration++;
        }
        
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
    
    void print_graph() {
        cout << "Grafo de entrada:\n";
        for (const auto& [u, adj_list] : graph) {
            cout << "  " << u << ": ";
            for (const Edge& edge : adj_list) {
                cout << "(" << edge.to << "," << edge.weight << ") ";
            }
            cout << "\n";
        }
    }
};

// Modificar apenas a função main:
int main() {
    auto [graph, optimal_value] = read_input();
    
    HungarianAlgorithm hungarian(graph);
    //hungarian.print_graph();
    
    auto [total_weight, matching_edges] = hungarian.solve();
    int result = -total_weight; // Negando para obter o valor real
    
    cout << result << "\n";

    // Imprimir arestas do emparelhamento final
    //print_pairing_edges(matching_edges);
    
    // Verificar se o resultado está correto
    if (optimal_value.has_value()) {
        if (result != optimal_value.value()) {
            throw std::runtime_error("Resultado incorreto! Valor encontrado (" + 
                                   std::to_string(result) + 
                                   ") diferente do valor ótimo (" + 
                                   std::to_string(optimal_value.value()) + ")");
        }
        cout << "Resultado correto! O valor encontrado corresponde ao valor ótimo.\n";
    }
    
    return 0;
}