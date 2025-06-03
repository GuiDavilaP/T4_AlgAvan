#include "../inc/readGraph.hpp"
#include <iostream>

GraphData read_input() {
    int n;
    std::cin >> n;
    
    // Matriz de pesos
    std::vector<std::vector<int>> weights(n, std::vector<int>(n));
    
    // Ler matriz de pesos
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            std::cin >> weights[i][j];
        }
    }
    
    // Tentar ler o valor ótimo
    std::optional<int> optimal_value;
    int value;
    if(std::cin >> value) {
        optimal_value = value;
    }
    
    // Converter para o formato do algoritmo húngaro
    std::map<std::string, std::vector<std::pair<std::string, int>>> graph;
    
    for(int i = 0; i < n; i++) {
        std::string source = "s" + std::to_string(i+1);
        std::vector<std::pair<std::string, int>> edges;
        
        for(int j = 0; j < n; j++) {
            std::string target = "t" + std::to_string(j+1);
            // Para maximização, negamos os pesos
            edges.emplace_back(target, -weights[i][j]);
        }
        
        graph[source] = edges;
    }
    
    return {graph, optimal_value};
}