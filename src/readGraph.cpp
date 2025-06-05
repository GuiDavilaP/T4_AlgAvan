#include "../inc/readGraph.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

GraphData read_input() {
    GraphData data;
    string line;
    
    // Ler primeira linha com o tamanho n
    if (getline(cin, line)) {
        int n = stoi(line);
        
        // Ler matriz de custos (n linhas com n valores cada)
        vector<vector<int>> cost_matrix(n, vector<int>(n));
        
        for (int i = 0; i < n; i++) {
            if (getline(cin, line)) {
                istringstream iss(line);
                for (int j = 0; j < n; j++) {
                    if (!(iss >> cost_matrix[i][j])) {
                        cerr << "ERRO: Não conseguiu ler elemento [" << i << "][" << j << "]" << endl;
                        return data;
                    }
                }
            }
        }
        
        // Converter matriz para grafo bipartido
        // Vértices S: 0, 1, 2, ..., n-1 (linhas)
        // Vértices T: n, n+1, n+2, ..., 2n-1 (colunas + offset n)
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                int from = i;           // Vértice em S
                int to = j + n;         // Vértice em T (com offset)
                int weight = cost_matrix[i][j];
                
                data.graph[from].emplace_back(to, weight);
            }
        }
        
        // Ler valor ótimo (última linha)
        if (getline(cin, line)) {
            data.optimal_value = stoi(line);
        }
    }
    
    return data;
}