#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <string>
#include <filesystem>
#include <cmath>

enum class GraphType {
    UNIFORM_RANDOM,    // Distribuição uniforme de pesos
    WORST_CASE,        // Grafos que forçam muitas iterações  
    BEST_CASE,         // Grafos que convergem rapidamente
    CLUSTERED,         // Pesos agrupados em faixas
    SPARSE_CRITICAL    // Densidade crítica para análise
};

class GraphGenerator {
private:
    std::mt19937 rng;
    
    // Gerar peso baseado no tipo de grafo
    int generate_weight(GraphType type, int min_weight = 1, int max_weight = 100) {
        switch (type) {
            case GraphType::UNIFORM_RANDOM:
                return std::uniform_int_distribution<int>(min_weight, max_weight)(rng);
                
            case GraphType::WORST_CASE:
                // Pesos que tendem a criar muitos empates e forçar mais iterações
                return std::uniform_int_distribution<int>(1, 10)(rng);
                
            case GraphType::BEST_CASE:
                // Pesos bem distintos que facilitam convergência rápida
                return std::uniform_int_distribution<int>(1, max_weight)(rng) * 10;
                
            case GraphType::CLUSTERED:
                // Pesos em 3 clusters distintos
                {
                    int cluster = std::uniform_int_distribution<int>(0, 2)(rng);
                    int base = (cluster + 1) * (max_weight / 3);
                    return base + std::uniform_int_distribution<int>(-5, 5)(rng);
                }
                
            case GraphType::SPARSE_CRITICAL:
            default:
                return std::uniform_int_distribution<int>(min_weight, max_weight)(rng);
        }
    }
    
public:
    GraphGenerator(unsigned seed = std::random_device{}()) : rng(seed) {}
    
    // Gerar grafo bipartido completo (matriz n x n)
    void generate_complete_bipartite(int n, GraphType type, const std::string& filename, 
                                   int optimal_value = -1) {
        std::vector<std::vector<int>> matrix(n, std::vector<int>(n));
        
        // Preencher matriz com pesos
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                matrix[i][j] = generate_weight(type);
            }
        }
        
        // Salvar arquivo
        save_graph(matrix, filename, optimal_value);
    }
    
    // Gerar grafo bipartido com densidade específica
    void generate_sparse_bipartite(int n, double density, GraphType type, 
                                 const std::string& filename, int optimal_value = -1) {
        std::vector<std::vector<int>> matrix(n, std::vector<int>(n, 0));
        
        // Garantir que cada linha e coluna tenha pelo menos uma aresta não-zero
        std::vector<bool> row_has_edge(n, false);
        std::vector<bool> col_has_edge(n, false);
        
        // Primeiro, garantir conectividade mínima
        for (int i = 0; i < n; i++) {
            // Garantir que linha i tenha pelo menos uma aresta
            if (!row_has_edge[i]) {
                int j = std::uniform_int_distribution<int>(0, n-1)(rng);
                matrix[i][j] = generate_weight(type);
                row_has_edge[i] = true;
                col_has_edge[j] = true;
            }
        }
        
        // Garantir que todas as colunas tenham arestas
        for (int j = 0; j < n; j++) {
            if (!col_has_edge[j]) {
                int i = std::uniform_int_distribution<int>(0, n-1)(rng);
                matrix[i][j] = generate_weight(type);
                row_has_edge[i] = true;
                col_has_edge[j] = true;
            }
        }
        
        // Adicionar arestas restantes baseado na densidade
        int total_edges = n * n;
        int target_edges = static_cast<int>(total_edges * density);
        int current_edges = 0;
        
        // Contar arestas já adicionadas
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (matrix[i][j] > 0) current_edges++;
            }
        }
        
        // Adicionar arestas restantes aleatoriamente
        while (current_edges < target_edges) {
            int i = std::uniform_int_distribution<int>(0, n-1)(rng);
            int j = std::uniform_int_distribution<int>(0, n-1)(rng);
            
            if (matrix[i][j] == 0) {
                matrix[i][j] = generate_weight(type);
                current_edges++;
            }
        }
        
        save_graph(matrix, filename, optimal_value);
    }
    
private:
    void save_graph(const std::vector<std::vector<int>>& matrix, const std::string& filename, 
                   int optimal_value = -1) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Não foi possível criar o arquivo: " + filename);
        }
        
        int n = matrix.size();
        file << n << std::endl;
        
        // Escrever matriz
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                file << matrix[i][j];
                if (j < n - 1) file << " ";
            }
            file << std::endl;
        }
        
        // Adicionar valor ótimo se fornecido
        if (optimal_value != -1) {
            file << optimal_value << std::endl;
        }
        
        file.close();
    }
};

// Função para criar diretórios se não existirem
void ensure_directory_exists(const std::string& path) {
    std::filesystem::create_directories(path);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: " << argv[0] << " <modo>\n";
        std::cout << "Modos disponíveis:\n";
        std::cout << "  1 - Teste de Convergência (laço principal)\n";
        std::cout << "  2 - Teste de Dijkstra (busca de caminho)\n";
        std::cout << "  3 - Teste de Escalabilidade\n";
        std::cout << "  4 - Todos os testes\n";
        return 1;
    }
    
    int mode = std::stoi(argv[1]);
    
    try {
        GraphGenerator generator;
        
        if (mode == 1 || mode == 4) {
            // Teste de Convergência - varia tamanho, densidade fixa
            std::cout << "Gerando grafos para teste de convergência...\n";
            ensure_directory_exists("data/convergence");
            
            std::vector<int> sizes = {10, 20, 50, 100, 200};
            double density = 0.7; // 70% de densidade
            int repetitions = 30;
            
            for (int size : sizes) {
                std::cout << "Tamanho " << size << ": ";
                for (int rep = 0; rep < repetitions; rep++) {
                    std::string filename = "data/convergence/n" + std::to_string(size) + 
                                         "_rep" + std::to_string(rep + 1) + ".dat";
                    generator.generate_sparse_bipartite(size, density, GraphType::UNIFORM_RANDOM, filename);
                    
                    if ((rep + 1) % 10 == 0) std::cout << (rep + 1) << " ";
                }
                std::cout << "✓\n";
            }
        }
        
        if (mode == 2 || mode == 4) {
            // Teste de Dijkstra - varia densidade, tamanho fixo
            std::cout << "Gerando grafos para teste de Dijkstra...\n";
            ensure_directory_exists("data/dijkstra");
            
            std::vector<double> densities = {0.3, 0.5, 0.7, 0.9, 1.0};
            int fixed_size = 100;
            int repetitions = 25;
            
            for (double density : densities) {
                std::cout << "Densidade " << (int)(density * 100) << "%: ";
                for (int rep = 0; rep < repetitions; rep++) {
                    std::string filename = "data/dijkstra/d" + std::to_string((int)(density * 100)) + 
                                         "_rep" + std::to_string(rep + 1) + ".dat";
                    
                    if (density == 1.0) {
                        generator.generate_complete_bipartite(fixed_size, GraphType::UNIFORM_RANDOM, filename);
                    } else {
                        generator.generate_sparse_bipartite(fixed_size, density, GraphType::UNIFORM_RANDOM, filename);
                    }
                    
                    if ((rep + 1) % 5 == 0) std::cout << (rep + 1) << " ";
                }
                std::cout << "✓\n";
            }
        }
        
        if (mode == 3 || mode == 4) {
            // Teste de Escalabilidade - combinações (n, densidade)
            std::cout << "Gerando grafos para teste de escalabilidade...\n";
            ensure_directory_exists("data/scalability");
            
            std::vector<int> sizes = {50, 100, 200, 400};
            std::vector<double> densities = {0.3, 0.6, 0.9};
            int repetitions = 20;
            
            for (int size : sizes) {
                for (double density : densities) {
                    std::cout << "n=" << size << ", d=" << (int)(density * 100) << "%: ";
                    
                    for (int rep = 0; rep < repetitions; rep++) {
                        std::string filename = "data/scalability/n" + std::to_string(size) + 
                                             "_d" + std::to_string((int)(density * 100)) +
                                             "_rep" + std::to_string(rep + 1) + ".dat";
                        
                        if (density == 1.0) {
                            generator.generate_complete_bipartite(size, GraphType::UNIFORM_RANDOM, filename);
                        } else {
                            generator.generate_sparse_bipartite(size, density, GraphType::UNIFORM_RANDOM, filename);
                        }
                        
                        if ((rep + 1) % 5 == 0) std::cout << (rep + 1) << " ";
                    }
                    std::cout << "✓\n";
                }
            }
        }
        
        std::cout << "\nTodos os grafos foram gerados com sucesso!\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Erro: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}