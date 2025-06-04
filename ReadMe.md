# Parâmetros
- S: conjunto de vértices do lado esquerdo do grafo bipartido
- T: conjunto de vértices do lado direito do grafo bipartido
- S₀: conjunto de vértices livres (não emparelhados) em S
- T₀: conjunto de vértices livres em T
- p_v: potencial associado a cada vértice v
- W: peso máximo entre todas as arestas

# Implementação do Método Húngaro (Versão Schrijver)
O método húngaro descrito no livro de Schrijver resolve este problema através de uma abordagem que:

- Busca caminhos aumentantes para incrementalmente melhorar o emparelhamento
- Usa a transformação de Johnson para manter pesos não-negativos
- Aplica Dijkstra para encontrar os caminhos aumentantes mais curtos

## Estruturas de Dados Utilizadas
### 1. Representação do Grafo
cppmap<string, vector<Edge>> graph;

- Lista de adjacência usando map para mapear vértices (strings) para suas arestas
- Estrutura Edge contém destino e peso
- Eficiência: O(log V) para acesso, mas adequado para grafos com vértices nomeados

### 2. Conjuntos de Vértices
cppset<string> S, T;        // Partições do grafo bipartido
set<string> S0, T0;      // Vértices livres (não emparelhados)

- Sets para operações de pertencimento eficientes O(log V)
- S: vértices de origem, T: vértices de destino

### 3. Emparelhamento
cppmap<string, string> matching;           // matching[u] = v
map<string, string> reverse_matching;   // reverse_matching[v] = u

- Mapeamento bidirecional para navegação eficiente em ambas as direções
- Permite O(log V) para verificar se um vértice está emparelhado

### 4. Potenciais
cppmap<string, int> potential;

Armazena os potenciais dos vértices para a transformação de Johnson

## Decisões de Implementação Importantes
### 1. Transformação para Maximização
cppint get_transformed_weight(const string& u, const string& v, int original_weight) {
    return -original_weight - (potential[v] - potential[u]);
}

- Negação dos pesos: Para resolver maximização usando algoritmos de caminho mínimo
- Transformação de Johnson: Mantém pesos não-negativos usando potenciais

### 2. Inicialização de Potenciais
cpp// S: p_v := 0
// T: p_v := W (peso máximo)

- Estratégia específica para o problema de maximização
- Garante que a transformação inicial produza pesos não-negativos

### 3. Algoritmo de Dijkstra Modificado

- Múltiplas fontes: Inicia de todos os vértices livres em S
- Grafo expandido: Inclui arestas do matching com peso 0 (direção T→S)
- Filtragem de arestas: Evita arestas já no matching na direção S→T

## Métricas de teste
### 1. Métricas de Caminho (path_metrics.csv)
- path_length: Comprimento do caminho aumentante (varia!)
- dijkstra_iterations: Quantos nós foram processados (eficiência)
- queue_operations: Push/pop na priority queue (complexidade)
- potential_updates: Quantas atualizações de distância (convergência)
- min_slack: Menor slack encontrado (proximidade da otimalidade)
- tight_edges_count: Arestas no subgrafo de igualdade (estrutura)

### 2. Métricas de Iteração (iteration_metrics.csv)
- matching_size: Crescimento do matching (0→10)
- avg_potential_s/t: Evolução dos potenciais (convergência)
- equality_edges: Tamanho do subgrafo de igualdade (estrutura)

### 3. Métricas Escalabilidade (main_metrics.csv)
- time_per_vertex: Tempo por vértice (escalabilidade)
- time_per_edge: Tempo por aresta (eficiência)
- max_path_length: Maior caminho encontrado (complexidade)
- convergence_rate: Taxa de convergência (performance)