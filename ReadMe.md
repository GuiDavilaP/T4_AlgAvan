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
cppmap<int, vector<Edge>> graph;

- Lista de adjacência usando map para mapear vértices (int de ID) para suas arestas
- Estrutura Edge contém destino e peso
- Eficiência: O(log V) para acesso, mas adequado para grafos com vértices nomeados

### 2. Conjuntos de Vértices
cppset<int> S, T;        // Partições do grafo bipartido
set<int> S0, T0;      // Vértices livres (não emparelhados)

- Sets para operações de pertencimento eficientes O(log V)
- S: vértices de origem, T: vértices de destino

### 3. Emparelhamento
cppmap<int, string> matching;           // matching[u] = v
map<int, string> reverse_matching;   // reverse_matching[v] = u

- Mapeamento bidirecional para navegação eficiente em ambas as direções
- Permite O(log V) para verificar se um vértice está emparelhado

### 4. Potenciais
cppmap<int, int> potential;

Armazena os potenciais dos vértices para a transformação de Johnson