#!/usr/bin/env python3
"""
Script para análise dos resultados dos testes do Algoritmo Húngaro
Gera gráficos e estatísticas para validar a complexidade O(n(m+n log n))
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from scipy.optimize import curve_fit
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import os
import warnings
warnings.filterwarnings('ignore')

# Configuração de estilo para gráficos
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

class HungarianAnalysis:
    def __init__(self):
        self.results_dir = 'results'
        self.figures_dir = os.path.join(self.results_dir, 'figures')
        os.makedirs(self.figures_dir, exist_ok=True)
        
    def load_data(self, test_type):
        """Carregar dados de um tipo específico de teste"""
        try:
            file_path = os.path.join(self.results_dir, test_type, 'aggregated_data.csv')
            return pd.read_csv(file_path)
        except FileNotFoundError:
            print(f"❌ Arquivo não encontrado: {file_path}")
            return None
    
    def analyze_convergence(self):
        """Análise do teste de convergência (laço principal)"""
        print(" Analisando Convergência...")
        
        df = self.load_data('convergence')
        if df is None:
            return
            
        # Calcular estatísticas por tamanho
        stats_by_size = df.groupby('size').agg({
            'total_iterations': ['mean', 'std', 'min', 'max'],
            'total_time_ms': ['mean', 'std'],
            'avg_iteration_time_ms': ['mean', 'std'],
            'convergence_rate_ms': ['mean', 'std']
        }).round(3)
        
        print(stats_by_size)
        
        # Gráfico 1: Iterações vs Tamanho
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 2, 1)
        sizes = df.groupby('size')['total_iterations'].mean()
        plt.plot(sizes.index, sizes.values, 'bo-', linewidth=2, markersize=8)
        plt.xlabel('Tamanho do Grafo (n)')
        plt.ylabel('Número de Iterações')
        plt.title('Convergência: Iterações vs Tamanho')
        plt.grid(True, alpha=0.3)
        
        # Regressão linear para verificar O(n)
        X = sizes.index.values.reshape(-1, 1)
        y = sizes.values
        reg = LinearRegression().fit(X, y)
        r2 = r2_score(y, reg.predict(X))
        plt.plot(sizes.index, reg.predict(X), 'r--', alpha=0.7, 
                label=f'Linear fit (R²={r2:.3f})')
        plt.legend()
        
        # Gráfico 2: Tempo total vs Tamanho
        plt.subplot(2, 2, 2)
        times = df.groupby('size')['total_time_ms'].mean()
        plt.plot(times.index, times.values, 'go-', linewidth=2, markersize=8)
        plt.xlabel('Tamanho do Grafo (n)')
        plt.ylabel('Tempo Total (ms)')
        plt.title('Tempo Total vs Tamanho')
        plt.grid(True, alpha=0.3)
        
        # Gráfico 3: Box plot das iterações
        plt.subplot(2, 2, 3)
        df.boxplot(column='total_iterations', by='size', ax=plt.gca())
        plt.title('Distribuição das Iterações por Tamanho')
        plt.suptitle('')
        
        # Gráfico 4: Taxa de convergência
        plt.subplot(2, 2, 4)
        conv_rates = df.groupby('size')['convergence_rate_ms'].mean()
        plt.plot(conv_rates.index, conv_rates.values, 'mo-', linewidth=2, markersize=8)
        plt.xlabel('Tamanho do Grafo (n)')
        plt.ylabel('Taxa de Convergência')
        plt.title('Taxa de Convergência vs Tamanho')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.figures_dir, 'convergence_analysis.png'), dpi=300)
        plt.show()
        
    def analyze_complexity(self):
        """Análise detalhada da complexidade computacional"""
        print("\n Analisando Complexidade Computacional...")
        
        df = self.load_data('scalability')
        if df is None:
            return
        
        # Prepare data
        grouped = df.groupby(['size', 'density']).agg({
            'total_time_ms': 'mean',
            'time_per_vertex': 'mean',
            'graph_size': 'first',
            'max_path_length': 'mean',
            'total_iterations': 'mean',
            'avg_tree_depth': 'mean',
            'potential_variance': 'mean',
            'tight_edges_ratio': 'mean'
        }).reset_index()
        
        # Generate individual plots
        self._plot_time_vs_theoretical(grouped)
        self._plot_queue_operations_ratio(grouped)
        self._plot_path_length_distribution(grouped)
        
        print(" ✅ Gráficos salvos em subdiretórios de", self.figures_dir)
        
    def _fit_complexity_models(self, data):
        """Ajustar diferentes modelos de complexidade"""
        print("\n Ajustando Modelos de Complexidade...")
        
        X = data[['n', 'm', 'theoretical_complexity']]
        y = data['total_time_ms']
        
        # Modelo 1: Linear simples
        reg_simple = LinearRegression().fit(X[['theoretical_complexity']], y)
        r2_simple = r2_score(y, reg_simple.predict(X[['theoretical_complexity']]))
        
        # Modelo 2: Modelo teórico completo
        def theoretical_model(params, n, m):
            a, b = params
            return a * n * (m + n * np.log(n)) + b
        
        # Modelo 3: Modelo empírico generalizado
        def empirical_model(params, n, m):
            a, alpha, beta, gamma, b = params
            return a * (n ** alpha) * (m ** beta) * (np.log(n) ** gamma) + b
        
        try:
            # Ajuste do modelo teórico
            def fit_theoretical(x, a, b):
                n, m = x
                return a * n * (m + n * np.log(n)) + b
            
            popt_theo, _ = curve_fit(
                lambda x, a, b: fit_theoretical((x[:, 0], x[:, 1]), a, b),
                data[['n', 'm']].values, y,
                p0=[1e-6, 0], maxfev=10000
            )
            
            y_pred_theo = fit_theoretical((data['n'], data['m']), *popt_theo)
            r2_theo = r2_score(y, y_pred_theo)
            
            print(f"Modelo Linear Simples - R²: {r2_simple:.4f}")
            print(f"Modelo Teórico O(n(m+n log n)) - R²: {r2_theo:.4f}")
            print(f"Parâmetros teóricos: a={popt_theo[0]:.2e}, b={popt_theo[1]:.2f}")
            
        except Exception as e:
            print(f"Erro no ajuste dos modelos: {e}")
    
    def _plot_time_vs_theoretical(self, data):
        """Plot execution time vs theoretical complexity for different densities"""
        plt.figure(figsize=(10, 6))
        
        # Plot real data for each density
        for density in sorted(data['density'].unique()):
            subset = data[data['density'] == density]
            plt.plot(subset['size'], subset['total_time_ms'], 'o-', 
                    label=f'Densidade {density}%', linewidth=2, markersize=6)
        
        # Add theoretical complexity line
        n_range = np.linspace(data['size'].min(), data['size'].max(), 100)
        theoretical_times = []
        
        # Calculate theoretical times for average density
        avg_density = data['density'].mean() / 100
        for n in n_range:
            m = n * n * avg_density
            complexity = n * (m + n * np.log(n))
            theoretical_times.append(complexity)
        
        theoretical_times = np.array(theoretical_times)
        scale_factor = data['total_time_ms'].max() / max(theoretical_times)
        theoretical_times = theoretical_times * scale_factor
        
        plt.plot(n_range, theoretical_times, 'k--', 
                label='Complexidade Teórica', linewidth=2, alpha=0.7)
        
        plt.xlabel('Tamanho do Grafo (n)')
        plt.ylabel('Tempo de Execução (ms)')
        plt.title('Tempo de Execução vs Complexidade Teórica')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        os.makedirs(os.path.join(self.figures_dir, 'performance'), exist_ok=True)
        plt.savefig(os.path.join(self.figures_dir, 'performance', 'time_vs_theoretical.png'), 
                    dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_queue_operations_ratio(self, data):
        """Plot queue efficiency metrics by density"""
        plt.figure(figsize=(10, 6))
        
        sns.boxplot(x='density', y='time_per_vertex', data=data)
        
        plt.xlabel('Densidade do Grafo (%)')
        plt.ylabel('Tempo por Vértice (ms)')
        plt.title('Eficiência de Processamento por Densidade')
        plt.grid(True, alpha=0.3)
        
        plt.savefig(os.path.join(self.figures_dir,'processing_efficiency.png'), 
                    dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_path_length_distribution(self, data):
        """Plot path length distribution by density"""
        plt.figure(figsize=(10, 6))
        
        # Create boxplot of path lengths
        sns.boxplot(x='density', y='max_path_length', data=data)
        
        plt.xlabel('Densidade do Grafo (%)')
        plt.ylabel('Comprimento do Caminho')
        plt.title('Distribuição do Comprimento dos Caminhos por Densidade')
        plt.grid(True, alpha=0.3)
        
        plt.savefig(os.path.join(self.figures_dir, 'path_length_distribution.png'), 
                    dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_complexity_analysis(self, data):
        """Gerar gráficos detalhados da análise de complexidade"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # Gráfico 1: Iterações vs Tamanho por Densidade
        ax2 = axes[0, 1]
        for density in sorted(data['density'].unique()):
            subset = data[data['density'] == density]
            ax2.plot(subset['n'], subset['total_iterations'], 'o-', 
                    label=f'Densidade {density:.1f}', linewidth=2, markersize=6)
        ax2.set_xlabel('Tamanho do Grafo (n)')
        ax2.set_ylabel('Número de Iterações')
        ax2.set_title('Iterações vs Tamanho por Densidade')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Gráfico 3: Fração de Arestas Tight
        ax3 = axes[0, 2]
        for density in sorted(data['density'].unique()):
            subset = data[data['density'] == density]
            ax3.plot(subset['n'], subset['tight_edges_ratio'], 's-', 
                    label=f'Densidade {density:.1f}', linewidth=2, markersize=6)
        ax3.set_xlabel('Tamanho do Grafo (n)')
        ax3.set_ylabel('Fração de Arestas Tight')
        ax3.set_title('Evolução das Arestas Tight')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        
        # Gráfico 4: Profundidade Média da Árvore
        ax5 = axes[1, 1]
        for density in sorted(data['density'].unique()):
            subset = data[data['density'] == density]
            ax5.plot(subset['n'], subset['avg_tree_depth'], 'd-', 
                    label=f'Densidade {density:.1f}', linewidth=2, markersize=6)
        ax5.set_xlabel('Tamanho do Grafo (n)')
        ax5.set_ylabel('Profundidade Média da Árvore')
        ax5.set_title('Estrutura dos Caminhos de Busca')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Gráfico 5: Heatmap de Performance
        ax6 = axes[1, 2]
        pivot_data = data.pivot(index='n', columns='density', values='total_time_ms')
        sns.heatmap(pivot_data, annot=True, fmt='.1f', cmap='YlOrRd', ax=ax6)
        ax6.set_title('Heatmap: Tempo por Tamanho e Densidade')
        ax6.set_xlabel('Densidade')
        ax6.set_ylabel('Tamanho do Grafo (n)')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.figures_dir, 'complexity_analysis.png'), dpi=300)
        plt.show()
        
    def run_complete_analysis(self):
        """Executar análise completa"""
        print(" Iniciando Análise Completa do Algoritmo Húngaro\n")
        print("=" * 60)
        
        # Executar todas as análises
        self.analyze_convergence()
        self.analyze_complexity()
        
        print("\n" + "=" * 60)
        print("✅ Análise Completa Finalizada!")
        print(f" Figuras salvas em: {self.figures_dir}")
        print(f" Relatório disponível em: {os.path.join(self.results_dir, 'analysis_report.md')}")

if __name__ == "__main__":
    # Criar instância do analisador
    analyzer = HungarianAnalysis()
    
    # Executar análise completa
    analyzer.run_complete_analysis()