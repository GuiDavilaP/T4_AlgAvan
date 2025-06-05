#!/bin/bash

# Script de automação para testes do Algoritmo Húngaro
# Baseado no validationTest.sh fornecido

set -e  # Parar execução em caso de erro

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Enumeração dos modos de salvamento
SAVE_NONE=0      # Não salva nenhum arquivo
SAVE_MAIN=1      # Salva apenas main_metrics.csv
SAVE_ITERATION=2 # Salva apenas iteration_metrics.csv
SAVE_PATH=3      # Salva apenas path_metrics.csv
SAVE_ALL=4       # Salva todos os arquivos

# Função para imprimir com cores
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Função para executar um teste individual
run_single_test() {
    local input_file="$1"
    local save_mode="$2"
    local timeout_duration="$3"
    
    filename=$(basename "$input_file")
    
    # Executar com timeout e capturar saída e erro
    if ! timeout "$timeout_duration" ./bin/emparelhamento "$save_mode" < "$input_file" > temp_output.log 2>error.log; then
        echo "Erro na execução:"
        cat error.log
        rm -f temp_output.log
        return 1
    fi
    
    rm -f temp_output.log
    return 0
}

# Função para executar teste de convergência
run_convergence_test() {
    print_status "=== TESTE DE CONVERGÊNCIA (Laço Principal) ==="
    
    # Criar diretório de resultados
    mkdir -p results/convergence
    
    # Contadores GLOBAIS
    total=0
    passed=0
    declare -a failed_files=()
    
    # Arquivo para coletar dados agregados
    echo "size,repetition,total_size, edge_count,total_iterations,total_time_ms,avg_iteration_time_ms,time_per_vertex,time_per_edge,max_path_length,convergence_rate_ms,avg_tree_depth,potential_variance,tight_edges_ratio" > results/convergence/aggregated_data.csv
    
    # Testar cada tamanho
    for size in 10 20 50 100 ; do
        print_status "Testando tamanho n=$size"
        
        # Contadores por tamanho (também não locais)
        size_passed=0
        size_total=0
        
        for rep in $(seq 1 30); do
            input_file="data/convergence/n${size}_rep${rep}.dat"
            
            # Verificar arquivo com caminho completo
            if [ ! -f "$input_file" ]; then
                print_warning "Arquivo não encontrado: $input_file"
                continue
            fi
            
            # Incrementar contadores
            total=$((total + 1))
            size_total=$((size_total + 1))
            
            # Determinar timeout baseado no tamanho
            timeout_val="30s"
            if [ $size -gt 200 ]; then
                timeout_val="60s"
            elif [ $size -gt 100 ]; then
                timeout_val="45s"
            fi
            
            print_status "Executando teste $rep de 30 para n=$size (total: $total)"
            
            # Executar teste e capturar status
            if run_single_test "$input_file" "$SAVE_MAIN" "$timeout_val"; then
                print_success "Teste $rep passou"
                passed=$((passed + 1))
                size_passed=$((size_passed + 1))
                
                # Verificar e salva as métricas de "main_metrics.csv" em aggregated_data.csv
                if [ -f "results/main_metrics.csv" ]; then
                    tail -n 1 "results/main_metrics.csv" | \
                    sed "s/^/$size,$rep,/" >> "results/convergence/aggregated_data.csv"
                else
                    print_warning "Arquivo de métricas não encontrado após teste"
                fi
            else
                print_error "Teste $rep falhou"
                failed_files+=("$input_file")
            fi
        done
        
        if [ $size_total -gt 0 ]; then
            success_rate=$(echo "scale=1; $size_passed*100/$size_total" | bc -l)
            print_status "n=$size: $size_passed/$size_total testes passaram ($success_rate%)"
        else
            print_warning "n=$size: Nenhum teste executado"
        fi
    done
    
    # Relatório final
    if [ $total -gt 0 ]; then
        total_success_rate=$(echo "scale=1; $passed*100/$total" | bc -l)
        print_success "Teste de Convergência: $passed/$total passaram ($total_success_rate%)"
    else
        print_error "Nenhum teste foi executado"
    fi
    
    # Listar arquivos que falharam
    if [ ${#failed_files[@]} -gt 0 ]; then
        print_warning "Arquivos que falharam:"
        printf '%s\n' "${failed_files[@]}"
    fi
}

# Função para executar teste de Dijkstra
run_dijkstra_test() {
    print_status "=== TESTE DE DIJKSTRA (Busca de Caminho) ==="
    
    mkdir -p results/dijkstra
    
    # Contadores GLOBAIS (sem local)
    total=0
    passed=0
    declare -a failed_files=()
    
    echo "density,repetition,path_search_time_ms,dijkstra_iterations,queue_operations,potential_updates" > results/dijkstra/aggregated_data.csv
    
    for density in 30 50 70 90 100; do
        print_status "Testando densidade $density%"
        
        # Contadores por densidade
        density_passed=0
        density_total=0
        
        for rep in {1..25}; do
            input_file="data/dijkstra/d${density}_rep${rep}.dat"
            
            if [ -f "$input_file" ]; then
                total=$((total + 1))
                density_total=$((density_total + 1))
                
                print_status "Executando teste $rep de 25 para densidade $density% (total: $total)"
                
                if run_single_test "$input_file" $SAVE_PATH "45s"; then
                    passed=$((passed + 1))
                    density_passed=$((density_passed + 1))
                    
                    # Coletar dados das métricas de caminho
                    if [ -f "results/path_metrics.csv" ]; then
                        # Calcular médias das métricas de caminho
                        awk -F',' -v density="$density" -v rep="$rep" '
                        NR > 1 {
                            search_time += $2
                            dijkstra_iter += $4
                            queue_ops += $5
                            potential_upd += $6
                            count++
                        }
                        END {
                            if (count > 0) {
                                printf "%d,%d,%.2f,%.2f,%.2f,%.2f\n", 
                                    density, rep, 
                                    search_time/count, 
                                    dijkstra_iter/count, 
                                    queue_ops/count, 
                                    potential_upd/count
                            }
                        }' results/path_metrics.csv >> results/dijkstra/aggregated_data.csv
                    else
                        print_warning "Arquivo path_metrics.csv não encontrado após teste"
                    fi
                else
                    print_error "Teste $rep falhou"
                    failed_files+=("$input_file")
                fi
                
                if [ $((rep % 5)) -eq 0 ]; then
                    echo -n "[$rep/25] "
                fi
            else
                print_warning "Arquivo não encontrado: $input_file"
            fi
        done
        
        echo # Nova linha após progresso
        if [ $density_total -gt 0 ]; then
            density_success_rate=$(echo "scale=1; $density_passed*100/$density_total" | bc -l)
            print_status "densidade $density%: $density_passed/$density_total testes passaram ($density_success_rate%)"
        else
            print_warning "densidade $density%: Nenhum teste executado"
        fi
    done
    
    # Relatório final
    if [ $total -gt 0 ]; then
        total_success_rate=$(echo "scale=1; $passed*100/$total" | bc -l)
        print_success "Teste de Dijkstra: $passed/$total passaram ($total_success_rate%)"
    else
        print_error "Nenhum teste foi executado"
    fi
    
    if [ ${#failed_files[@]} -gt 0 ]; then
        print_warning "Arquivos que falharam:"
        printf '%s\n' "${failed_files[@]}"
    fi
}

# Função para executar teste de escalabilidade
run_scalability_test() {
    print_status "=== TESTE DE ESCALABILIDADE ==="
    
    mkdir -p results/scalability
    
    # Contadores GLOBAIS
    total=0
    passed=0
    declare -a failed_files=()
    
    echo "size,density,repetition,graph_size,edge_count,total_iterations,total_time_ms,avg_iteration_time_ms,time_per_vertex,time_per_edge,max_path_length,convergence_rate,avg_tree_depth,potential_variance,tight_edges_ratio,theoretical_complexity" > results/scalability/aggregated_data.csv
    
    for size in 50 100 200; do
        for density in 30 60 90; do
            print_status "Testando n=$size, densidade=$density%"
            
            # Contadores por combinação
            combo_passed=0
            combo_total=0
            
            for rep in {1..20}; do
                input_file="data/scalability/n${size}_d${density}_rep${rep}.dat"
                
                if [ -f "$input_file" ]; then
                    total=$((total + 1))
                    combo_total=$((combo_total + 1))
                    
                    # Timeout maior para grafos grandes
                    timeout_val="60s"
                    if [ $size -gt 200 ]; then
                        timeout_val="120s"
                    fi
                    
                    print_status "Executando teste $rep de 20 para n=$size, d=$density% (total: $total)"
                    
                    if run_single_test "$input_file" $SAVE_MAIN "$timeout_val"; then
                        passed=$((passed + 1))
                        combo_passed=$((combo_passed + 1))
                        
                        if [ -f "results/main_metrics.csv" ]; then
                            # Calcula complexidade teórica: n * (m + n * log(n))
                            # m = n * n * density/100 (número aproximado de arestas)
                            m=$(echo "$size * $size * $density / 100" | bc -l)
                            theoretical=$(echo "$size * ($m + $size * l($size) / l(2))" | bc -l)
                            
                            # Captura metricas e adiciona tamanho, densidade, repetição e complexidade teórica
                            tail -n 1 "results/main_metrics.csv" | \
                            sed "s/^/$size,$density,$rep,/" | \
                            sed "s/$/,$theoretical/" >> results/scalability/aggregated_data.csv
                        else
                            print_warning "Arquivo main_metrics.csv não encontrado após teste"
                        fi
                    else
                        print_error "Teste $rep falhou"
                        failed_files+=("$input_file")
                    fi
                    
                    if [ $((rep % 5)) -eq 0 ]; then
                        echo -n "[$rep/20] "
                    fi
                    
                    # Debug: mostrar contadores atuais
                    echo "DEBUG: total=$total, passed=$passed, combo_passed=$combo_passed"
                else
                    print_warning "Arquivo não encontrado: $input_file"
                fi
            done
            
            echo # Nova linha após progresso
            if [ $combo_total -gt 0 ]; then
                combo_success_rate=$(echo "scale=1; $combo_passed*100/$combo_total" | bc -l)
                print_status "n=$size, d=$density%: $combo_passed/$combo_total passaram ($combo_success_rate%)"
            else
                print_warning "n=$size, d=$density%: Nenhum teste executado"
            fi
        done
    done
    
    # Relatório final
    if [ $total -gt 0 ]; then
        total_success_rate=$(echo "scale=1; $passed*100/$total" | bc -l)
        print_success "Teste de Escalabilidade: $passed/$total passaram ($total_success_rate%)"
    else
        print_error "Nenhum teste foi executado"
    fi
    
    if [ ${#failed_files[@]} -gt 0 ]; then
        print_warning "Arquivos que falharam:"
        printf '%s\n' "${failed_files[@]}"
    fi
    
    # Debug final
    echo "DEBUG FINAL: total=$total, passed=$passed, failed=${#failed_files[@]}"
}

# Função principal
main() {
    if [ $# -lt 1 ]; then
        echo "Uso: $0 <modo> [opções]"
        echo "Modos disponíveis:"
        echo "  generate <tipo>  - Gerar grafos (1=convergência, 2=dijkstra, 3=escalabilidade, 4=todos)"
        echo "  test <tipo>      - Executar testes (1=convergência, 2=dijkstra, 3=escalabilidade, 4=todos)"
        echo "  full <tipo>      - Gerar e testar (1=convergência, 2=dijkstra, 3=escalabilidade, 4=todos)"
        echo ""
        echo "Opções:"
        echo "  --skip-compile   - Pular compilação"
        exit 1
    fi
    
    mode="$1"
    test_type="$2"
    
    # Executar operação solicitada
    case "$mode" in
        "generate")
            print_status "Gerando grafos (tipo: $test_type)..."
            ./bin/generator "$test_type"
            print_success "Geração de grafos concluída!"
            ;;
            
        "test")
            case "$test_type" in
                "1") run_convergence_test ;;
                "2") run_dijkstra_test ;;
                "3") run_scalability_test ;;
                "4") 
                    run_convergence_test
                    run_dijkstra_test  
                    run_scalability_test
                    ;;
                *) print_error "Tipo de teste inválido: $test_type" ;;
            esac
            ;;
            
        "full")
            print_status "Executando pipeline completo (tipo: $test_type)..."
            ./bin/generator "$test_type"
            print_success "Geração concluída!"
            
            case "$test_type" in
                "1") run_convergence_test ;;
                "2") run_dijkstra_test ;;
                "3") run_scalability_test ;;
                "4") 
                    run_convergence_test
                    run_dijkstra_test
                    run_scalability_test
                    ;;
                *) print_error "Tipo de teste inválido: $test_type" ;;
            esac
            ;;
            
        *)
            print_error "Modo inválido: $mode"
            exit 1
            ;;
    esac
    
    print_success "Operação concluída!"
}

# Verificar se bc está instalado (necessário para cálculos)
if ! command -v bc &> /dev/null; then
    print_error "bc não está instalado. Instale com: sudo apt-get install bc"
    exit 1
fi

# Executar função principal
main "$@"