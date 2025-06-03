#!/bin/bash

# Compilar o programa (assumindo que o Makefile está configurado)
make

# Verificar se a compilação foi bem sucedida
if [ $? -ne 0 ]; then
    echo "Erro na compilação!"
    exit 1
fi

# Contador para os testes
total=0
passed=0

# Executar para cada arquivo .dat
for input in data/default/*.dat; do
    filename=$(basename "$input")
    echo "Testando $filename..."
    
    # Executar o programa e capturar saída e erro
    output=$(./bin/emparelhamento < "$input" 2>&1)
    
    # Verificar se a execução foi bem sucedida
    if [ $? -eq 0 ]; then
        echo "✅ Passou"
        ((passed++))
    else
        echo "❌ Falhou"
        echo "Erro:"
        echo "$output"
    fi
    
    ((total++))
    echo "----------------------------------------"
done

# Imprimir resumo
echo "Resumo dos testes:"
echo "Total de testes: $total"
echo "Testes passados: $passed"
echo "Testes falhados: $((total - passed))"

# Calcular porcentagem de sucesso
success_rate=$(echo "scale=2; ($passed * 100) / $total" | bc)
echo "Taxa de sucesso: ${success_rate}%"