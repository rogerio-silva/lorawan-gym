#!/bin/bash

pyapp_path="./scratch/lorawan-gym/"

# Verifica se os caminhos são válidos
if [ ! -f "$app_path" ]; then
    echo "Erro: O arquivo não foi encontrado em '$pyapp_path'."
    exit 1
fi

# Execute o executável Linux
echo "Executando a simulação ns-3..."
./ns3 run "lorawan_gym"

# Execute o aplicativo Python
echo "Executando o agente RL Python..."
python3 "$pyapp_path" + "agent.py"


