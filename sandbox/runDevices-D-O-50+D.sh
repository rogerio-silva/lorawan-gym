#!/bin/sh
# Gerar Dispositivos conforme densidade
for s in $(seq 1 50); do #50 rodadas para IC
  start=$(date +%s.%N)
  for d in 60 70 80 90 100; do # 100 200 300 400 500; do
    ./ns3.36-devices-density-oriented-distrib-debug --nDevices=$d --seed=$s
    # Gerar distribuição de Gateways pelo método DO
    #for g in $(seq 1 30); do
    #  ./ns3.36-gateways-density-oriented-distrib-debug --nGateways=$g --seed=$s --nDevices=$d
    #done
  done
  end=$(date +%s.%N)
  runtime=$(python -c "print(${end} - ${start})")
  echo "Runtime was [$runtime]. Run:[$s]"
done