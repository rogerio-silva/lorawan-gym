#!/bin/sh
for s in $(seq 1 50); do
  start=$(date +%s.%N)
  for d in 10 20 30; do
    ./ns3.36-optimal-distrib-experiment-debug --nDevices=$d --seed=$s --nGateways=10
  done
  end=$(date +%s.%N)
  runtime=$(python -c "print(${end} - ${start})")
  echo "Runtime [$s] was $runtime."
done
python3 ../../optimizer/model/sound.py
