#!/bin/sh
for s in $(seq 1 50); do
  start=$(date +%s.%N)
  ./ns3.36-thesis-experiments-debug --nDevices=10 --seed=$s --nGateways=10 --nPlanes=1
  ./ns3.36-thesis-experiments-debug --nDevices=20 --seed=$s --nGateways=10 --nPlanes=1
  ./ns3.36-thesis-experiments-debug --nDevices=30 --seed=$s --nGateways=10 --nPlanes=1
  ./ns3.36-thesis-experiments-debug --nDevices=40 --seed=$s --nGateways=10 --nPlanes=1
  ./ns3.36-thesis-experiments-debug --nDevices=50 --seed=$s --nGateways=10 --nPlanes=1
  end=$(date +%s.%N)
  runtime=$(python -c "print(${end} - ${start})")
  echo "Runtime was $runtime."
done
