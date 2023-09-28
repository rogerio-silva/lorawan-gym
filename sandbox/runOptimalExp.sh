#!/bin/sh
startapp=$(date +%s.%N)
for s in $(seq 1 50); do
  starts=$(date +%s.%N)
  startg=$(date +%s.%N)
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=4 --nPlanes=1 --nDevices=10 --seed=$s
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=4 --nPlanes=2 --nDevices=20 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=10 --nPlanes=1 --nDevices=10 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=10 --nPlanes=1 --nDevices=40 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=10 --nPlanes=2 --nDevices=30 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=10 --nPlanes=2 --nDevices=20 --seed=$s"   
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=12 --nPlanes=1 --nDevices=10 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=14 --nPlanes=1 --nDevices=10 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=20 --nPlanes=1 --nDevices=10 --seed=$s"
  ./ns3.36-optimal-distrib-experiment-debug --nGateways=25 --nPlanes=1 --nDevices=10 --seed=$s"
  ends=$(date +%s.%N)
  runtime=$(python -c "print(${ends} - ${starts})")
  echo "Runtime [Seed: $s] was $runtime."
done
endapp=$(date +%s.%N)
runtime=$(python -c "print(${endapp} - ${startapp})")
echo "Runtime was $runtime."
