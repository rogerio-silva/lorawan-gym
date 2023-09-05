#!/bin/sh
startapp=$(date +%s.%N)
for s in $(seq 1 50); do
  starts=$(date +%s.%N)
  for g in $(seq 11 20); do
    startg=$(date +%s.%N)
    d50="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=10 --nGateways=$g --seed=$s"
    d100="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=20 --nGateways=$g --seed=$s"
    d150="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=30 --nGateways=$g --seed=$s"
    d200="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=40 --nGateways=$g --seed=$s"
    d250="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=50 --nGateways=$g --seed=$s"
    e50="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=10 --nGateways=$g --seed=$s"
    e100="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=20 --nGateways=$g --seed=$s"
    e150="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=30 --nGateways=$g --seed=$s"
    e200="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=40 --nGateways=$g --seed=$s"
    e250="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=50 --nGateways=$g --seed=$s"
    # u50="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=10 --nGateways=$g --seed=$s"
    # u100="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=20 --nGateways=$g --seed=$s"
    # u150="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=30 --nGateways=$g --seed=$s"
    # u200="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=40 --nGateways=$g --seed=$s"
    # u250="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=50 --nGateways=$g --seed=$s"
    mpirun $d250 : $e250 : $d50 : $d200 : $e200   &
    mpirun $e100 : $d100 : $e50 : $d150 : $e150
    endg=$(date +%s.%N)
    runtime=$(python -c "print(${endg} - ${startg})")
    echo "Runtime [Gateways: $g] was $runtime."
  done
  ends=$(date +%s.%N)
  runtime=$(python -c "print(${ends} - ${starts})")
  echo "Runtime [Seed: $s] was $runtime."
done
endapp=$(date +%s.%N)
runtime=$(python -c "print(${endapp} - ${startapp})")
echo "Runtime was $runtime."
