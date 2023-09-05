#!/bin/sh
startapp=$(date +%s.%N)
for s in $(seq 1 50); do
  starts=$(date +%s.%N)
  for g in 1 2 3 4 5 6 7 8 9 10 12 14 16 25 81 289; do
    startg=$(date +%s.%N)
    d50="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    d100="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=250 --nGateways=$g --seed=$s"
    d150="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=500 --nGateways=$g --seed=$s"
    d200="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=750 --nGateways=$g --seed=$s"
    d250="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=1000 --nGateways=$g --seed=$s"
    e50="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    e100="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=250 --nGateways=$g --seed=$s"
    e150="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=500 --nGateways=$g --seed=$s"
    e200="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=750 --nGateways=$g --seed=$s"
    e250="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=1000 --nGateways=$g --seed=$s"
    u50="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    u100="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=250 --nGateways=$g --seed=$s"
    u150="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=500 --nGateways=$g --seed=$s"
    u200="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=750 --nGateways=$g --seed=$s"
    u250="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=1000 --nGateways=$g --seed=$s"
    mpirun $u250 : $d250 : $e250 : $u200 : $d50 & 
    mpirun $d200 : $e200 : $d150 : $e150 : $u50 &
    mpirun $u150 : $u100 : $e100 : $d100 : $e50
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
