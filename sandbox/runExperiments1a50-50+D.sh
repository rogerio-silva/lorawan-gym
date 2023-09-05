#!/bin/sh
# Executar experimentos para {10, 20, 30, 40, 50} Devices e {1 .. 50} Gateways
startapp=$(date +%s.%N)
for s in $(seq 1 50); do  #Rodadas para IC
  starts=$(date +%s.%N)
  for g in $(seq 1 50); do # Gateways
    startg=$(date +%s.%N)
    d10="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=60 --nGateways=$g --seed=$s"
    e10="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=60 --nGateways=$g --seed=$s"
    u10="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=60 --nGateways=$g --seed=$s"
    mpirun $e10 : $d10 : $u10 &
    d20="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=70 --nGateways=$g --seed=$s"
    e20="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=70 --nGateways=$g --seed=$s"
    u20="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=70 --nGateways=$g --seed=$s"
    mpirun $e20 : $d20 : $u20 &
    d30="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=80 --nGateways=$g --seed=$s"
    d40="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=90 --nGateways=$g --seed=$s"
    d50="-np 1 ns3.36-density-oriented-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    e30="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=80 --nGateways=$g --seed=$s"
    e40="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=90 --nGateways=$g --seed=$s"
    e50="-np 1 ns3.36-equidistant-distrib-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    u30="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=80 --nGateways=$g --seed=$s"
    u40="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=90 --nGateways=$g --seed=$s"
    u50="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=100 --nGateways=$g --seed=$s"
    mpirun $u30 : $u40 : $u50 &
    mpirun $d30 : $d40 : $d50 &
    mpirun $e30 : $e40 : $e50
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
