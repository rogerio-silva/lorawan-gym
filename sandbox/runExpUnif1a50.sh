#!/bin/sh
# Executar experimentos para {10, 20, 30, 40, 50} Devices e {1 .. 50} Gateways
startapp=$(date +%s.%N)
for s in $(seq 1 50); do  #Rodadas para IC
  starts=$(date +%s.%N)
  for g in $(seq 1 50); do # Gateways
    startg=$(date +%s.%N)
    if [ $g -lt 11 ]
    then
      u10="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=10 --nGateways=$g --seed=$s"
      mpirun $u10 &
    fi
    if [ $g -lt 21 ]
    then
      u20="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=20 --nGateways=$g --seed=$s"
      mpirun $u20 &
    fi
    if [ $g -lt 31 ]
    then
      u30="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=30 --nGateways=$g --seed=$s"
      mpirun $u30 &
    fi
    if [ $g -lt 41 ]
    then
      u40="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=40 --nGateways=$g --seed=$s"
      mpirun $u40 &
    fi 
    u50="-np 1 ns3.36-uniform-distrib-experiment-debug --nDevices=50 --nGateways=$g --seed=$s"
    mpirun $u50 
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
