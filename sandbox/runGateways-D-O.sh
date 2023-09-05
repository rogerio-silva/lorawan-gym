#!/bin/sh
for s in $(seq 1 50); do #50 rodadas para IC
  for d in 10 20 30 40 50; do
    start=$(date +%s.%N)
    for g in $(seq 1 30); do
      ./ns3.36-gateways-density-oriented-distrib-debug --nGateways=$g --seed=$s --nDevices=$d
    done
    end=$(date +%s.%N)
  done
  runtime=$(python -c "print(${end} - ${start})")
  execs=$(python -c "print(${s} * ${g} * ${d})")
  echo "Runtime [$s] was $runtime. [$execs] Runs."
done
