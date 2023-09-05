#!/bin/sh
d_name=old_`date +%d-%b-%Y-%H-%M`
path='/home/rogerio/git/sim-res/datafile'
for d in 'density-oriented' 'uniform' 'equidistant'; do
    mkdir -pv $path/$d/results/$d_name
    tar czf $path/$d/results/$d_name/results.tar.gz $path/$d/results/trans*
    rm $path/$d/results/transmission*
done 