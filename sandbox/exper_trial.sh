#!/bin/sh

BASEDIR="/home/ns3/ns-3-dev"
PLOTDIR="/home/ns3/ns-3-dev/plots"
ECHO="/bin/echo -e"

#cd $BASEDIR
#$ECHO "Rodando Experimentos..."
#./ns3 run "scratch/complete-lorawan-network-example --nDevices=20 --rings=1"
#./ns3 run "scratch/complete-lorawan-network-example --nDevices=20 --rings=1"

cd $BASEDIR
rm $PLOTDIR/*
$ECHO "Preparando plots..."
for ARQDAT in *.dat; do
  ARQ=$(basename "$ARQDAT" .dat)
  ARQPLT=$(basename "$ARQDAT" .dat).plt
  DEVICES=$(echo $ARQ | cut -d "_" -f 4)
  GATEWAYS=$(echo $ARQ | cut -d "_" -f 5)
  $ECHO '#Reset all previously set options\nreset\n#Set terminal up\n' \
    'set term pngcairo font "FreeSans, 10" size 1024, 768\n' \
    'set output "'$ARQ'.png"\n' \
    'set title "Devices x Spread Factor ('"$DEVICES"'x'"$GATEWAYS"')"\n' \
    '# Use a good looking palette\n' \
    "set palette defined ( 0 '#D53E4F', 1 '#F46D43', 2 '#FDAE61', 3 '#FEE08B', 4 '#E6F598', 5 '#ABDDA4', 6 '#66C2A5', 7 '#3288BD')" \
    '# Set up style for buildings\n' \
    'set style rect fc lt -1 fs solid 0.15 noborder\n' \
    '# Plot the data\n plot '\
    '"'"$ARQDAT"'" using 1:2:3 notitle with points pt 2 palette \n'\
    'unset output' >$PLOTDIR"/""$ARQPLT"
    cp "$ARQDAT" $PLOTDIR
done

cd $PLOTDIR
$ECHO "Preparando gráficos..."
for ARQPLT in *.plt; do
  gnuplot $PLOTDIR"/""$ARQPLT"
done
