rm packetsPerDR/*
for i in {0..5} ; do
    ./ns3 run "scratch/simpleLorawanSlice-Example1 --dR="$i""
done
