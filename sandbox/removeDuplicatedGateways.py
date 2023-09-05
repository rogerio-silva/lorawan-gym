import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from glob import glob

path = '/home/rogerio/git/sim-res/datafile/optimized-oriented/to_NS3/optimizedPlacement10x1*.dat'

files = sorted(glob(path))

for file in files:
    data = pd.read_csv(file, names=['x', 'y', 'z'], sep=" ", index_col=False)
    data.drop_duplicates(inplace=True)
    newfilename = file[0:80] + "optGPlacement_" + file[98:]
    data.to_csv(newfilename, header=False, sep=" ", index=False)
    print(newfilename + " SAVED!")