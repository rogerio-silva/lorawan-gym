import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from glob import glob
from datetime import datetime

dh = datetime.now().strftime('%Y%m%d-%H-%M_')
# devXGw = {'op': {10:3, 20}}

devices = 10
gateways = 1

fileD = "/home/rogerio/git/sim-res/datafile/devices/placement/endDevices_LNM_Placement_1s+10d.dat"
graph_title = "GWxDEV_Placement"


dataG = {'x': [6500], 'y': [7000], 'z': [30]}
dfG = pd.DataFrame(dataG)
dataD = pd.read_csv(fileD, names=['x', 'y', 'z'], sep=" ", index_col=False)
dfD = pd.DataFrame(dataD)

# Scatter plot para dfG
plt.scatter(dfG['x'], dfG['y'], label='dfG', color='red', marker='x')

# Scatter plot para dfD
plt.scatter(dfD['x'], dfD['y'], label='dfD', color='blue', marker='o')

plt.xlim(-500, 10500)
plt.ylim(-500, 10500)
# Configuração do gráfico
plt.xlabel('Eixo X')
plt.ylabel('Eixo Y')
# plt.legend(loc='best')
plt.title(graph_title)
plt.grid(False)

# Exibir o gráfico
plt.show()
