import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


original_points = np.array([(0,0), (2.38,0), (2.67, 1.43), (-1.11, 0.8)])

# archivos_csv = ['cartographer.csv']
# archivos_csv = ['trans_gmapping.csv']
# colores = ['red']
archivos_csv = ['trans_gmapping.csv', 'trans_slam_toolbox.csv', "trans_hector.csv", "trans_rtabmap.csv", "trans_cartographer.csv"]
colores = ['red', 'blue', "orange", "purple", "green"]

# Crea la figura y los ejes
fig, ax = plt.subplots()

for archivo, color in zip(archivos_csv, colores):
    # Lee el archivo CSV
    data = pd.read_csv(archivo, sep=" ")
    
    # Extrae las columnas 'x' y 'y' del DataFrame
    x = data['x'].to_numpy()
    y = data['y'].to_numpy()
    
    # Grafica los puntos del archivo CSV con el color correspondiente
    ax.scatter(x, y, label=archivo, color=color, s=10)
    ax.plot(x, y, color=color)

ax.plot(original_points[:, 0], original_points[:, 1], color='black', label='trayectoria original')

ax.set_xlabel('Eje X')
ax.set_ylabel('Eje Y')
ax.set_title('Comparacion de trayectorias')
ax.legend()
plt.show()

