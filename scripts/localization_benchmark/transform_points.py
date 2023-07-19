import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def rotate_y_axis(points, angulo, axis):
    # Convertir el ángulo a radianes
    angulo = np.radians(angulo)
    if axis == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(angulo), -np.sin(angulo)],
                                    [0, np.sin(angulo), np.cos(angulo)]])
    elif axis == 'y':
        rotation_matrix = np.array([[np.cos(angulo), 0, np.sin(angulo)],
                                    [0, 1, 0],
                                    [-np.sin(angulo), 0, np.cos(angulo)]])
    elif axis == 'z':
        rotation_matrix = np.array([[np.cos(angulo), -np.sin(angulo), 0],
                                    [np.sin(angulo), np.cos(angulo), 0],
                                    [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")

    # Aplicar la rotación a cada punto individualmente
    rotated_points = []
    for point in points:
        rotated_point = np.dot(rotation_matrix, point)
        rotated_points.append(rotated_point)

    # Convertir los puntos rotados a un arreglo NumPy
    return np.array(rotated_points)


def calculate_translation_distance(point1, point2):
    vector = np.array(point2) - np.array(point1)
    return vector


def translate_points(points, translation_distances):
    translated_points = []
    for point in points:
        translated_point = (point[0] + translation_distances[0],
                           point[1] + translation_distances[1],
                           point[2] + translation_distances[2])
        translated_points.append(translated_point)
    return np.array(translated_points)

def save_points_to_csv(xs,ys,file_name):
    data = np.column_stack((xs, ys))
    np.savetxt("trans_"+file_name, data, delimiter=' ', header='x y', comments='')

# Leer el archivo CSV
file_name = 'cartographer.csv'
data = pd.read_csv(file_name, sep=" ")

# Extraer las columnas 'x' y 'y' del DataFrame
x = data['x'].to_numpy()
y = data['y'].to_numpy()
z = np.zeros(y.size)

# Convertir los datos a un arreglo NumPy
points = np.column_stack((x, y, z))

# transform the points
# points = rotate_y_axis(points, 80, 'z')
points = rotate_y_axis(points, 180, 'x')
d = calculate_translation_distance(points[0], (0,0,0))
points = translate_points(points, d)

# Extraer las coordenadas x y y después de la rotación
rotated_x = points[:, 0]
rotated_y = points[:, 1]

save_points_to_csv(rotated_x, rotated_y, file_name)

# Crear el gráfico
fig, ax = plt.subplots()

# for i in range(len(points) - 1):
#     ax.plot([rotated_x[i], rotated_x[i + 1]], [rotated_y[i], rotated_y[i + 1]], c='red')

# Graficar los puntos rotados
ax.scatter(rotated_x, rotated_y, color='red', label='Puntos Rotados')

ax.scatter(x, y, color='blue', label='Puntos')

# Etiqueta de los ejes
ax.set_xlabel('Eje X')
ax.set_ylabel('Eje Y')

# Título del gráfico
ax.set_title('Puntos Rotados')

# Agregar una leyenda
ax.legend()

# Mostrar el gráfico
plt.show()