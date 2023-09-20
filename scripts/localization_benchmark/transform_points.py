import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

def rotate_by_axis(points, angulo, axis):
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

def calc_slope(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    if dx == 0:
        return float('inf')  # Pendiente infinita (recta vertical)
    return dy / dx

def get_angle_between_vectors(punto1_r1, punto2_r1, punto1_r2, punto2_r2):
    pendiente_r1 = calc_slope(punto1_r1, punto2_r1)
    pendiente_r2 = calc_slope(punto1_r2, punto2_r2)

    # Calcular el ángulo entre las pendientes utilizando la fórmula del ángulo entre dos líneas
    angulo_radianes = abs(math.atan((pendiente_r2 - pendiente_r1) / (1 + pendiente_r1 * pendiente_r2)))
    angulo_grados = math.degrees(angulo_radianes)

    return angulo_grados

if __name__=="__main__":
    # Leer el archivo CSV
    file_name = 'rtabmap.csv'

    transform_t1_algos = ["slam_toolbox.csv", "gmapping.csv", "rtabmap.csv"]
    transform_t2_algos = ["hector.csv", "cartographer.csv"]

    data = pd.read_csv(file_name, sep=" ")

    # real trajectory
    rt = [(0,0), (2.38,0), (2.67, 1.43), (-1.11, 0.8)]

    # Extraer las columnas 'x' y 'y' del DataFrame
    x = data['x'].to_numpy()
    y = data['y'].to_numpy()
    z = np.zeros(y.size)

    # Convertir los datos a un arreglo NumPy
    points = np.column_stack((x, y, z))

    # transform the points
    if file_name in transform_t1_algos:
        points = rotate_by_axis(points, 180, 'y')
        alpha = get_angle_between_vectors(rt[0], rt[1], rt[2], rt[3]) 
        points = rotate_by_axis(points, alpha - 90, 'z')
    elif file_name in transform_t2_algos:
        points = rotate_by_axis(points, 180, 'x')

    d = calculate_translation_distance(points[0], (0,0,0))
    points = translate_points(points, d)

    # Extraer las coordenadas x e y 
    rotated_x = points[:, 0]
    rotated_y = points[:, 1]

    save_points_to_csv(rotated_x, rotated_y, file_name)

    # Crear el gráfico
    fig, ax = plt.subplots()

    ax.scatter(rotated_x, rotated_y, color='red', label='Trayectoria real')
    ax.plot(rotated_x, rotated_y, color='red')

    ax.scatter(x, y, color='blue', label='Trayectoria calculada')
    ax.plot(x, y, color='blue')

    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_title('Comparación de trayectorias')
    ax.legend()
    plt.show()