import math
import numpy as np
import pandas as pd

def get_start_end_index(start_point, end_point, point_list):
    start_index = np.where(np.all(point_list == start_point, axis=1))[0][0]
    end_index = np.where(np.all(point_list == end_point, axis=1))[0][0]

    return start_index, end_index


def distance_point_segment(A, r1, r2): 
    # x, y, x1, y1, x2, y2
    x = A[0]
    y = A[1]
    x1 = r1[0]
    y1 = r1[1]
    x2 = r2[0]
    y2 = r2[1]

    # Calcula la distancia entre el punto P y los extremos del segmento AB
    dist_PA = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
    dist_PB = math.sqrt((x - x2) ** 2 + (y - y2) ** 2)

    # Calcula la longitud del segmento AB
    longitud_AB = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Calcula la distancia mínima entre el punto P y el segmento AB
    if dist_PA <= dist_PB and 0 <= (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1) <= longitud_AB ** 2:
        distancia = abs((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / longitud_AB
    else:
        distancia = min(dist_PA, dist_PB)

    return distancia


def euclidean_distance(A, B):
    return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)


def get_nearest_point(target_point, point_list):
    # Initialize variables for tracking the nearest point and minimum distance
    nearest_point = None
    min_distance = float('inf')

    # Iterate through each point in the list
    for point in point_list:
        # Calculate the Euclidean distance between the target point and the current point
        distance = euclidean_distance(target_point, point)

        # Update the nearest point if a closer point is found
        if distance < min_distance:
            min_distance = distance
            nearest_point = point

    return nearest_point

real_points = [(0,0), (2.38,0), (2.67, 1.43), (-1.11, 0.8)]
archivos_csv = ['trans_gmapping.csv', 'trans_slam_toolbox.csv', "trans_hector.csv", "trans_rtabmap.csv", "trans_cartographer.csv"]
algoritmos = ['gmapping', 'slam_toolbox', "hector", "rtabmap", "cartographer"]

for archivo, algo in zip(archivos_csv, algoritmos):
    theo_points = pd.read_csv(archivo, sep=" ")

    x = theo_points['x']
    y = theo_points['y']

    acum_err = 0
    max_err = 0
    # Convertir los datos a un arreglo NumPy
    theo_points = np.column_stack((x, y))

    for i in range(len(real_points)):
        if i+1 >= len(real_points):
            break
        r_0 = real_points[i]
        r_1 = real_points[i+1] 

        t_0 = get_nearest_point(r_0, theo_points)
        t_1 = get_nearest_point(r_1, theo_points)

        acum_err += euclidean_distance(t_0, r_0) 
        acum_err += euclidean_distance(t_1, r_1) 

        start, end = get_start_end_index(t_0, t_1, theo_points) 

        for i in range(start, end):
            local_err = distance_point_segment(theo_points[i], r_0, r_1)
            acum_err += local_err
            if max_err < local_err:
                max_err = local_err

    average_error = acum_err / len(theo_points)
    print("%s: " % algo)
    print("\t average error %f" % average_error)
    print("\t max error %f" % max_err)
    print("\t cantidad de puntos %d" % len(theo_points))