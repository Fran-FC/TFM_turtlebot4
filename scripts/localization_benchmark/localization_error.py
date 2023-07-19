import math
import numpy as np
import pandas as pd

def get_points_between(start_point, end_point, n):
    x_start, y_start = start_point
    x_end, y_end = end_point
    
    # Calcular la distancia entre los puntos en cada eje
    dx = (x_end - x_start) / (n + 1)
    dy = (y_end - y_start) / (n + 1)
    
    # Calcular los puntos intermedios
    intermediate_points = []
    for i in range(1, n + 1):
        x_intermediate = x_start + i * dx
        y_intermediate = y_start + i * dy
        intermediate_points.append((x_intermediate, y_intermediate))
    
    return intermediate_points

def euclidean_distance(A, B):
    return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

def get_start_end_index(start_point, end_point, point_list):
    start_index = np.where(np.all(point_list == start_point, axis=1))[0][0]
    end_index = np.where(np.all(point_list == end_point, axis=1))[0][0]

    return start_index, end_index

def count_points_between(start_point, end_point, point_list):
    count = 0

    start_index, end_index = get_start_end_index(start_point, end_point, point_list)
    # start_index = np.where(np.all(point_list == start_point, axis=1))[0][0]
    # end_index = np.where(np.all(point_list == end_point, axis=1))[0][0]
    
    if start_index > end_index:
        start_index, end_index = end_index, start_index
    
    for i in range(start_index + 1, end_index):
        count += 1
    
    return count


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


csv_file_points = "trans_cartographer.csv"
real_points = [(0,0), (2.38,0), (2.67, 1.43), (-1.11, 0.8)]

theo_points = pd.read_csv(csv_file_points, sep=" ")

x = theo_points['x']
y = theo_points['y']

acum_err = 0
# Convertir los datos a un arreglo NumPy
theo_points = np.column_stack((x, y))

for i in range(len(real_points)):
    if i+1 >= len(real_points):
        break
    r_0 = real_points[i]
    r_1 = real_points[i+1] 

    t_0 = get_nearest_point(r_0, theo_points)
    t_1 = get_nearest_point(r_1, theo_points)

    # num_sub_divisions = 
    r_between = get_points_between(r_0, r_1, count_points_between(t_0, t_1, theo_points))
    
    acum_err = acum_err + euclidean_distance(t_0, r_0) 
    acum_err = acum_err + euclidean_distance(t_1, r_1) 

    start, end = get_start_end_index(t_0, t_1, theo_points) 
    for r, i in zip(r_between, range(len(r_between))):
        acum_err = acum_err + euclidean_distance(r, theo_points[start+i+1])

acum_err = acum_err / len(theo_points)
print(acum_err)