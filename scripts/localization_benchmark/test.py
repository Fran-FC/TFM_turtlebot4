import math

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

# Puntos de ejemplo
# punto_A = (1, 2)
# punto_B = (4, 5)
r1_A = (2.67, 1.43)
r1_B = (-1.11, 0.8)

r2_A = (0,0)
r2_B = (2.38,0)

# Calcular el ángulo
angulo = get_angle_between_vectors(r1_A, r1_B, r2_A, r2_B)
print("El ángulo con respecto al eje x es:", angulo)