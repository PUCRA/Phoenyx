import numpy as np
import math

# Posiciones conocidas de los ArUcos en el mundo (x, y)
aruco_positions = {
    0: (0, 1.0),
    1: (2.0, 1.0),
    2: (1.0, 2.0),
    3: (2.0, 2.0)
}

def calculate_robot_pos(x1, y1, x2, y2, theta1, theta2):
    # Convertir ángulos de grados a radianes si es necesario
    theta1 = math.radians(90-theta1)# el angulo que dan los arucos es el contrario al que se necesita
    theta2 = math.radians(90-theta2)
    
    # Calcular x3
    x3 = ((y1 - y2) + x2 * math.tan(theta2) - x1 * math.tan(theta1)) / (math.tan(theta2) - math.tan(theta1))
    
    # Calcular y3
    y3 = ((y1 * math.tan(theta2)) - (y2 * math.tan(theta1)) - ((x1 - x2) * math.tan(theta2) * math.tan(theta1))) / (math.tan(theta2) - math.tan(theta1))

    return x3, y3

# === PROGRAMA PRINCIPAL ===
def main():
    # IDs de los ArUcos detectados (ejemplo)
    detected_aruco_ids = [0, 1]

    if len(detected_aruco_ids) >= 2:
        # Obtener las posiciones de los ArUcos detectados
        id1, id2 = detected_aruco_ids[:2]
        (x1, y1) = aruco_positions[id1]
        (x2, y2) = aruco_positions[id2]

        # Ejemplo de ángulos (deberían ser obtenidos de alguna manera)
        theta1, theta2 = 45, 26.57  # Ángulos en grados

        # Realizar la triangulación
        robot_position = calculate_robot_pos(x1, y1, x2, y2, theta1, theta2)
        if robot_position is not None:
            print(f"Posición del robot: X={robot_position[0]:.3f}, Y={robot_position[1]:.3f}")
        else:
            print("No se pudo determinar la posición del robot.")
    else:
        print("Se necesitan al menos dos ArUcos detectados para calcular la posición del robot.")

if __name__ == "__main__":
    main()