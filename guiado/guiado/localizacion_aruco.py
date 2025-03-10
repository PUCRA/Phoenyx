import cv2
import numpy as np
import os
import math

# Posiciones conocidas de los ArUcos en el mundo (x, y)
aruco_positions = {
    0: (1.0, 1.0),
    1: (2.0, 1.0),
    2: (1.0, 2.0),
    3: (2.0, 2.0)
}

def triangulate_position(aruco_positions, detected_aruco_ids, tvecs):
    """Realiza la triangulación para determinar la posición del robot."""
    if len(detected_aruco_ids) < 2:
        print("Se necesitan al menos 2 ArUcos para la triangulación.")
        return None

    # Obtener las posiciones conocidas de los ArUcos detectados
    positions = [aruco_positions[id[0]] for id in detected_aruco_ids]
    # Obtener las distancias medidas desde el robot a los ArUcos
    distances = [np.linalg.norm(tvec[0][0]) for tvec in tvecs]

    # Resolver el sistema de ecuaciones para encontrar la posición del robot
    A = []
    B = []
    for (x, y), d in zip(positions, distances):
        A.append([2*x, 2*y, 1])
        B.append([x**2 + y**2 - d**2])
    A = np.array(A)
    B = np.array(B)
    pos = np.linalg.lstsq(A, B, rcond=None)[0]
    return pos[0][0], pos[1][0]

# === PROGRAMA PRINCIPAL ===
def main():
    # IDs de los ArUcos detectados (ejemplo)
    detected_aruco_ids = np.array([[0], [1]])

    # Vectores de traslación de los ArUcos detectados (ejemplo)
    tvecs = [np.array([[[1.0, 0.0, 0.0]]]), np.array([[[2.0, 0.0, 0.0]]])]

    # Realizar la triangulación
    robot_position = triangulate_position(aruco_positions, detected_aruco_ids, tvecs)
    if robot_position is not None:
        print(f"Posición del robot: X={robot_position[0]:.3f}, Y={robot_position[1]:.3f}")
    else:
        print("No se pudo determinar la posición del robot.")

if __name__ == "__main__":
    main()