import numpy as np
import math

# Posiciones conocidas de los ArUcos en el mundo (x, y)
aruco_positions = {
    0: (0, 1.0),
    1: (2.0, 1.0),
    2: (1.0, 2.0),
    3: (2.0, 2.0),
    4: (3.0, 7.0)
}
posXabs = 0  # Asignar un valor adecuado
posZabs = 0  # Asignar un valor adecuado
AngleRobot = 0  # Asignar un valor adecuado


def calculate_robot_pos2(Xrel, Zrel, aruco_id, thetaArucoRel):
    # Declarar variables no declaradas
    
    if aruco_positions[aruco_id][0] == 0: #x minimo
        thetaArucoAbs = 0
        posXabs = Zrel
        posZabs = Xrel + aruco_positions[aruco_id][1]
    elif aruco_positions[aruco_id][0] == 7: #x maximo
        thetaArucoAbs = 180
        posXabs = aruco_positions[aruco_id][0] - Zrel
        posZabs = aruco_positions[aruco_id][1] - Xrel
    elif aruco_positions[aruco_id][1] == 0: #z minimo
        thetaArucoAbs = 90
        posXabs = aruco_positions[aruco_id][0] - Xrel
        posZabs = Zrel
    elif aruco_positions[aruco_id][1] == 7: #z maximo
        thetaArucoAbs = -90
        posXabs = aruco_positions[aruco_id][0] - Xrel
        posZabs = aruco_positions[aruco_id][1] - Zrel
        
    if thetaArucoRel == 0:
        AngleRobot = thetaArucoAbs - thetaArucoRel-180
    if thetaArucoRel != 0:
        AngleRobot = thetaArucoAbs - thetaArucoRel

    # if AngleRobot > 360:
    #     AngleRobot = AngleRobot - 360 #encontraremos angulos mas grandes de 720?
    # elif AngleRobot <= 90 and AngleRobot >= 0:
    #     AngleRobot = AngleRobot - 180

    print(f"Posición del robot: X={posXabs:.3f}, Y={posZabs:.3f}, Ángulo={AngleRobot:.3f}")
    

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

    calculate_robot_pos2(1, 3, 4, 0)
    # print(f"Posición del robot: X={posXabs:.3f}, Y={posZabs:.3f}")
    # # IDs de los ArUcos detectados (ejemplo)
    # detected_aruco_ids = [0, 1]

    # if len(detected_aruco_ids) >= 2:
    #     # Obtener las posiciones de los ArUcos detectados
    #     id1, id2 = detected_aruco_ids[:2]
    #     (x1, y1) = aruco_positions[id1]
    #     (x2, y2) = aruco_positions[id2]

    #     # Ejemplo de ángulos (deberían ser obtenidos de alguna manera)
    #     theta1, theta2 = 45, 26.57  # Ángulos en grados

    #     # Realizar la triangulación
    #     robot_position = calculate_robot_pos(x1, y1, x2, y2, theta1, theta2)
    #     if robot_position is not None:
    #         print(f"Posición del robot: X={robot_position[0]:.3f}, Y={robot_position[1]:.3f}")
    #     else:
    #         print("No se pudo determinar la posición del robot.")
    # else:
    #     print("Se necesitan al menos dos ArUcos detectados para calcular la posición del robot.")

if __name__ == "__main__":
    main()