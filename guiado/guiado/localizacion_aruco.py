import numpy as np
import math

# Posiciones conocidas de los ArUcos en el mundo (x, y)
aruco_positions = {
    0: (0, 1.0),
    1: (2.0, 0.0),
    2: (1.0, 2.0),
    3: (2.0, 2.0),
    4: (3.0, 7.0),
    5: (7.0, 3.0)
}
posXabs = 0  # Asignar un valor adecuado
posZabs = 0  # Asignar un valor adecuado
AngleRobot = 0  # Asignar un valor adecuado
thetaArucoAbs = 0  


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
        thetaArucoAbs = 270
        posXabs = aruco_positions[aruco_id][0] - Xrel
        posZabs = aruco_positions[aruco_id][1] - Zrel
        

    AngleRobot = thetaArucoAbs - thetaArucoRel

    if AngleRobot >= 360:
        AngleRobot = AngleRobot - 360 #encontraremos angulos mas grandes de 720

    print(f"Posición del robot: X={posXabs:.3f}, Y={posZabs:.3f}, Ángulo={AngleRobot:.3f}")
    

# === PROGRAMA PRINCIPAL ===
def main():

    calculate_robot_pos2(-2, 5, 5, 180)


if __name__ == "__main__":
    main()