import cv2
import numpy as np

# === CONFIGURACIÓN ===
# Selecciona el diccionario de ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

# Parámetros para la detección
parameters = cv2.aruco.DetectorParameters()

# Longitud del lado del marcador ArUco en metros
aruco_marker_length = 0.1745

# Matriz de cámara y coeficientes de distorsión
camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')
 

# === FUNCIONES ===
def detect_aruco_and_estimate_pose(frame):
    """
    Detecta ArUco markers en el frame y estima su posición y orientación.
    """
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detectar marcadores ArUco
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Si se detecta al menos un marcador
    if ids is not None:
        # Dibujar bordes de los marcadores detectados
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Iterar sobre los marcadores detectados
        for corner, marker_id in zip(corners, ids):
            # Estimar pose del marcador
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, aruco_marker_length, camera_matrix, dist_coeffs)

            # Dibujar el eje del marcador
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            # Imprimir pose del marcador
            print_pose(marker_id, tvec, rvec)

    return frame


def print_pose(marker_id, tvec, rvec):
    """
    Imprime la información del marcador detectado con sus unidades.
    """
    print("\n=== ArUco Marker Detected ===")
    print(f"Marker ID: {marker_id[0]}")
    print("Translation Vector (tvec):")
    print(f"  X: {tvec[0][0][0]:.3f} m")
    print(f"  Y: {tvec[0][0][1]:.3f} m")
    print(f"  Z: {tvec[0][0][2]:.3f} m")
    print("Rotation Vector (rvec):")
    print(f"  Rx: {rvec[0][0][0]:.3f} rad")
    print(f"  Ry: {rvec[0][0][1]:.3f} rad")
    print(f"  Rz: {rvec[0][0][2]:.3f} rad")


# === PROGRAMA PRINCIPAL ===
def main():
    """
    Captura video desde la cámara, detecta ArUco markers y muestra resultados.
    """
    # Inicializar la cámara
    camera_index = 2  # Cambiar este índice para seleccionar otra cámara
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Error: No se puede acceder a la cámara en índice {camera_index}.")
        return

    print("Cámara detectada correctamente.")
    print("Presiona 'q' para salir.")

    while True:
        # Capturar un frame de la cámara
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar el frame.")
            break

        # Detectar marcadores y estimar su pose
        frame_with_markers = detect_aruco_and_estimate_pose(frame)

        # Mostrar el frame procesado
        cv2.imshow('ArUco Marker Detection', frame_with_markers)

        # Salir al presionar 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
