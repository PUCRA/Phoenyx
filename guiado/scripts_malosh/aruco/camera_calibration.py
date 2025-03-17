import cv2
import numpy as np


# Configuración
CHESSBOARD_SIZE = (9, 6)  # Número de esquinas internas del patrón de ajedrez (columnas, filas)
SQUARE_SIZE = 0.0235  # Tamaño real de cada cuadrado en metros
TOTAL_IMAGES = 100  # Número de imágenes para calibración

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Puntos del patrón de ajedrez en el mundo real
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

objpoints = []  # Puntos 3D en el mundo real
imgpoints = []  # Puntos 2D en la imagen

# Inicializar cámara
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: No se puede acceder a la cámara.")
    exit()

print("Cámara detectada correctamente. Capturando imágenes para calibración...")

captured_images = 0
while captured_images < TOTAL_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Error al capturar la imagen.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if found:
        objpoints.append(objp)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)
        captured_images += 1
        print(f"Imagen {captured_images}/{TOTAL_IMAGES} capturada correctamente.")

cap.release()

# Calibración de la cámara
if len(objpoints) >= TOTAL_IMAGES:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\nResultados de la Calibración")
    print(f"Error de reproyección: {ret:.5f}")
    print("\nMatriz de Cámara:")
    print(camera_matrix)
    print("\nCoeficientes de Distorsión:")
    print(dist_coeffs)

    np.save("camera_matrix.npy", camera_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)
    print("\nParámetros guardados en 'camera_matrix.npy' y 'dist_coeffs.npy'.")
    print("\nParámetros guardados en 'camera_matrix.npy' y 'dist_coeffs.npy'.")

else:
    print("\nNo se capturaron suficientes imágenes para la calibración. Inténtalo de nuevo.")
