import cv2
import numpy as np
import pytesseract
import traceback

class image2recorte():
    def __init__(self):
        pass

    def ordenar_puntos(self, pts):
        # Convertir a un array 2D normal si es necesario
        pts = pts.reshape(4, 2)
        # Ordenar por suma (x + y) para identificar esquinas
        suma = pts.sum(axis=1)
        diferencia = np.diff(pts, axis=1)
        # Superior izquierdo: menor suma
        punto_sup_izq = pts[np.argmin(suma)]
        # Inferior derecho: mayor suma
        punto_inf_der = pts[np.argmax(suma)]
        # Superior derecho: menor diferencia (x - y)
        punto_sup_der = pts[np.argmin(diferencia)]
        # Inferior izquierdo: mayor diferencia (x - y)
        punto_inf_izq = pts[np.argmax(diferencia)]
        return np.array([punto_sup_izq, punto_sup_der, punto_inf_izq, punto_inf_der], dtype=np.float32)
    
    # def detectar_contornos2(self, frame):

    #     # Convertir a escala de grises y aplicar desenfoque
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #     # Aplicar detector de bordes Canny
    #     edges = cv2.Canny(blurred, 50, 150)
    #     # Detectar contornos
    #     contornos, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     for contorno in contornos:
    #         # Aproximar el contorno a un polígono
    #         epsilon = 0.02 * cv2.arcLength(contorno, True)  # Tolerancia más baja
    #         approx = cv2.approxPolyDP(contorno, epsilon, True)
    #         # Si el contorno tiene 4 vértices, podría ser un cuadrado
    #         if len(approx) == 4 and cv2.isContourConvex(approx):
    #             x, y, w, h = cv2.boundingRect(approx)
    #             aspect_ratio = float(w) / h  # Relación de aspecto
    #             # Verificar que la relación de aspecto sea aproximadamente 1
    #             if 0.95 <= aspect_ratio <= 1.05 and w * h > 5000:
    #                 # Verificar que los ángulos sean cercanos a 90 grados
    #                 angles = []
    #                 for i in range(4):
    #                     p1 = approx[i][0]
    #                     p2 = approx[(i + 1) % 4][0]
    #                     p3 = approx[(i + 2) % 4][0]
    #                     v1 = p1 - p2
    #                     v2 = p3 - p2
    #                     cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    #                     angle = np.arccos(cosine_angle) * (180.0 / np.pi)
    #                     angles.append(angle)
    #                 # Comprobar si los ángulos están cerca de 90 grados
    #                 if all(85 <= ang <= 95 for ang in angles):
    #                     cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
    #                     print("🟢 Se ha detectado un cuadrado")
    #                     # Extraer solo la parte interna del cuadrado
    #                     mask = np.zeros_like(frame)
    #                     cv2.drawContours(mask, [approx], -1, (255, 255, 255), -1)
    #                     result = cv2.bitwise_and(frame, mask)
    #                     # Recortar la región del cuadrado
    #                     cropped = frame[y:y+h, x:x+w]
    #                     # Mostrar la imagen solo con el cuadrado detectado
    #                     cv2.imshow("Cuadrado Detectado", cropped)
    #     # Mostrar imágenes de depuración
    #     cv2.imshow("Bordes Canny", edges)
    #     cv2.imshow("Detección General", frame)
        
    def detectar_contornos(self, frame):
        vertices = []
        contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4:
                    area = cv2.contourArea(approx)
                    # print(area)
                    if area < 40000 and area > 5000:
                        if len(approx) == 4:
                            x, y, w, h = cv2.boundingRect(approx)
                            aspect_ratio = float(w) / h  # Relación de aspecto
                            # Verificar que la relación de aspecto sea aproximadamente 1
                            if 0.9 <= aspect_ratio <= 1.1:
                                # Verificar que los ángulos sean cercanos a 90 grados
                                angles = []
                                for i in range(4):
                                    p1 = approx[i][0]
                                    p2 = approx[(i + 1) % 4][0]
                                    p3 = approx[(i + 2) % 4][0]
                                    v1 = p1 - p2
                                    v2 = p3 - p2
                                    cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                                    angle = np.arccos(cosine_angle) * (180.0 / np.pi)
                                    angles.append(angle)
                                # Comprobar si los ángulos están cerca de 90 grados
                                if all(80 <= ang <= 100 for ang in angles):
                                    # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                                    # print("🟢 Se ha detectado un cu
                                    return approx                    
        return vertices

    # def detectar_contornos(self, frame):
    #     vertices = []
    #     contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     for contour in contours:
    #         # Calcular el área y el perímetro
    #         area = cv2.contourArea(contour)
    #         perimeter = cv2.arcLength(contour, True)

    #         # Calcular la circularidad
    #         if perimeter > 0:
    #             circularity = 4 * np.pi * area / (perimeter ** 2)

    #             # Filtrar formas que tengan una circularidad que corresponda a un cuadrado o rectángulo con bordes redondeados
    #             if 0.7 < circularity < 1.2:
    #                 # Si tiene la circularidad esperada, puede ser un cuadrado con bordes redondeados
    #                 # A continuación, se puede aplicar más lógica si es necesario, como verificar el área
    #                 if area > 2000:
    #                     vertices = contour
    #                     break
    #     return vertices

    def obtener_recorte(self, frame: np.ndarray, log_level=0):
        try:
            if not isinstance(frame, np.ndarray):
                raise ValueError("El parámetro de entrada debe ser una imagen de OpenCV (numpy.ndarray).")
            # img_peq = frame[40:440, 120:520]
            # copy_image = img_peq.copy()
            # detected_color = None
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  

            # Mostrar la imagen HSV
            if log_level == 1:
                cv2.imshow('Frame Image', frame)

            # Definir el rango para el color azul
            # lower_blue = np.array([100, 150, 50])
            # upper_blue = np.array([140, 255, 255])
            # lower_red_1 = np.array([0, 50, 50])
            # upper_red_1 = np.array([10, 255, 255])
            # lower_red_2 = np.array([170, 50, 50])
            # upper_red_2 = np.array([180, 255, 255])

            # Crear máscaras para azul y rojo
            # mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
            # mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
            # mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
            # mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
            # combined_mask = cv2.bitwise_or(mask_blue, mask_red)
            combined_mask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # combined_mask = cv2.equalizeHist(combined_mask)
            # combined_mask = cv2.GaussianBlur(combined_mask, (5, 5), 0)
            _, combined_mask = cv2.threshold(combined_mask, 120, 255, cv2.THRESH_BINARY)
            # cleaned_mask = combined_mask
            if log_level == 1:
                cv2.imshow('Combined Mask', combined_mask)

            # # Tratamiento morfológico
            
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            close_img = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            cleaned_mask = cv2.morphologyEx(close_img, cv2.MORPH_OPEN, kernel)
            cleaned_mask = close_img
            cleaned_mask = cv2.bitwise_not(cleaned_mask)
            if log_level == 1:
                cv2.imshow('Cleaned Mask', cleaned_mask)

            vertices = self.detectar_contornos(cleaned_mask)
            # vertices = self.detectar_contornos2(combined_mask)

            # if log_level == 1:
                # cv2.imshow('Detected Squares', copy_image)
            # print(vertices)
            if len(vertices) == 0:
                return None

            puntos_origen = np.array([vertices[2][0], vertices[3][0], vertices[0][0], vertices[1][0]], np.float32)
            puntos_origen = self.ordenar_puntos(puntos_origen)

            mask_black = np.zeros_like(frame)
            cv2.fillPoly(mask_black, [vertices], (255, 255, 255))
            result = cv2.bitwise_and(frame, mask_black)

            if log_level == 1:
                cv2.imshow('Masked Region', result)

            ancho = 800
            alto = 800
            puntos_destino = np.float32([[0, 0], [ancho, 0], [0, alto], [ancho, alto]])
            matriz = cv2.getPerspectiveTransform(puntos_origen, puntos_destino)
            numero_cuadrado = cv2.warpPerspective(result, matriz, (ancho, alto))

            if log_level == 1:
                cv2.imshow('Corrected Image', numero_cuadrado)

            return numero_cuadrado

        except Exception as e:
            print(f"Ocurrió un error: {e}")
            print("Error en la línea:", traceback.format_exc())
            return None
