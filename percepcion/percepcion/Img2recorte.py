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

    def contornos_depth(self, depth_mask, color_image):
        # Encontramos los contornos en la máscara de profundidad
        contornos, _ = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        depth_image = depth_mask.copy()

        # Escala en mm por píxel (debe ser definida según la cámara)
        mm_por_pixel = 1.0  # Ajustar según la calibración de tu cámara

        for cnt in contornos:
            if cv2.contourArea(cnt) < 1000:
                continue  # Filtrar contornos pequeños

            # Rectángulo mínimo que puede rotar (mejor que boundingRect)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)  # Devuelve 4 puntos flotantes
            box = np.intp(box)         # Convierte a enteros

            # Obtener ancho y alto del rectángulo rotado
            ancho, alto = rect[1]

            # Calculamos la relación de aspecto
            if ancho == 0 or alto == 0:
                continue  # Evitar división por cero

            relacion_aspecto = max(ancho, alto) / min(ancho, alto)

            # Calcular área en píxeles
            area_pix = ancho * alto
            area_mm2 = area_pix * (mm_por_pixel ** 2)  # Convertir a mm²
            area_cm2 = area_mm2 / 100  # Convertir a cm²

            # Filtrar por área mínima de 40x40 cm (1600 cm²)
            if area_cm2 < 1600:  # 40x40 cm = 1600 cm²
                continue

            # Filtrar por relación de aspecto (no debe ser demasiado alargado)
            if relacion_aspecto < 0.8 and relacion_aspecto > 1.2:  # Ajusta este umbral según tus necesidades
                continue

            # Recorte para analizar profundidad en la región interna
            x, y, w, h = cv2.boundingRect(cnt)
            recorte = depth_image[y:y+h, x:x+w]
            profundidad_region = recorte[recorte > 0]

            if len(profundidad_region) == 0:
                continue

            # Profundidad media y desviación estándar
            profundidad_media = np.median(profundidad_region)
            desviacion = np.std(profundidad_region)

            # Verificar si la profundidad es suficientemente plana (desviación pequeña)
            if desviacion < 30:
                print(f"Contorno plano encontrado. Profundidad media: {profundidad_media} mm")
                print(f"Vértices del rectángulo: {box.tolist()}")

                # Devolvemos el recorte de la imagen de color en el área del contorno
                return color_image[y:y+h, x:x+w]  # O guarda los vértices para aplicar warp

        return None

    def obtener_recorte(self, frame: np.ndarray, depth: np.ndarray, log_level=0):
        try:
            if not isinstance(frame, np.ndarray):
                raise ValueError("El parámetro de entrada debe ser una imagen de OpenCV (numpy.ndarray).")
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
            # frame = self.contornos_depth(depth, hsv_image)
            # if frame is None:
            #     return None, None
            # # Mostrar la imagen HSV
            # cv2.imwrite("Imagen_HSV.jpg", frame)
            if log_level == 1:
                cv2.imshow('Frame Image', frame)
            combined_mask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, combined_mask = cv2.threshold(combined_mask, 120, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            if log_level == 1:
                cv2.imshow('Combined Mask', combined_mask)

            # # Tratamiento morfológico
            combined_mask = cv2.bitwise_not(combined_mask)
            combined_mask = cv2.bitwise_and(combined_mask, depth)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            close_img = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
            cleaned_mask = cv2.morphologyEx(close_img, cv2.MORPH_OPEN, kernel)
            # cleaned_mask = close_imgk
            # cleaned_mask = cv2.bitwise_not(cleaned_mask)
            if log_level == 1:
                cv2.imshow('Cleaned Mask', cleaned_mask)
            vertices = self.detectar_contornos(cleaned_mask)
            # print(vertices)
            if len(vertices) == 0:
                return None, cleaned_mask

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

            return numero_cuadrado, cleaned_mask

        except Exception as e:
            print(f"Ocurrió un error: {e}")
            print("Error en la línea:", traceback.format_exc())
            return None, None
