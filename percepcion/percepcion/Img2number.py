import cv2
import numpy as np
import pytesseract


class image2number():
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

    def img_clean(self, img, log_level = 0):
        try:
            # Encontrar los contornos en la imagen binaria
            contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Si se encontraron contornos
            if contours:
                # Encontrar el contorno con la mayor área
                largest_contour = max(contours, key=cv2.contourArea)

                # Crear una máscara negra del mismo tamaño que la imagen original
                mask = np.zeros_like(img)

                # Dibujar solo el contorno más grande en la máscara blanca
                cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

                # Aplicar la máscara a la imagen original
                result = cv2.bitwise_and(img, img, mask=mask)

                # Opcional: recortar la región de interés (ROI) de la imagen original
                x, y, w, h = cv2.boundingRect(largest_contour)
                cropped = img[y:y+h, x:x+w]

                # Guardar o mostrar la imagen resultante
                if log_level == 1:
                    cv2.imshow('Largest Region', result)
                    cv2.imshow('Cropped Region', cropped)
                return result
        except Exception as e:
            print(f"Ocurrió un error: {e}")
            return None
        finally:
            pass

    def obtener_num(self, imagen, log_level = 0):
        try:
            copy_image = imagen.copy()
            detected_color = None
            hsv_image = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)  
            if(log_level == 1):
                cv2.imshow('Loaded Image', imagen)
            # Definir el rango para el color azul
            lower_blue = np.array([100, 150, 50])  # Límite inferior del azul (H, S, V)
            upper_blue = np.array([140, 255, 255])  # Límite superior del azul (H, S, V)

            lower_red_1 = np.array([0, 50, 50])  # Límite inferior del rojo (primer rango)
            upper_red_1 = np.array([10, 255, 255])  # Límite superior del rojo (primer rango)

            lower_red_2 = np.array([170, 50, 50])  # Límite inferior del rojo (segundo rango)
            upper_red_2 = np.array([180, 255, 255])  # Límite superior del rojo (segundo rango)

            # Crear máscaras para azul y rojo
            mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
            mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
            mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)

            # Combinar las máscaras del rojo
            mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

            # Combinar máscaras azul y rojo
            combined_mask = cv2.bitwise_or(mask_blue, mask_red)

            # Aplicar la máscara combinada a la imagen original
            result = cv2.bitwise_and(imagen, imagen, mask=combined_mask)

            # cv2.imshow('Combined Mask', combined_mask)



            # -------------------------------
            #      TRATADO MORFOLOGICO
            # -------------------------------

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            # Cierre
            closed_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            # cv2.imshow('Closed Mask', closed_mask)
            # Aplicar la apertura morfológica para eliminar ruido
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)
            # if(log_level == 1):
            cv2.imshow('Cleaned Mask', cleaned_mask)

            # -------------------------------
            #      DETECCIÓN DE CONTORNOS
            # -------------------------------


            contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                # Aproximar el contorno
                epsilon = 0.02 * cv2.arcLength(contour, True)  # Ajusta el factor de aproximación
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Verificar si tiene 4 lados (cuadrilátero)
                if len(approx) == 4:
                    # Calcular el área del contorno para descartar falsos positivos
                    area = cv2.contourArea(approx)
                    if area > 500:  # Ajusta el valor mínimo de área según el tamaño esperado
                        # Calcular el rectángulo delimitador (bounding box)
                        x, y, w, h = cv2.boundingRect(approx)
                        aspect_ratio = float(w) / h  # Relación de aspecto entre el ancho y el alto

                        # Verificar si la relación de aspecto es cercana a 1 (cuadrado)
                        if 0.8 < aspect_ratio < 1.2:  # Ajusta este rango según la tolerancia que desees
                            # Calcular los ángulos para asegurarse de que sean cerca de 90 grados
                            angles = []
                            for i in range(4):
                                pt1 = approx[i][0]
                                pt2 = approx[(i + 1) % 4][0]
                                pt3 = approx[(i + 2) % 4][0]

                                # Calcular los vectores entre puntos consecutivos
                                v1 = np.array([pt1[0] - pt2[0], pt1[1] - pt2[1]])
                                v2 = np.array([pt3[0] - pt2[0], pt3[1] - pt2[1]])

                                # Calcular el ángulo entre los vectores
                                angle = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0))
                                angle_deg = np.degrees(angle)
                                angles.append(angle_deg)

                            # Verificar si todos los ángulos son cercanos a 90 grados
                            if all(80 < angle < 100 for angle in angles):  # Ajusta este rango de ángulos
                                # Dibujar el contorno detectado
                                cv2.drawContours(copy_image, [approx], -1, (0, 255, 0), 3)

                                # Opcional: Dibujar los vértices
                                for point in approx:
                                    x, y = point[0]
                                    cv2.circle(copy_image, (x, y), 5, (0, 0, 255), -1)
                                vertices = approx
                                #print("Detectado un cuadrado")
            if(log_level == 1):
                cv2.imshow('Detected Squares', imagen)
            if(log_level == 1):
                print('Contornos detectados:', len(vertices))
                print("Vercices: ", vertices)
                print("vertice 1", vertices[0][0])
            puntos_origen = np.array([vertices[2][0], vertices[3][0], vertices[0][0], vertices[1][0]], np.float32)
            puntos_origen = self.ordenar_puntos(puntos_origen)
            # Mascara
            mask_black = np.zeros_like(imagen)
            # Dibujar el contorno del cuadrado en la máscara
            cv2.fillPoly(mask_black, [vertices], (255, 255, 255))  # Blanco dentro del cuadrado
            result = cv2.bitwise_and(imagen, mask_black)
            if(log_level == 1):
                cv2.imshow('Masked Region', result)

            # -------------------------------
            #      CORRECCIÓN DE PERSPECTIVA
            # -------------------------------
            # Coordenadas del rectángulo deseado en la vista frontal (el tamaño del resultado)
            ancho = 800  # Ancho del rectángulo corregido
            alto = 800  # Alto del rectángulo corregido
            puntos_destino = np.float32([[0, 0], [ancho, 0], [0, alto], [ancho, alto]])
            # Calcular la transformación de perspectiva
            matriz = cv2.getPerspectiveTransform(puntos_origen, puntos_destino)

            # Aplicar la transformación
            numero_cuadrado = cv2.warpPerspective(result, matriz, (ancho, alto))
            # cv2.imshow('Original Image', imagen)

            # if(log_level == 1):
            cv2.imshow('Corrected Image', numero_cuadrado)


            # -------------------------------
            #      SEGMENTACIÓN DE NÚMEROS
            #      EN EL CUADRADO CORREGIDO
            # -------------------------------
            gs_image = cv2.cvtColor(numero_cuadrado, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            eq_im = clahe.apply(gs_image)
            _, final = cv2.threshold(eq_im, 127, 255, cv2.THRESH_BINARY)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            cleaned_num = cv2.morphologyEx(final, cv2.MORPH_OPEN, kernel)
            # if(log_level == 1):
            cv2.imshow('Cleaned Num', cleaned_num)


            # ---- Identificar el color del cuadrado ----
            detected_color = self.detectar_color(numero_cuadrado, cleaned_num)

            final_clean = self.img_clean(cleaned_num)
            imagen_final = final_clean

            number = self.image_to_number(final_clean)
            #return cleaned_num
            # hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            print(f"Ocurrió un error: {e}")
            imagen_final = None
            number = None
            detected_color = None
        finally:
                return number, detected_color

    def detectar_color(self, numero_cuadrado, cleaned_num):
        # Convertir la imagen del cuadrado a HSV
        hsv_warped = cv2.cvtColor(numero_cuadrado, cv2.COLOR_BGR2HSV)
        # Crear una máscara para el cuadrado usando un rango de color específico (e.g., un rango para el borde)
        # Este rango es solo un ejemplo para detectar un color específico, ajusta según tus necesidades
        lower_color = np.array([100, 100, 100])  # Mínimo valor para detectar el color
        upper_color = np.array([140, 255, 255])  # Máximo valor para detectar el color
        mask_square = cv2.inRange(hsv_warped, lower_color, upper_color)
        # Mostrar la máscara del cuadrado
        avg_color = cv2.mean(hsv_warped, mask=mask_square)[:3]

        # Determinar el color basado en el matiz (H)
        if 100 <= avg_color[0] <= 140:  # Azul
            detected_color = "Azul"
        elif 0 <= avg_color[0] <= 10 or 170 <= avg_color[0] <= 180:  # Rojo
            detected_color = "Rojo"
        else:
            detected_color = "Color desconocido"

        # print(f"Color del cuadrado: {detected_color}")
        return detected_color

    def cuadrado(self, frame, color, text, confianza = 0):
        square_size = 100 
        # Definir el color del cuadrado: Azul (BGR)
        if color == "Azul":
            square_color = (255, 0, 0)
        elif color == "Rojo":
            square_color = (0, 0, 255)
        else:
            square_color = (0, 0, 0)
        # Dibujar el cuadrado en la esquina superior izquierda
        cv2.rectangle(frame, (10, 10), (10 + square_size, 10 + square_size), square_color, -1)

        # Definir el texto y la fuente
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Colocar el texto en el centro del cuadrado
        cv2.putText(frame, text, (30, 70), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, confianza, (30, 90), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

        return frame

    def image_to_number(self, image):
        try:
            new_image = cv2.resize(image, (100, 100))
            new_image = cv2.morphologyEx(new_image, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))
            cv2.imshow("Number Image", new_image)
            number =  pytesseract.image_to_string(new_image, config='--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789')
            config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'
            data = pytesseract.image_to_data(new_image, config=config, output_type=pytesseract.Output.DICT)
            confidences = data['conf']
            average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0
            number = number[0]
            if number == "":
                return None, 0
            if average_confidence < 1:
                return None, 0
            return int(number), average_confidence
        except Exception as e:
            print(f"Ocurrió un error: {e}")
            return None, 0

