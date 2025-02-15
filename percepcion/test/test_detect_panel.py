import cv2
import numpy as np
import pytesseract  
    



def detectar_panel(log_level=0):
    cap = cv2.VideoCapture(0)  # Capturar desde la webcam
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)  # Ajustar el brillo
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo capturar el frame.")
            break

        copy_image = frame.copy()

        # Convertir la imagen a escala de grises
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplicar umbral adaptativo para detectar áreas blancas
        mask_white = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                          cv2.THRESH_BINARY, 11, 2)

        # Procesamiento morfológico
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        # Cierre para eliminar pequeños agujeros
        open_mask = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('Cleaned Mask 1', open_mask)

        # Aplicar la apertura morfológica para eliminar ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        cleaned_mask = cv2.morphologyEx(open_mask, cv2.MORPH_OPEN, kernel)
        cv2.imshow('Cleaned Mask', cleaned_mask)

        # Detección de contornos
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  # Verificar si es cuadrilátero
                area = cv2.contourArea(approx)
                if area > 1200:  # Ajustar el valor según sea necesario
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h

                    if 1.3 <= aspect_ratio <= 1.6:  # Relación de aspecto entre 1.3 y 1.6
                        angles = []
                        for i in range(4):
                            pt1, pt2, pt3 = approx[i][0], approx[(i + 1) % 4][0], approx[(i + 2) % 4][0]
                            v1 = np.array([pt1[0] - pt2[0], pt1[1] - pt2[1]])
                            v2 = np.array([pt3[0] - pt2[0], pt3[1] - pt2[1]])
                            angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
                            angles.append(angle)

                        if all(80 < angle < 100 for angle in angles):  # Verificar que los ángulos sean cercanos a 90°
                            cv2.drawContours(copy_image, [approx], -1, (0, 255, 0), 3)
                            for point in approx:
                                cv2.circle(copy_image, tuple(point[0]), 5, (0, 0, 255), -1)

        cv2.imshow('Detección de Rectángulos Blancos', copy_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    cap = cv2.VideoCapture(0)  # Capturar desde la webcam
    cap.set(cv2.CAP_PROP_BRIGHTNESS, -30)  # Ajustar el brillo
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo capturar el frame.")
            break

        copy_image = frame.copy()
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rango para el color blanco en HSV (opcional si lo usas en conjunto con el umbral adaptativo)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])

        # Convertir la imagen a escala de grises
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplicar umbral adaptativo para detectar áreas blancas
        mask_white = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                          cv2.THRESH_BINARY, 11, 2)

        # Procesamiento morfológico
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        # Cierre para eliminar pequeños agujeros
        open_mask = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('Cleaned Mask 1', open_mask)

        # Aplicar la apertura morfológica para eliminar ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        cleaned_mask = cv2.morphologyEx(open_mask, cv2.MORPH_OPEN, kernel)
        cv2.imshow('Cleaned Mask', cleaned_mask)

        # Detección de contornos
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  # Verificar si es cuadrilátero
                area = cv2.contourArea(approx)
                if area > 1200:  # Ajustar el valor según sea necesario
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h

                    if 1.3 <= aspect_ratio <= 1.6:  # Relación de aspecto entre 1.3 y 1.6
                        angles = []
                        for i in range(4):
                            pt1, pt2, pt3 = approx[i][0], approx[(i + 1) % 4][0], approx[(i + 2) % 4][0]
                            v1 = np.array([pt1[0] - pt2[0], pt1[1] - pt2[1]])
                            v2 = np.array([pt3[0] - pt2[0], pt3[1] - pt2[1]])
                            angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
                            angles.append(angle)

                        if all(80 < angle < 100 for angle in angles):  # Verificar que los ángulos sean cercanos a 90°
                            cv2.drawContours(copy_image, [approx], -1, (0, 255, 0), 3)
                            for point in approx:
                                cv2.circle(copy_image, tuple(point[0]), 5, (0, 0, 255), -1)

        cv2.imshow('Detección de Rectángulos Blancos', copy_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    cap = cv2.VideoCapture(0)  # Capturar desde la webcam
    cap.set(cv2.CAP_PROP_BRIGHTNESS, -30)  # Ajustar el brillo
    if not cap.isOpened():
        print("Error: No se pudo abrir la camara.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo capturar el frame.")
            break

        copy_image = frame.copy()
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rango para el color blanco en HSV
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])

        # Crear máscara para blanco
        # mask_white = cv2.inRange(hsv_image, lower_white, upper_white)
        # cv2.imshow("mask_white", mask_white)
        mask_white = cv2.threshold(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 200, 255, cv2.THRESH_BINARY)[1]
        # Procesamiento morfológico
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        # Cierre
        open_mask = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('Cleaned Mask 1', open_mask)

        # cv2.imshow('Closed Mask', closed_mask)
        # Aplicar la apertura morfológica para eliminar ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        cleaned_mask = cv2.morphologyEx(open_mask, cv2.MORPH_OPEN, kernel)
        # if(log_level == 1):
        cv2.imshow('Cleaned Mask', cleaned_mask)

        # Detección de contornos
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  # Verificar si es cuadrilátero
                area = cv2.contourArea(approx)
                if area > 1200:  # Ajustar el valor según sea necesario
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h

                    if 1.3 <= aspect_ratio <= 1.6:  # Relación de aspecto entre 1.3 y 1.6
                        angles = []
                        for i in range(4):
                            pt1, pt2, pt3 = approx[i][0], approx[(i + 1) % 4][0], approx[(i + 2) % 4][0]
                            v1 = np.array([pt1[0] - pt2[0], pt1[1] - pt2[1]])
                            v2 = np.array([pt3[0] - pt2[0], pt3[1] - pt2[1]])
                            angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
                            angles.append(angle)

                        if all(80 < angle < 100 for angle in angles):  # Verificar que los ángulos sean cercanos a 90°
                            cv2.drawContours(copy_image, [approx], -1, (0, 255, 0), 3)
                            for point in approx:
                                cv2.circle(copy_image, tuple(point[0]), 5, (0, 0, 255), -1)

        cv2.imshow('Deteccion de Rectangulos Blancos', copy_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()


def ordenar_puntos(pts):
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

def img_clean(img, log_level = 0):
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

def obtener_num(imagen, log_level = 0):
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
        puntos_origen = ordenar_puntos(puntos_origen)
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
        # cv2.imshow("Numero_cuadrado", numero_cuadrado)
        # hsv_warped = cv2.cvtColor(numero_cuadrado, cv2.COLOR_BGR2HSV)
        # # final
        # mask = np.zeros_like(hsv_warped, dtype=np.uint8)
        # numcontours, _ = cv2.findContours(cleaned_num, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(mask, numcontours, -1, (255, 255, 255), thickness=cv2.FILLED)
        # resultado = cv2.bitwise_and(hsv_warped, mask)
        # cv2.imshow("que es mask", mask)
        # cv2.imshow("que es esto", resultado)
        # avg_color = cv2.mean(hsv_warped)[:3]
        # # Determinar el color (modificar los rangos según tus necesidades)
        # if 100 <= avg_color[0] <= 140:  # Azul
        #     detected_color = "Azul"
        # elif 0 <= avg_color[0] <= 10 or 170 <= avg_color[0] <= 180:  # Rojo
        #     detected_color = "Rojo"
        # else:
        #     detected_color = "Color desconocido"
        # if(log_level == 1):
        #     print(f"Color del cuadrado: {detected_color}")
        detected_color = detectar_color(numero_cuadrado, cleaned_num)

        final_clean = img_clean(cleaned_num)
        imagen_final = final_clean
        #return cleaned_num
        # hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    except Exception as e:
        print(f"Ocurrió un error: {e}")
        imagen_final = None
    finally:
            return copy_image, imagen_final, detected_color

def detectar_color(numero_cuadrado, cleaned_num):
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

def cuadrado(frame, color, text, confianza = 0):
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

def image_to_number(image):
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

def leer_imagenes_de_la_camara(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    
    # Exposición y ganancia
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)  # Reduce un poco más la exposición
    cap.set(cv2.CAP_PROP_GAIN, 1)  # Baja la ganancia para reducir ruido

    # Brillo y contraste
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)  # Baja un poco más el brillo
    cap.set(cv2.CAP_PROP_CONTRAST, 85)  # Ajusta el contraste para mantener detalles

    # Balance de blancos
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4400)  # Pequeña corrección al azul
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 4400)  # Ajuste fino en el rojo

    # Saturación y nitidez
    cap.set(cv2.CAP_PROP_SATURATION, 110)  # Mantener colores vivos sin sobresaturar
    cap.set(cv2.CAP_PROP_SHARPNESS, 70)  # Reducción ligera de nitidez para suavizar bordes


    # cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 1000)  # Balance de blancos
    if not cap.isOpened():
        print("Error: No se puede abrir la cámara")
        return
    
    fondo = np.zeros((640, 480, 3), dtype=np.uint8)
    imagen = np.zeros((200, 200), dtype=np.uint8)

    x_offset = 520
    y_offset = 20

    numero_detectado = None
    confianza = None
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se puede recibir la imagen (stream end?). Saliendo ...")
            break

        # Procesar el fotograma actual antes de pasar al siguiente
        # Procesar el frame completamente antes de leer el siguiente
        new_fondo, new_imagen, color = obtener_num(frame, 0)
        if new_fondo is not None:
            fondo = new_fondo
        if new_imagen is not None:
            imagen = new_imagen
            new_numero_detectado, new_confianza = image_to_number(imagen)
            if new_numero_detectado is not None:   
                numero_detectado = new_numero_detectado
                confianza = new_confianza

        superpon = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
        superpon = cv2.resize(superpon, (100, 100))
        # Superponer la imagen redimensionada en el frame
        frame[y_offset:y_offset+100, x_offset:x_offset+100] = superpon
        
        # Dibujar el cuadrado
        cuadrado(fondo, color, str(numero_detectado), str(confianza))
        
        # Mostrar la imagen procesada
        cv2.imshow('Processed Image', fondo)
        
        # Esperar hasta que se haya procesado el fotograma y se haya mostrado antes de seguir
        # Si el usuario presiona 'q', salir del bucle
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # adjust_camera_settings()
    # leer_imagenes_de_la_camara(0)
    # imagen = cv2.imread("black_9.png")
    # imagen = cv2.threshold(cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)[1]
    # numero = encontrar_num(imagen)
    # print("Número detectado: ", numero)
    detectar_panel()
    # filter_depth_webcam()
    cv2.waitKey(0)