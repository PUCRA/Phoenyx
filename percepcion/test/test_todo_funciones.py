import cv2
import numpy as np
import pytesseract  
import traceback
import os

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

def obtener_recorte(frame, log_level=0):
    try:
        copy_image = frame.copy()
        detected_color = None
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  

        # Mostrar la imagen HSV
        if log_level > 0:
            cv2.imshow('HSV Image', hsv_image)
        
        # Definir el rango para el color azul
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        lower_red_1 = np.array([0, 50, 50])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 50, 50])
        upper_red_2 = np.array([180, 255, 255])

        # Crear máscaras para azul y rojo
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        combined_mask = cv2.bitwise_or(mask_blue, mask_red)

        if log_level > 0:
            cv2.imshow('Combined Mask', combined_mask)

        # Tratamiento morfológico
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)

        if log_level > 0:
            cv2.imshow('Cleaned Mask', cleaned_mask)

        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vertices = []

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                area = cv2.contourArea(approx)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / float(h)
                    if 0.8 < aspect_ratio < 1.2:
                        angles = []
                        for i in range(4):
                            pt1 = approx[i][0]
                            pt2 = approx[(i + 1) % 4][0]
                            pt3 = approx[(i + 2) % 4][0]
                            v1 = np.array([pt1[0] - pt2[0], pt1[1] - pt2[1]])
                            v2 = np.array([pt3[0] - pt2[0], pt3[1] - pt2[1]])
                            angle = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0))
                            angle_deg = np.degrees(angle)
                            angles.append(angle_deg)

                        if all(80 < angle < 100 for angle in angles):
                            cv2.drawContours(copy_image, [approx], -1, (0, 255, 0), 3)
                            for point in approx:
                                x, y = point[0]
                                cv2.circle(copy_image, (x, y), 5, (0, 0, 255), -1)
                            vertices = approx

        if log_level > 0:
            cv2.imshow('Detected Squares', copy_image)
        # print(vertices)
        if len(vertices) == 0:
            if(log_level > 0):
                print("No se detectó un cuadrado.")
            return None
        
        puntos_origen = np.array([vertices[2][0], vertices[3][0], vertices[0][0], vertices[1][0]], np.float32)
        puntos_origen = ordenar_puntos(puntos_origen)

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
    
def detectar_color_hsv(numero_cuadrado):
    """Detecta la probabilidad de ser rojo o azul basándose en el matiz (H) en HSV."""
    hsv_image = cv2.cvtColor(numero_cuadrado, cv2.COLOR_BGR2HSV)
    
    # Promedio del canal de matiz (H)
    avg_h = np.mean(hsv_image[:, :, 0])  # Hue (Matiz)
    
    if (avg_h < 10 or avg_h > 160):  # Rojos típicos en OpenCV HSV (0-10 y 160-180)
        detected = "Rojo"
    elif 100 < avg_h < 140:  # Azules típicos en OpenCV HSV (90-130)
        detected = "Azul"
    else:
        detected = "Indefinido"
    
    return detected



def detectar_color_bgr(numero_cuadrado):
    """Detecta la probabilidad de ser rojo o azul basándose en la proporción de los canales BGR."""
    bgr_image = numero_cuadrado

    # Promedio de los canales BGR
    avg_b = np.mean(bgr_image[:, :, 0])  # Azul
    avg_g = np.mean(bgr_image[:, :, 1])  # Rojo
    avg_r = np.mean(bgr_image[:, :, 2])  # Rojo

    if avg_b > avg_r and avg_b > avg_g:
        # Intercambiar los valores si el azul es mayor que el rojo
        detected = "Azul"
    elif avg_r > avg_b and avg_r > avg_g:
        detected = "Rojo"
    else:
        detected = "Indefinido"

    return detected


def obtener_num(image, log_level=1):
    """Preprocesa la imagen y extrae un número usando OCR."""
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convertir a escala de grises
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)  # Binarización con inversión
        if log_level > 0:
            cv2.imshow('Umbral', thresh)
        resized = cv2.resize(thresh, (100, 100))  # Redimensionar
        processed_image = cv2.morphologyEx(resized, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))  # Erosión
        if log_level > 0:
            cv2.imshow('Procesada', processed_image)
        
        config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'
        number = pytesseract.image_to_string(processed_image, config=config).strip()
        data_list = pytesseract.image_to_data(processed_image, config=config, output_type=pytesseract.Output.DICT)
        confidences = data_list['conf']
        average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0
        if log_level > 0:
            print(f"Número detectado: {number}, Confianza: {average_confidence}")
        
        if not number or average_confidence < 1:
            return None, 0
        
        return int(number[0]), average_confidence
    except Exception as e:
        print(f"Ocurrió un error: {e}")
        return None, 0

def decision_making(data_list):
    prob_rojo = [fila[0] for fila in data_list]  # Probabilidad de rojo
    prob_azul = [fila[1] for fila in data_list]  # Probabilidad de azul

    # Extraer los valores en la posición 3 y sus confianzas
    numeros = [fila[3] for fila in data_list if fila[3] not in [None, '']]
    confianzas = [fila[4] for fila in data_list if fila[4] not in [None, '']]

    # Calcular la media de cada probabilidad
    media_rojo = np.mean(prob_rojo)
    media_azul = np.mean(prob_azul)
    
    frecuencia_por_numero = {i: 0 for i in range(1, 10)}

    for valor, confianza in zip(numeros, confianzas):
        # Filtrar por confianza: solo contar si la confianza supera el umbral
        if confianza >= 1:
            if valor in frecuencia_por_numero:
                frecuencia_por_numero[valor] += confianza  # Ponderar por la confianza

    # Determinar el número con mayor frecuencia ponderada
    numero = max(frecuencia_por_numero, key=frecuencia_por_numero.get)
    prob_numero = frecuencia_por_numero[numero] / sum(frecuencia_por_numero.values())  # Frecuencia relativa ponderada

    # Determinar el color con mayor probabilidad
    if media_rojo > media_azul:
        color = "Rojo"
        prob_color = media_rojo
    elif media_azul > media_rojo:
        color = "Azul"
        prob_color = media_azul
    else:
        color = "Indefinido"

    medidas_usadas = len(numeros)

    return color, prob_color, numero, prob_numero, medidas_usadas

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # Abre la cámara (cambiar el índice si es necesario)
    data_list = []  # Lista para almacenar los datos
    recolectando_datos = False  # Variable para saber si estamos recogiendo datos

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        recorte = obtener_recorte(frame, log_level=0)  # Llama a la función para cada frame

        if recorte is not None:
            detectado = detectar_color_bgr(recorte)
            num_detectado, average_confidence = obtener_num(recorte, log_level=0)
            num_detectado = num_detectado if num_detectado is not None else -1

            if recolectando_datos:  # Solo agregar datos si estamos en modo de recolección
                if detectado == "Azul":
                    data_list.append([0, 1, 0, num_detectado, average_confidence])
                elif detectado == "Rojo":
                    data_list.append([1, 0, 0, num_detectado, average_confidence])
                else:
                    data_list.append([0, 0, 1, num_detectado, average_confidence])

            # cv2.imshow("Recorte", recorte)

        # Espera por una tecla y verifica si es 'w' o 'a' o 'q'
        key = cv2.waitKey(1) & 0xFF

        # Si se presiona 'w', empezar a recoger datos
        if key == ord('w'):
            recolectando_datos = True
            print("Recolección de datos iniciada...")

        # Si se presiona 'a', hacer la decisión final
        if key == ord('a'):
            if len(data_list) > 0:
                color, prob_color, numero, prob_numero, medidas_usadas = decision_making(data_list)
                print(f"Decisión final: Color: {color}, Prob: {prob_color:.2f}, Número: {numero}, Prob: {prob_numero:.2f}, Medidas usadas: {medidas_usadas}")
                # Limpiar la lista después de tomar la decisión
                data_list.clear()
            break  # Salir del bucle después de tomar la decisión

        # Si se presiona 'q', salir
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
