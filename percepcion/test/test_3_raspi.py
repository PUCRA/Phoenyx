import cv2
import numpy as np
import pytesseract  
import traceback
import os

# Crear carpeta "datos" si no existe
output_folder = "datos"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Ruta del archivo de salida
output_file = os.path.join(output_folder, "detecciones.npy")

def obtener_imagen(index):
    # Intentar abrir la cámara en el índice especificado
    cap = cv2.VideoCapture(index)
    
    if not cap.isOpened():
        print(f"Error: No se pudo abrir la cámara en el índice {index}")
        return None

    # Intentar leer un fotograma
    ret, frame = cap.read()
    
    if not ret:
        print("Error: No se pudo capturar un fotograma")
        cap.release()
        return None

    cap.release()  # Liberar la cámara después de capturar el fotograma
    return frame

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

        # Tratamiento morfológico
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)

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

        if len(vertices) == 0:
            if(log_level):
                print("No se detectó un cuadrado.")
            return None
        
        puntos_origen = np.array([vertices[2][0], vertices[3][0], vertices[0][0], vertices[1][0]], np.float32)
        puntos_origen = ordenar_puntos(puntos_origen)

        mask_black = np.zeros_like(frame)
        cv2.fillPoly(mask_black, [vertices], (255, 255, 255))
        result = cv2.bitwise_and(frame, mask_black)

        ancho = 800
        alto = 800
        puntos_destino = np.float32([[0, 0], [ancho, 0], [0, alto], [ancho, alto]])
        matriz = cv2.getPerspectiveTransform(puntos_origen, puntos_destino)
        numero_cuadrado = cv2.warpPerspective(result, matriz, (ancho, alto))

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

def obtener_num(image, log_level=0):
    """Preprocesa la imagen y extrae un número usando OCR."""
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convertir a escala de grises
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)  # Binarización con inversión
        resized = cv2.resize(thresh, (100, 100))  # Redimensionar
        processed_image = cv2.morphologyEx(resized, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))  # Erosión
        
        config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'
        number = pytesseract.image_to_string(processed_image, config=config).strip()
        data = pytesseract.image_to_data(processed_image, config=config, output_type=pytesseract.Output.DICT)
        confidences = data['conf']
        average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0
        
        if not number or average_confidence < 1:
            return None, 0
        
        return int(number[0]), average_confidence
    except Exception as e:
        print(f"Ocurrió un error: {e}")
        return None, 0

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # Abre la cámara (cambiar el índice si es necesario)
    data_list = []  # Lista para almacenar los datos

    # Crear carpeta "datos" si no existe
    output_dir = "datos"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "detecciones.npy")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        recorte = obtener_recorte(frame, log_level=0)  # Llama a la función para cada frame

        if recorte is not None:
            detectado = detectar_color_bgr(recorte)
            num_detectado, average_confidence = obtener_num(recorte, log_level=0)
            num_detectado = num_detectado if num_detectado is not None else -1

            # Imprimir en la terminal
            print(f"Color detectado: {detectado}, Número detectado: {num_detectado}, Confianza: {average_confidence}")

            if detectado == "Azul":
                data_list.append([0, 1, 0, num_detectado, average_confidence])
            elif detectado == "Rojo":
                data_list.append([1, 0, 0, num_detectado, average_confidence])
            else:
                data_list.append([0, 0, 1, num_detectado, average_confidence])

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Salir si se presiona 'q'
            break

    cap.release()
    cv2.destroyAllWindows()

    # Guardar los datos en el archivo .npy
    np.save(output_file, np.array(data_list, dtype=object))
    print(f"Datos guardados en {output_file}")