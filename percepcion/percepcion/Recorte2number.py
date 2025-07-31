import sklearn
import joblib
import cv2
import numpy as np
import pytesseract  

class Recorte2number():
    def __init__(self):
        self.knn = joblib.load("/home/pucra/Phoenyx/src/percepcion/percepcion/final_knn.pkl")
        # self.knn = joblib.load("/home/pucra/Phoenyx/src/percepcion/percepcion/modelo_knn(2).pkl")
        # self.knn2 = joblib.load("/home/pucra/Phoenyx/src/percepcion/percepcion/modelo_knn(1).pkl")
        self.prev_num = 0




    def obtener_num(self, image, log_level=0):
        """Preprocesa la imagen y extrae un número usando OCR."""
        try:
            # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convertir a escala de grises
            # _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)  # Binarización con inversión
            # resized = cv2.resize(thresh, (100, 100))  # Redimensionar
            # processed_image = cv2.morphologyEx(image, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))  # Erosión
            image = cv2.bitwise_not(image)
            config = '--psm 10 -c tessedit_char_whitelist=12346789' # Pol: He quitado el 5 y el 0 de la whitelist
            number = pytesseract.image_to_string(image, config=config).strip()
            data_list = pytesseract.image_to_data(image, config=config, output_type=pytesseract.Output.DICT)
            confidences = data_list['conf']
            average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0

            if not number or average_confidence < 1:
                return None #, 0

            return int(number[0]) #, average_confidence
        except Exception as e:
            print(f"Ocurrió un error: {e}")
            return None #, 0
        
    def detectar_color_bgr(self, numero_cuadrado):
        """Detecta la probabilidad de ser rojo o azul basándose en la proporción de los canales BGR."""
        bgr_image = numero_cuadrado

        # Promedio de los canales BGR
        avg_b = np.mean(bgr_image[:, :, 0])  # Azul
        avg_g = np.mean(bgr_image[:, :, 1])  # Rojo
        avg_r = np.mean(bgr_image[:, :, 2])  # Rojo
        max_value = max(avg_b, avg_g, avg_r)
        print(f"avg_b: {avg_b}, avg_g: {avg_g}, avg_r: {avg_r}")
        if max_value == avg_b and avg_g < 130 and avg_r < 130:
            detected = "Azul"
        # elif avg_r > avg_b and (avg_r > avg_g and avg_b < 70 and avg_g < 70):
        elif max_value == avg_r and avg_g < 130 and avg_b < 130:
            detected = "Rojo"
        else:
            detected = "Indefinido"
        return detected
    
    def obtener_knn_num(self, img_thresh):
        img_flat = img_thresh.reshape(1, -1)
        white_pixels = np.count_nonzero(img_thresh > 100)
        print(f"white_pixels: {white_pixels}")
        # print(white_pixels)
        # if white_pixels < 20 or white_pixels > 1200:
        #     return 0
        prediccion = self.knn.predict(img_flat)[0]
        # if prediccion != 5:
        #     prediccion = self.obtener_num(img_thresh)
        # self.prev_num = prediccion
        return prediccion

    def obtener_colorYnum(self, image):
        color = self.detectar_color_bgr(image)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ####################################################################
        #               MODIFICAR UMBRAL EN CASO DE AJUSTE
        #               
        #######################################################################
        umbral = 150
        if color == "Rojo":
            umbral = 130#230
        elif color == "Azul":
            umbral = 140#230
        else:
            umbral = 120
        _, img_thresh = cv2.threshold(gray, umbral, 255, cv2.THRESH_BINARY) #+cv2.THRESH_OTSU
        frame_thickness = 10  # Ajusta este valor según el grosor del marco que quieras
        cv2.imwrite('imagen_umbralizada.png', img_thresh)
        # Poner en negro los píxeles del marco exterior (bordes)
        img_thresh[-frame_thickness:, :] = 0  # Borde inferior
        img_thresh[:, :frame_thickness] = 0  # Borde izquierdo
        img_thresh[:frame_thickness, :] = 0  # Borde superior
        img_thresh[:, -frame_thickness:] = 0  # Borde derecho

        img_thresh = self.bounding_box(img_thresh)
        contornos, jerarquia = cv2.findContours(img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # Crear una imagen vacía para dibujar los contornos suavizados
        img_smooth = np.zeros_like(img_thresh)  # Inicializa la imagen con ceros (negro)
        contornos_vacios = []
        # Suavizar y dibujar contornos
        for i, contorno in enumerate(contornos):
            # Comprobar si el contorno es exterior (nivel 0 en la jerarquía)
            epsilon = 0.01 * cv2.arcLength(contorno, True)  # Ajustar epsilon para más suavizado
            contorno_suavizado = cv2.approxPolyDP(contorno, epsilon, True)
            if jerarquia[0][i][3] == -1:  # Contorno exterior
                
                
                # Dibujar el contorno suavizado en la nueva imagen binaria, relleno de blanco
                new_image = cv2.drawContours(img_smooth, [contorno_suavizado], -1, (255), thickness=cv2.FILLED)
            else:
                contornos_vacios.append(contorno_suavizado)
            # elif cv2.contourArea(contorno) < 10:
            #     new_image = cv2.drawContours(new_image, [contorno_suavizado], -1, (0), thickness=cv2.FILLED)
        
        for contorno in contornos_vacios:
            new_image = cv2.drawContours(new_image, [contorno], -1, (0), thickness=cv2.FILLED)
        img_thresh = cv2.resize(new_image, (50, 50))
        cv2.imwrite('imagen_suavizada.png', img_thresh)
        
        # image = cv2.resize(image, (50, 50))
        # numero = self.obtener_num(image)
        numero = self.obtener_knn_num(img_thresh)
        return numero, color, img_thresh

    def ordenar_puntos_bounding_box(self, puntos):
        suma = puntos.sum(axis=1)
        dif = np.diff(puntos, axis=1)

        ordenados = np.zeros((4, 2), dtype="float32")
        ordenados[0] = puntos[np.argmin(suma)]  # Top-left
        ordenados[2] = puntos[np.argmax(suma)]  # Bottom-right
        ordenados[1] = puntos[np.argmin(dif)]   # Top-right
        ordenados[3] = puntos[np.argmax(dif)]   # Bottom-left
        return ordenados

    def bounding_box(self, binaria):
        # Contornos
        contornos, _ = cv2.findContours(binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            contorno_mas_grande = max(contornos, key=cv2.contourArea)
            rect = cv2.minAreaRect(contorno_mas_grande)
            centro, size, angulo = rect
            ancho_rect, alto_rect = size
            # Aseguramos que alto_rect sea el mayor (definimos orientación vertical)
            if ancho_rect > alto_rect:
                alto_rect, ancho_rect = ancho_rect, alto_rect
                angulo += 90
            # Nuevo tamaño cuadrado centrado
            lado_cuadrado = alto_rect
            # Crear el nuevo rectángulo cuadrado centrado, misma orientación
            rect_cuadrado = (centro, (lado_cuadrado, lado_cuadrado), angulo)
            box_cuadrado = cv2.boxPoints(rect_cuadrado)
            box_cuadrado = np.intp(box_cuadrado)
            # Warping
            destino = np.array([
                [0, 0],
                [lado_cuadrado - 1, 0],
                [lado_cuadrado - 1, lado_cuadrado - 1],
                [0, lado_cuadrado - 1]
            ], dtype="float32")
            puntos_origen = self.ordenar_puntos_bounding_box(np.float32(box_cuadrado))
            M = cv2.getPerspectiveTransform(puntos_origen, destino)
            imagen_enderezada = cv2.warpPerspective(binaria, M, (int(lado_cuadrado), int(lado_cuadrado)))

            # Redimensionar a tamaño uniforme
            tamaño_final = 50
            imagen_final = cv2.resize(imagen_enderezada, (tamaño_final, tamaño_final))
            imagen_final = self.suavizar_numero(imagen_final)
            return imagen_final
        return -1

    def suavizar_numero(self, img_thresh):
        contornos, jerarquia = cv2.findContours(img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # Crear una imagen vacía para dibujar los contornos suavizados
        img_smooth = np.zeros_like(img_thresh)  # Inicializa la imagen con ceros (negro)
        contornos_vacios = []
        # Suavizar y dibujar contornos
        for i, contorno in enumerate(contornos):
            # Comprobar si el contorno es exterior (nivel 0 en la jerarquía)
            epsilon = 0.01 * cv2.arcLength(contorno, True)  # Ajustar epsilon para más suavizado
            contorno_suavizado = cv2.approxPolyDP(contorno, epsilon, True)
            if jerarquia[0][i][3] == -1:  # Contorno exterior


                # Dibujar el contorno suavizado en la nueva imagen binaria, relleno de blanco
                new_image = cv2.drawContours(img_smooth, [contorno_suavizado], -1, (255), thickness=cv2.FILLED)
            else:
                contornos_vacios.append(contorno_suavizado)
            # elif cv2.contourArea(contorno) < 10:
            #     new_image = cv2.drawContours(new_image, [contorno_suavizado], -1, (0), thickness=cv2.FILLED)

        for contorno in contornos_vacios:
            new_image = cv2.drawContours(new_image, [contorno], -1, (0), thickness=cv2.FILLED)
        return new_image
