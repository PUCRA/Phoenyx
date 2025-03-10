import sklearn
import joblib
import cv2
import numpy as np
import pytesseract  

class Recorte2number():
    def __init__(self):
        self.knn = joblib.load("/home/pucra/Phoenyx/src/percepcion/percepcion/modelo_knn(2).pkl")
        self.knn2 = joblib.load("/home/pucra/Phoenyx/src/percepcion/percepcion/modelo_knn(1).pkl")
        self.prev_num = 0
        # self.knn = joblib.load("/home/icehot03/Phoenyx/src/percepcion/percepcion/modelo_knn(1).pkl")


    def obtener_num(self, image, log_level=0):
        """Preprocesa la imagen y extrae un número usando OCR."""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convertir a escala de grises
            _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)  # Binarización con inversión
            resized = cv2.resize(thresh, (100, 100))  # Redimensionar
            processed_image = cv2.morphologyEx(resized, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))  # Erosión

            config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'
            number = pytesseract.image_to_string(processed_image, config=config).strip()
            data_list = pytesseract.image_to_data(processed_image, config=config, output_type=pytesseract.Output.DICT)
            confidences = data_list['conf']
            average_confidence = sum(confidences) / len(confidences) if len(confidences) > 0 else 0

            if not number or average_confidence < 1:
                return None, 0

            return int(number[0]), average_confidence
        except Exception as e:
            print(f"Ocurrió un error: {e}")
            return None, 0
        
    def detectar_color_bgr(self, numero_cuadrado):
        """Detecta la probabilidad de ser rojo o azul basándose en la proporción de los canales BGR."""
        bgr_image = numero_cuadrado

        # Promedio de los canales BGR
        avg_b = np.mean(bgr_image[:, :, 0])  # Azul
        avg_g = np.mean(bgr_image[:, :, 1])  # Rojo
        avg_r = np.mean(bgr_image[:, :, 2])  # Rojo
        max_value = max(avg_b, avg_g, avg_r)
        print(f"avg_b: {avg_b}, avg_g: {avg_g}, avg_r: {avg_r}")
        if max_value == avg_b and avg_g < 80 and avg_r < 80:
            detected = "Azul"
        # elif avg_r > avg_b and (avg_r > avg_g and avg_b < 70 and avg_g < 70):
        elif max_value == avg_r and avg_g < 80 and avg_b < 80:
            detected = "Rojo"
        else:
            detected = "Indefinido"
        return detected
    
    def obtener_knn_num(self, img_thresh):
        img_flat = img_thresh.reshape(1, -1)
        white_pixels = np.count_nonzero(img_thresh > 100)
        print(white_pixels)
        if white_pixels < 20 or white_pixels > 300:
            return 0
        if self.prev_num == 2 or self.prev_num == 3:
            img_resize = cv2.resize(img_thresh, (28, 28))
            img_resize = img_resize.reshape(1, -1)
            prediccion = self.knn2.predict(img_resize)[0]
        else:
            
            prediccion = self.knn.predict(img_flat)[0]
        self.prev_num = prediccion
        return prediccion

    def obtener_colorYnum(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, img_thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_thresh = cv2.resize(img_thresh, (50, 50))
        image = cv2.resize(image, (50, 50))
        color = self.detectar_color_bgr(image)
        # numero = self.obtener_num(image)
        numero = self.obtener_knn_num(img_thresh)
        return numero, color
