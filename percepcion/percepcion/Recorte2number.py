import cv2
import numpy as np
import pytesseract  

class Recorte2number():
    def __init__(self):
        pass

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

        if avg_b > avg_r and avg_b > avg_g:
            # Intercambiar los valores si el azul es mayor que el rojo
            detected = "Azul"
        elif avg_r > avg_b and avg_r > avg_g:
            detected = "Rojo"
        else:
            detected = "Indefinido"

        return detected
    
    def obtener_colorYnum(self, image):
        color = self.detectar_color_bgr(image)
        numero = self.obtener_num(image)
        return numero, color
