import sklearn
from image2recorte import image2recorte
import cv2
import joblib
import numpy as np
import time
import sys

def decision_making(numeros, colores):
        prob_rojo = 0  # Probabilidad de rojo
        prob_azul = 0 # Probabilidad de azul
        for color in colores:
            if color == "Azul":
                prob_azul += 1
            elif color =="Rojo":
                prob_rojo += 1
        prob_rojo /= float(len(colores))
        prob_azul /= float(len(colores))

        frecuencia_por_numero = {i: 0 for i in range(0, 10)}

        for valor in numeros:
            # Filtrar por confianza: solo contar si la confianza supera el umbral
            if valor in frecuencia_por_numero:
                frecuencia_por_numero[valor] += 1  # Ponderar por la confianza

        # Determinar el número con mayor frecuencia ponderada
        numero = max(frecuencia_por_numero, key=frecuencia_por_numero.get)
        # prob_numero = frecuencia_por_numero[numero] / sum(frecuencia_por_numero.values())  # Frecuencia relativa ponderada

        # Determinar el color con mayor probabilidad
        if prob_rojo > prob_azul:
            color = "Rojo"
            prob_color = prob_rojo
        elif prob_azul > prob_rojo:
            color = "Azul"
            prob_color = prob_azul
        else:
            color = "Distractorio"

        return numero, color

def detectar_color_bgr(numero_cuadrado):
        """Detecta la probabilidad de ser rojo o azul basándose en la proporción de los canales BGR."""
        bgr_image = numero_cuadrado

        # Promedio de los canales BGR
        avg_b = np.mean(bgr_image[:, :, 0])  # Azul
        avg_g = np.mean(bgr_image[:, :, 1])  # Rojo
        avg_r = np.mean(bgr_image[:, :, 2])  # Rojo
        if avg_b > avg_r and (avg_b > avg_g and avg_g < 70 and avg_r < 70):
            detected = "Azul"
        elif avg_r > avg_b and (avg_r > avg_g and avg_b < 70 and avg_g < 70):
            detected = "Rojo"
        else:
            detected = "Indefinido"
        return detected


if __name__ == "__main__":
    recortador = image2recorte()
    # Cargar el modelo k-NN desde el archivo
    knn = joblib.load("modelo_knn(1).pkl")
    print("Modelo cargado")
    # Cargar imagen
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
    # Desactivar balance de blancos automático
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)

    # Ajustar manualmente (prueba con diferentes valores)
    # cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)
    # Intenta aumentar el brillo (prueba valores entre 0 y 1 o más según la cámara)
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.6)
    numeros = []
    colores = []
    longitud = 100
    recorte = None
    print("Presiona intro para detectar un numero:")
    input()
    print("Escaneando numero...")
    ini_time = time.time()
    while True:
        
        ret, frame = cap.read()
        if frame is not None:
            recorte = recortador.obtener_recorte(frame)
        else:
            print("Frame no encontrado")
        if recorte is not None:
            imagen_gris = cv2.cvtColor(recorte, cv2.COLOR_BGR2GRAY)
            # Aplicar umbral binario
            _, recorte_bin = cv2.threshold(imagen_gris, 128, 255, cv2.THRESH_BINARY)
            recorte_bin = cv2.resize(recorte_bin, (28, 28))
            img_flat = recorte_bin.reshape(1, -1)
            # Prediccion numero
            predicciones = knn.predict(img_flat)
            white_dots = np.count_nonzero(recorte_bin > 100)
            # print(white_dots)
            if white_dots < 20:
                numeros.append(0)
            else:
                numeros.append(int(predicciones[0]))
            # prediccion color
            colores.append(detectar_color_bgr(recorte))
            # print(predicciones)
            # Actualizar barra de progreso
            progreso = len(numeros) / longitud
            porcentaje = int(progreso * 100)
            barra = "#" * (porcentaje // 2)  # Barra de 50 caracteres máx.
            espacio = " " * (50 - len(barra))  # Relleno para mantener tamaño fijo
            sys.stdout.write(f"\r[{barra}{espacio}] {porcentaje}%")
            sys.stdout.flush()
        
        if len(colores) >= longitud or len(numeros) >= longitud or time.time() - ini_time > 60:
            print("")
            numero, color = decision_making(numeros, colores)
            if numero == 0:
                print("Numero: No hay numero Color: "+color)
            else:
                print("Numero: "+str(numero)+" Color: "+color)
            colores = []
            numeros = []
            ini_time = time.time()
            print("----------------------------------------")
            print("Presiona intro para detectar un numero:")
            input()
            print("")
            print("Escaneando numero...")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()