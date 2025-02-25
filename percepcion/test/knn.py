import sklearn
from image2recorte import image2recorte
import cv2
import joblib
import numpy as np
import time


if __name__ == "__main__":
    recortador = image2recorte()
    # Cargar el modelo k-NN desde el archivo
    knn = joblib.load("modelo_knn(1).pkl")
    # Cargar imagen
    # cap = cv2.VideoCapture(0)
    # while True:
    #     ret, frame = cap.read()
    #     ini_time = time.time()
    #     cv2.imshow('Original', frame)
    #     recorte = recortador.obtener_recorte(frame)
    #     recorte_time = time.time()
    #     if recorte is not None:
    #         cv2.imshow('Recorte', recorte)
    #         # _, recorte_bin = cv2.threshold(recorte, 0, 180, cv2.THRESH_BINARY)
    #         imagen_gris = cv2.cvtColor(recorte, cv2.COLOR_BGR2GRAY)

    #         # Aplicar umbral binario
    #         _, recorte_bin = cv2.threshold(imagen_gris, 128, 255, cv2.THRESH_BINARY)
    #         recorte_bin = cv2.resize(recorte_bin, (28, 28))
    #         img_flat = recorte_bin.reshape(1, -1)
    #         predicciones = knn.predict(img_flat)
    #         predict_time = time.time()
    #         print("Tiempo recorte: "+str(recorte_time-ini_time)+" Tiempo predict: "+str(predict_time-recorte_time))
    #         print(predicciones)
    #         cv2.imshow('Recorte binarizado', recorte_bin)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyAllWindows()
    # nombre = "black_"
    # ext = ".png"
    # pruebas = [9, 8, 1, 5, 2, 3, 7, 4, 6]
    # ini_time = time.time()
    # for i in range(0, 9):
    #     recorte = cv2.imread(nombre+str(pruebas[i])+ext)
    #     imagen_gris = cv2.cvtColor(recorte, cv2.COLOR_BGR2GRAY)
    #     # Aplicar umbral binario
    #     _, recorte_bin = cv2.threshold(imagen_gris, 128, 255, cv2.THRESH_BINARY)
    #     recorte_bin = cv2.resize(recorte_bin, (28, 28))
    #     img_flat = recorte_bin.reshape(1, -1)  # Convierte en un array 2D con 1 fila y muchas columnas
    #     predicciones = knn.predict(img_flat)
    #     print("Numero "+str(i)+" predecido como: "+str(predicciones))
    # print("9 imagenes en: "+str(time.time()-ini_time))