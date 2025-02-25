import joblib
import numpy as np
import time

class KNN():
    def __init__(self):
        self.knn = joblib.load("modelo_knn(1).pkl")

    def get_prediction(self, img):
        img_flat = img.reshape(1, -1)
        prediccion = self.knn.predict(img_flat)
        return prediccion