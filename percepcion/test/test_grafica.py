import numpy as np
import matplotlib.pyplot as plt
import os

# Cargar el archivo .npy
file_path = "datos/detecciones.npy"

if not os.path.exists(file_path):
    print(f"No se encontró el archivo {file_path}. Asegúrate de haber guardado datos.")
    exit()

# Cargar los datos
data = np.load(file_path, allow_pickle=True)

# Extraer las probabilidades
prob_rojo = [fila[0] for fila in data]  # Probabilidad de rojo
prob_azul = [fila[1] for fila in data]  # Probabilidad de azul
prob_indefinido = [fila[2] for fila in data]  # Probabilidad indefinida

# Calcular la media de cada probabilidad
media_rojo = np.mean(prob_rojo)
media_azul = np.mean(prob_azul)
media_indefinido = np.mean(prob_indefinido)

# Número de muestras extraídas
num_muestras = len(data)

# Crear la lista de colores y sus medias de probabilidad
colores = ['Rojo', 'Azul', 'Indefinido']
probabilidades = [media_rojo, media_azul, media_indefinido]

# Crear la gráfica de barras
plt.figure(figsize=(8, 6))
bars = plt.bar(colores, probabilidades, color=['red', 'blue', 'gray'])

# Agregar las probabilidades encima de las barras
for bar, prob in zip(bars, probabilidades):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
             f'{prob:.2f}', ha='center', va='bottom', fontsize=12)

# Determinar el color con mayor probabilidad
if media_rojo > media_azul:
    color_mas_probable = "Rojo"
    probabilidad_mas_probable = media_rojo
else:
    color_mas_probable = "Azul"
    probabilidad_mas_probable = media_azul

# Mostrar el color con mayor probabilidad en la gráfica
plt.text(0.7, 0.95, f"Color más probable: {color_mas_probable} ({probabilidad_mas_probable:.2f})",
         transform=plt.gca().transAxes, fontsize=12, verticalalignment='top', horizontalalignment='center')

# Etiquetas y título
plt.ylabel("Probabilidad Media")
plt.title("Probabilidad Media de Colores Detectados (Rojo vs Azul)")
plt.ylim(0, 1)  # Limitar el rango de la gráfica entre 0 y 1

# Añadir el número de muestras extraídas en la parte superior izquierda
plt.text(0.05, 0.95, f"Muestras extraídas: {num_muestras}", transform=plt.gca().transAxes,
         fontsize=12, verticalalignment='top')

# Mostrar la gráfica
plt.show()
