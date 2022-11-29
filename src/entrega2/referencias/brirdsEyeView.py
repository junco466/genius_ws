import cv2
import numpy as np
import matplotlib.pyplot as plt

IMAGE_H = 223
IMAGE_W = 1280

src = np.float32([[0, IMAGE_H], # Inferior izquierda
                  [1207, IMAGE_H], # Inferior derecha
                  [0, 0],  # superior izquierda
                  [IMAGE_W, 0]]) # superior derecha
dst = np.float32([[569, IMAGE_H], # Inferior izquierda
                  [711, IMAGE_H], # Inferior derecha
                  [0, 0], # superior izquierda
                  [IMAGE_W, 0]]) # superior derecha

M = cv2.getPerspectiveTransform(src, dst) # Matrix de transformacion
Minv = cv2.getPerspectiveTransform(dst, src) # Matrix de transformacion inversa

img = cv2.imread('testImg.jpg') # Imagen de prueba
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Seleccion de una seccion de la imagen
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Transformacion de la imagen
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Grafico de la imagen transformada
plt.show()