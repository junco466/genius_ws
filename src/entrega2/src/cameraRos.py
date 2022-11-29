#!/usr/bin/python3
# coding=utf-8

import sys
import rospy
import cv2
from sensor_msgs.msg import Image # Funcion de opencv encargada de convertir el mensaje en una imagen de
from cv_bridge import CvBridge, CvBridgeError # opencv y viceversa

class imageReader:
    def __init__(self):
        # Creamos el topico en el que publicaremos la imagen resultado
        self.imagePub = rospy.Publisher("opencvTopic", Image, queue_size=1)
        self.bridge = CvBridge()  # Creamos un objeto para realizar la conversion de la imagen
        # Creamos el subcriptor al topico de la camara
        self.imageSub = rospy.Subscriber("/camera/image", Image, self.callback)
        self.cvImage = 0

    def callback(self, data):
        try:
            # Con CvBridge convertimos el mensaje recibido del topico
            self.cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # en una imagen de opencv
            cv2.imshow('Camera', self.cvImage)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)  # En caso de que suceda un error, el sistema imprimira una e

        # Realizamos una conversion a escala de grises de la imagen


def main(args):
    ir = imageReader()  # Iniciamos la clase
    rospy.init_node('imageReading', anonymous=True)  # Creamos el nodo
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

