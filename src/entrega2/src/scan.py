import sys
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class Scanner:
    def __init__(self, _robot, _odom, _follow):
        self.moveRobot = _robot
        self.odom = _odom
        self.follow = _follow
        self.scanOn = False
        self.scanParking = False
        # self.carParked = False
        # self.initScan() # Define una propiedad para inicial el escaneo
        print('init scan')
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor
    
    # def initScan(self): # Inicio el escaneo
    #     self._scan = None # Creo una variable de control
    #     while self._scan is None: # Mientras la condicion se cumpla
    #         try:                
    #             self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) # Se espera a la lectura de un mensaje
    #         except:
    #             rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
    #     rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg):

        # print('ScanCallback')

        if self.scanOn:

            lidar = msg.ranges
            angulo = 120

            ranges = np.zeros(angulo)

            ranges[0:angulo//2] = lidar[angulo//2:0:-1]
            # ranges[(angulo//2)::] = lidar[360:359-angulo//2:-1]
            for i in range(0,len(ranges)):
                if (ranges[i] == np.inf) or (ranges[i] > 0.8):
                    ranges[i] = 0

            # print('ranges: ', ranges)
            
            # lidAvg = self.pesos(ranges)
            # print('lidAvg: ', lidAvg)

            suma = sum(ranges)
            # print('suma: ',suma)

            if suma > 13:
                self.follow.cons = True
        
        if self.scanParking:

            lidar = msg.ranges
            angulo = 200

            ranges = np.zeros(angulo)

            ranges[0:angulo//2] = lidar[angulo//2:0:-1]
            ranges[(angulo//2)::] = lidar[360:359-angulo//2:-1]
            for i in range(0,len(ranges)):
                if (ranges[i] < 0.35):
                    # self.carParked = True
                    if i < 110:
                        self.follow.leftParked = True
                        self.scanParking = False
                    else:
                        self.follow.rightParked = True
                        self.scanParking = False


            # print('ranges: ', ranges)
            
            # lidAvg = self.pesos(ranges)
            # print('lidAvg: ', lidAvg)

            # suma = sum(ranges)
            # print('suma: ',suma)

            # if suma > 13:
            #     self.follow.cons = True




    # def pesos(self,d):

    #     w = [-0.4, -0.6, -0.8, 1, 0.8, 0.6, 0.4]
    #     lista = list(self.divList(d, math.ceil(len(d)/7)))

    #     prom = []
    #     for ob in lista:
            
    #         prom.append(np.mean(ob))

    #     # print(prom)
    #     aux = np.matmul(w,prom)
    #     # print('lidAvg: ', aux)
    #     return aux

    # def divList(self, p, n):
    #         for i in range(0,len(p), n):
    #             yield p[i:i+n]