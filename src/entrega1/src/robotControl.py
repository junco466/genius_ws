#!/usr/bin/python3

import sys
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import math
#import jupyter

import pdb

class moveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self):
        self.ctrl_c = False 
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador

    def publishOnceCmdVel(self, cmd): # Se crea la funcion de publicacion
        while not self.ctrl_c:
            connections = self.velPublisher.get_num_connections()
            if connections > 0:
                self.velPublisher.publish(cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def stop(self): # Se crea una funcion de detenido
        #rospy.loginfo("shutdown time! Stop the robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publishOnceCmdVel(cmd)

    def move(self, movingTime, linearSpeed, angularSpeed): # Se crea una funcion de movimiento
        cmd = Twist()
        cmd.linear.x = linearSpeed
        cmd.angular.z = angularSpeed
        
        self.publishOnceCmdVel(cmd)
        time.sleep(movingTime)

class scanner:
    def __init__(self):
        self.moveObject = moveRobot()
        self.initScan() # Define una propiedad para inicial el escaneo
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor
        self.initFuzzy()

    def initScan(self): # Inicio el escaneo
        self._scan = None # Creo una variable de control
        while self._scan is None: # Mientras la condicion se cumpla
            try:                
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) # Se espera a la lectura de un mensaje
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg):
        distances = msg.ranges

        # sen = [x for x in distances]
        # sen = np.array(sen)
        # print(np.nanmax(sen[sen != np.inf]))
        self.moveRules(distances)

        

    def moveRules(self,lidar):
        
        angulo = 90

        ranges = np.zeros(angulo)

        ranges[0:angulo//2] = lidar[angulo//2:0:-1]
        ranges[(angulo//2)::] = lidar[360:359-angulo//2:-1]
        for i in range(0,len(ranges)):
            if (ranges[i] == np.inf):
                ranges[i] = 4

        # print(ranges)
        # pdb.set_trace()

        self.pesos(ranges)
        # tetha = [x for x in range(0,angulo)] 

        # plt.plot(tetha,ranges)
        # plt.show()
        # print(len(ranges[1]))

        # if lidar[0] > 3.4:
        #     self.moveObject.move(0.0,0.0,-2.0)
        #     rospy.loginfo("Moving")
        # elif lidar[0] < 1.0:
        #     self.moveObject.stop()
        #     rospy.loginfo("Stop")

        # aux = 0
        # for i in range(1,6):
            
        #     ranges[aux:(angulo//5*i)]
        #     aux = angulo//5*i

    def divList(self, p, n):
        for i in range(0,len(p), n):
            yield p[i:i+n]
    
    def pesos(self,d):
        w = [-0.4, -0.6, -0.8, 1, 0.8, 0.6, 0.4]
        lista = list(self.divList(d, math.ceil(len(d)/7)))
        # print(lista)

        prom = []
        for ob in lista:
            prom.append(np.mean(ob))

        print(prom)
        aux = np.matmul(w,prom)
        # suma = sum(abs(aux))
        print(1/aux)

    def initFuzzy(self):
        lidarDist = ctrl.Antecedent(np.arange(0.3, 3.8, 0.1), 'distance')
        direction = ctrl. Antecedent(np.arange(0, 10, 1), 'direction')
        lid = ctrl.Antecedent(np.arange(-1,5,0.1),'detection')

        linearSpeed = ctrl.Consequent(np.arange(0, 1, 0.01), 'linear')
        angularSpeed = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'angular')

        lid['asd'] = fuzz.trimf(lid.universe, [0, 0, 13])
        lid['medium'] = fuzz.trimf(lid.universe, [0, 13, 25])
        lid['high'] = fuzz.trimf(lid.universe, [13, 25, 25])
        lidNames = ['center','left','center','right','farRight']
        lid.automf(names=lidNames)

        lidarNames = ['closest','close','medium','far','further']
        lidarDist.automf(names=lidarNames)

        directionNames = ['farLeft','left','center','right','farRight']
        direction.automf(names=directionNames)

        linearNames = ['slowest','slower','slow','average','fast','faster','fastest']
        linearSpeed.automf(names=linearNames)

        angularNames = ['sharpRight','right','softRight','stright','softLeft','left','sharpLeft']
        angularSpeed.automf(names=angularNames)

        # fuzzyLidar.view()
        # plt.show()

        # direction.view()
        # plt.show()

        # linearSpeed.view()
        # plt.show()

        # angularSpeed.view()
        # plt.show()

        #REGLAS:
        rule1 = ctrl.Rule(lidarDist['closest'], linearSpeed['slowest'])
        rule2 = ctrl.Rule(lidarDist['further'], linearSpeed['fastest'])
        rule3 = ctrl.Rule(direction['center'] & lidarDist['close'], linearSpeed['slowest'])


    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()
