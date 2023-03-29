
from sensor_msgs.msg import LaserScan

import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import math
from moveRobot import moveRobot

class Fuzzy:
    def __init__(self,moveRobot):
        self.moveObject = moveRobot
        # self.initScan() # Define una propiedad para inicial el escaneo
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor
        self.initFuzzy()

    def initScan(self): # Inicio el escaneo
        self._scan = None # Creo una variable de control
        while self._scan is None: # Mientras la condicion se cumpla
            try:
                # print('hola hijueputas')               
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) # Se espera a la lectura de un mensaje
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        # print('hola hijueputas')  
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg):
        print('holaaaaaassssss!!!!!')
        distances = msg.ranges
        self.moveRules(distances)

        

    def moveRules(self,lidar):
        
        angulo = 90

        ranges = np.zeros(angulo)

        ranges[0:angulo//2] = lidar[angulo//2:0:-1]
        ranges[(angulo//2)::] = lidar[360:359-angulo//2:-1]
        for i in range(0,len(ranges)):
            if (ranges[i] == np.inf):
                ranges[i] = 4

        lidAvg = self.pesos(ranges)
        outLin, outAng = self.fuzzInput(lidAvg)

        for dato in ranges:
            if dato < 0.45:
                outLin = -0.25
                print('Nooooo, las guevas!!!!')
                break

        self.moveObject.move(0, outLin, outAng)
        print('linear: ' ,outLin)
        print('angular: ', outAng)
        rospy.loginfo("Moving")


    def divList(self, p, n):
        for i in range(0,len(p), n):
            yield p[i:i+n]
    
    def pesos(self,d):
        w = [-0.4, -0.6, -0.8, 1, 0.8, 0.6, 0.4]
        lista = list(self.divList(d, math.ceil(len(d)/7)))

        prom = []
        for ob in lista:
            
            prom.append(np.mean(ob))

        # print(prom)
        aux = np.matmul(w,prom)
        print('lidAvg: ', aux)
        return aux

    def fuzzInput(self,avg):

        # Se dan valores a las entradas del sistema
        self.bot_sim.input['detection'] = avg

        # Se procesan los datos y se obtiene el resultado
        self.bot_sim.compute()

        return self.bot_sim.output['linear'], self.bot_sim.output['angular']


    def initFuzzy(self):

        lid = ctrl.Antecedent(np.arange(-5,8,0.1),'detection')

        linearSpeed = ctrl.Consequent(np.arange(0, 1, 0.01), 'linear')
        angularSpeed = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'angular')

        lid['choque-derecha'] = fuzz.trimf(lid.universe, [-5, -5, -3])
        lid['corregir-derecha'] = fuzz.trimf(lid.universe, [-4, -2, 0])
        lid['choque-centro'] = fuzz.trimf(lid.universe, [-1, 0, 2])
        lid['corregir-izquierda'] = fuzz.trimf(lid.universe, [0, 2.5, 4])
        lid['ideal'] = fuzz.trimf(lid.universe, [3.5, 4, 4.5])
        lid['corregir-izquierda2'] = fuzz.trimf(lid.universe, [4, 5, 7])
        lid['choque-izquierda'] = fuzz.trimf(lid.universe, [6, 7, 8])
        
        linearSpeed['stop'] = fuzz.trimf(linearSpeed.universe, [0, 0, 0.05])
        linearSpeed['slow'] = fuzz.trimf(linearSpeed.universe, [0.04, 0.1, 0.2])
        linearSpeed['average'] = fuzz.trimf(linearSpeed.universe, [0.1, 0.5, 0.9])
        linearSpeed['fast'] = fuzz.trimf(linearSpeed.universe, [0.8, 1 ,1])

        angularNames = ['sharpRight','right','softRight','stright','softLeft','left','sharpLeft']
        angularSpeed.automf(names=angularNames)

        #REGLAS:
        rule1 = ctrl.Rule(lid['choque-derecha'],consequent= (linearSpeed['stop'],angularSpeed['sharpRight']))

        rule2 = ctrl.Rule(lid['choque-izquierda'], consequent = (linearSpeed['stop'], angularSpeed['sharpLeft']))

        rule3 = ctrl.Rule(lid['choque-centro'], consequent = linearSpeed['stop'])

        rule4 = ctrl.Rule(lid['corregir-derecha'], (linearSpeed['average'], angularSpeed['softLeft']))
        rule5 = ctrl.Rule(lid['corregir-izquierda'], (linearSpeed['average'], angularSpeed['softRight']))

        rule6 = ctrl.Rule(lid['corregir-izquierda2'], (linearSpeed['average'], angularSpeed['softRight']))

        rule7 = ctrl.Rule(lid['ideal'], (linearSpeed['average'],angularSpeed['stright']))

        # Se crea el contorlador del sistema
        self.bot_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7])
        # Se realiza una simulación dle controlador par auna cituación en específico
        self.bot_sim = ctrl.ControlSystemSimulation(self.bot_ctrl)