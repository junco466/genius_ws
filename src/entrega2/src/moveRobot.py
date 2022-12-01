import sys
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from laneDetect import laneDetect
import time


class moveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self,_odom):
        
        self.odom = _odom
        self.ctrl_c = False 
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador
        self.ruta = True
    
    
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

    def moveRight(self):
        print('Entre rigth')
        self.stop()
        time.sleep(2)
        self.move(1,0,-1.5)
        self.stop()
        time.sleep(1)
        self.move(1,0.05,0)
        time.sleep(2)

    def moveLeft(self):
        print('Entre rigth')
        self.stop()
        time.sleep(2)
        self.move(1,0,1.5)
        self.stop()
        time.sleep(1)
        self.move(1,0.05,0)
        time.sleep(2)

    def moveObstaculos(self):

        if self.ruta:

            print("···············Enderezar······························")
            while self.odom.grd > 5: #not in range(0, 10):
                self.move(0, 0, -1)
            print("················Avanza······························")
            self.stop()
            self.move(0.5, 0, 1)
            self.move(3, 0.04, 0)
            self.stop()
            time.sleep(2)
            print('Angle: ',self.odom.grd)
            print("················Giro 90·····························")
            while self.odom.grd < 80: #not in range(0, 10):
                print('segundo bucle')
                self.move(0, 0, 1)
            print("··············································")

            self.stop()
            # self.move(0.2, 0, -1)
            self.move(5, 0.08, 0)
            self.stop()
            time.sleep(2)

            print('Angle: ',self.odom.grd)
            print("················Giro 180·····························")
            while self.odom.grd < 170: #not in range(0, 10):
                print('tercer bucle')
                self.move(0, 0, 1)
            print("··············································")

            self.stop()
            # self.move(0.2, 0, -1)
            self.move(4, 0.08, 0)
            self.stop()
            time.sleep(2)

            print('Angle: ',self.odom.grd)
            print("················Giro 90 2·····························")
            while self.odom.grd > 100: #not in range(0, 10):
                print('cuarto bucle')
                self.move(0, 0, -1)
            print("··············································")

            self.stop()
            # self.move(0.2, 0, 1)
            self.move(5.5, 0.08, 0)
            self.stop()
            time.sleep(2)

            print('Angle: ',self.odom.grd)
            print("················Giro 0·····························")
            while self.odom.grd > 10: #not in range(0, 10):
                print('cuarto bucle')
                self.move(0, 0, -1)
            print("··············································")

            self.stop()
            self.move(0.2, 0, 1)
            self.move(4.5, 0.08, 0)
            self.stop()
            time.sleep(2)

            print('Angle: ',self.odom.grd)
            print("················Giro 90  3·····························")
            while self.odom.grd < 90: #not in range(0, 10):
                print('quinto bucle')
                self.move(0, 0, 1)
            print("··············································")

            self.stop()
            # self.move(0.2, 0, -1)
            self.move(5.2, 0.085, 0)
            self.stop()
            time.sleep(2)

            print('Angle: ',self.odom.grd)
            print("················Giro 140·····························")
            while self.odom.grd < 160: #not in range(0, 10):
                print('quinto bucle')
                self.move(0, 0, 1)
            print("··············································")

            self.stop()
            # self.move(0.2, 0, -1)
            self.move(3, 0.08, 0.1)
            self.stop()
            time.sleep(2)

        self.ruta = False
        self.stop()
        

    def parkSt(self):

        self.stop()
        time.sleep(2)
        self.move(6,0.08,0)
        self.stop()
        time.sleep(2)
        print("················Giro REF 270·····························")
        while True:

            if self.odom.grd < 260: #not in range(0, 10):
                print('start bucle')
                self.move(0, 0.01, 1)
            elif self.odom.grd > 280: #not in range(0, 10):
                self.move(0, 0, -1)
            else:
                break

        print("··············································")
        self.stop()
        time.sleep(1)
        # self.move(0.9,0,1.7)
        # self.stop()
        # time.sleep(2)
        self.move(4,0.08,0)
        self.stop()
        time.sleep(1)


    def parkLeft(self):

        print("················Giro REF 270·····························")
        while True:

            if self.odom.grd < 260: #not in range(0, 10):
                print('primer bucle')
                self.move(0, 0, 1)
            elif self.odom.grd > 280: #not in range(0, 10):
                self.move(0, 0, -1)
            else:
                break

        print("··············································")

        self.stop()
        time.sleep(2)
        self.move(3,0.08,0)
        self.stop()
        time.sleep(2)

        print('Angle: ',self.odom.grd)
        print("················Giro REF 0º·····························")
        while self.odom.grd > 10: #not in range(0, 10):
            print('primer bucle')
            self.move(0, 0, 1)
        print("··············································")

        self.stop()
        time.sleep(1)
        self.move(2.8,-0.08,0)
        self.stop()
        time.sleep(2)

        
        self.move(2.8,0.08,0)
        self.stop()
        time.sleep(2)

        print('Angle: ',self.odom.grd)
        print("················Giro REF 90º·····························")
        while self.odom.grd < 85: #not in range(0, 10):
            print('segundo bucle')
            self.move(0, 0, 1)
        print("··············································")


        self.stop()
        time.sleep(1)
        self.move(4,0.08,0)
        self.stop()
        time.sleep(2)



    def parkRight(self):

        print("················Giro REF 270·····························")
        while True:

            if self.odom.grd < 260: #not in range(0, 10):
                print('primer bucle')
                self.move(0, 0, 1)
            elif self.odom.grd > 280: #not in range(0, 10):
                self.move(0, 0, -1)
            else:
                break

        print("··············································")

        self.stop()
        time.sleep(2)
        self.move(3,0.08,0)
        self.stop()
        time.sleep(2)

        print('Angle: ',self.odom.grd)
        print("················Giro REF 180º·····························")
        while self.odom.grd > 180: #not in range(0, 10):
            print('primer bucle')
            self.move(0, 0, -1)
        print("··············································")

        self.stop()
        time.sleep(1)
        self.move(2.8,-0.08,0)
        self.stop()
        time.sleep(2)

        
        self.move(2.8,0.08,0)
        self.stop()
        time.sleep(2)

        print('Angle: ',self.odom.grd)
        print("················Giro REF 90º·····························")
        while self.odom.grd > 90: #not in range(0, 10):
            print('segundo bucle')
            self.move(0, 0, -1)
        print("··············································")


        self.stop()
        time.sleep(1)
        self.move(4,0.08,0)
        self.stop()
        time.sleep(2)


        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(3,0.08,0)
        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(0.9,0,-1.7)
        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(1,-0.08,0)
        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(1,0.08,0)
        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(0.9,0,-1.7)
        # self.moveRobot.stop()
        # time.sleep(2)
        # self.moveRobot.move(3,0.08,0)
        # self.moveRobot.stop()
        # time.sleep(2)
        
    def publishOnceCmdVel(self, cmd): # Se crea la funcion de publicacion
        while not self.ctrl_c:
            connections = self.velPublisher.get_num_connections()
            if connections > 0:
                self.velPublisher.publish(cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()