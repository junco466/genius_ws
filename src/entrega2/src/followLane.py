import time
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class Follower:
  def __init__(self, robot):
    self.moveRobot = robot
    self.Tadvice = False
    self.Radvice = False
    self.Ladvice = False
    self.Padvice = False
    self.giroPrevioD = False
    self.giroPrevioI = False
    self.slowly = False
    self.cons  = False
    self.bar = False
    self.victory = False
        
  def seguirLinea(self,tendency):

    hsv = cv2.cvtColor(tendency, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = tendency.shape
    search_top = 3*h//4
    search_bot = 3*h//4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(tendency, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      # print('error: ' , err)


      if self.Tadvice:
        print('entre TT triangulo')
        self.moveRobot.move(0, 0.04, -float(err) / 100)
        self.Tadvice = False

      elif self.Radvice:
        print('voltie derecha')
        time.sleep(0.5)
        # self.giroDerecha()
        self.moveRobot.moveRight()
        self.Radvice = False
        self.slowly = True
        self.giroPrevioD = True

      elif self.Ladvice:
        time.sleep(0.5)
        print('voltie izquierda')
        self.moveRobot.moveLeft()
        # self.giroIzquierda()
        self.Ladvice = False
        self.slowly = True
        self.giroPrevioI = True

      elif self.Padvice:
        if self.giroPrevioD:
          self.moveRobot.stop()
          time.sleep(1)
          self.moveRobot.move(5,0.05,0)
          self.moveRobot.stop()
          time.sleep(1)
          self.moveRobot.move(0.8,0,-1.7)
          self.moveRobot.stop()
          time.sleep(1)
          # self.moveRobot.moveRight()
          
        elif self.giroPrevioI:
          self.moveRobot.stop()
          time.sleep(1)
          self.moveRobot.move(5,0.05,0)
          self.moveRobot.stop()
          time.sleep(1)
          self.moveRobot.move(0.8,0,1.7)
          self.moveRobot.stop()
          time.sleep(1)

          #self.moveRobot.move(4,0.05,0.25)
          # self.moveRobot.moveLeft()

        self.Padvice = False
        self.slowly = False
          
      elif self.cons:
        # print('pareeeeeeee')
        self.moveRobot.stop()
        self.moveRobot.moveObstaculos()
        self.cons = False

      elif self.bar:
        print('pare por barra')
        self.moveRobot.stop()

      elif self.victory:
        self.moveRobot.stop()

      else:

        if self.slowly:
          print('voy en slowly')
          self.moveRobot.move(0, 0.04, (-float(err)) / 100)
        else:
          print('sigo linea normal')
          self.moveRobot.move(0, 0.06, (-float(err)) / 100)
        # pass

    #   self.twist.linear.x = 0.05
    #   self.twist.angular.z = -float(err) / 100
    #   self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    cv2.imshow("mask",mask)
    cv2.imshow("output", tendency)
    cv2.waitKey(3)

  # def giroDerecha(self):
  #   self.moveRobot.stop()
  #   time.sleep(2)
  #   self.moveRobot.move(1,0,-1.8)
  #   self.moveRobot.stop()
  #   time.sleep(1)
  #   self.moveRobot.move(1,0.04,0)
  #   time.sleep(2)

  # def giroIzquierda(self):
  #   self.moveRobot.stop()
  #   time.sleep(2)
  #   self.moveRobot.move(1,0,1.8)
  #   self.moveRobot.stop()
  #   time.sleep(1)
  #   self.moveRobot.move(1,0.04,0)
  #   time.sleep(2)