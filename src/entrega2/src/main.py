#!/usr/bin/python3
#coding=utf-8

import sys
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

from followLane import Follower
from scan import Scanner
from laneDetect import laneDetect
from moveRobot import moveRobot
#from fuzzy import Fuzzy
from odomPosition import odomNode

class mainClass:

    def __init__(self):
       odom = odomNode()
       robot = moveRobot(odom)
       follow = Follower(robot)
       scan = Scanner(robot,odom,follow)
       lane = laneDetect(follow,scan,odom,robot)
    #    fuzz = Fuzzy(robot) 
    #    # Initialise subscribers
    #    self.imageSub = rospy.Subscriber("/camera/image", Image, lane.imageCallback)
    #    lane.imageSub = self.imageSub


def main(args):
    ldexp = mainClass()  # Iniciamos la clase
    rospy.init_node('mainClass', anonymous=True)  # Creamos el nodo
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
