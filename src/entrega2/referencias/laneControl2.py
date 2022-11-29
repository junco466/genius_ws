#!/usr/bin/python3
#coding=utf-8

import sys
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class laneControl(object):
    def __init__(self):
        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.cvImage = []
        self.bridge = CvBridge()

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""

        # Initialise publishers
        self.cmdVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialise subscribers
        self.imageSub = rospy.Subscriber("/camera/image_compensated", Image, self.imageCallback)
         
    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    
    def imageCallback(self, msg):
        try:
            self.cvImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = self.cvImage.shape[:2]
            cropImg = self.cvImage[:,:]
            
            hsv = cv2.cvtColor(cropImg, cv2.COLOR_BGR2HSV)
            

            
            mask = white_mask + yellow_mask
            
            # Cacular momentos de inercia
            
            m = cv2.moments(mask, False)

            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height//2, width//2
            
            res = cv2.bitwise_and(cropImg,cropImg, mask=mask)
            cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
            
            cv2.imshow("Original", self.cvImage)
            cv2.imshow("HSV", hsv)
            cv2.imshow("MASK", mask)
            cv2.imshow("RES", res)
            
            cv2.waitKey(1)
            
            errorX = cx - width/2
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -errorX/100
            self.cmdVelPub.publish(twist)
        except CvBridgeError as e:
            self.get_logger().info("Turtlebot3 image is not captured.")

def main(args):
    ir = laneControl()  # Iniciamos la clase
    rospy.init_node('imageReading', anonymous=True)  # Creamos el nodo
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
