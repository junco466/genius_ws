import math
import rospy
from nav_msgs.msg import Odometry

class odomNode(object):
    def __init__(self):
        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.ctrlStartP = False
        self.starto = 0
        self.startp = 0
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""

        # Initialise subscribers
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCallback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def eulerFromQuaternion(self, x, y, z, w):
        t0 = +2.0*(w*x+y*z)
        t1 = +1.0-2.0*(x*x+y*y)
        rollX = math.atan2(t0, t1)
        
        t2 = +2.0*(w*x-y*z)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = +1.0 if t2 < -1.0 else t2
        pitchY = math.asin(t2)
        
        t3 = +2.0*(w*z+x*y)
        t4 = +1.0 - 2.0*(y*y+z*z)
        yawZ = math.atan2(t3, t4)
        
        return rollX, pitchY, yawZ
        
    def yaw2Degrees(self, yaw):
        if yaw >= 0:
            degree = yaw*180/math.pi
        else:
            tmp = yaw*180/math.pi
            degree = 360 + tmp
        return degree
    
    def odomCallback(self, msg):
        pose = msg.pose.pose
        self.position = pose.position
        self.orientation = pose.orientation
        orientationQ = self.orientation
        (roll, pitch, yaw) = self.eulerFromQuaternion(orientationQ.x, orientationQ.y, orientationQ.z, orientationQ.w)

        self.grd = self.yaw2Degrees(yaw)
        # print('Ángulo en grados:', str(self.grd) + '°')
