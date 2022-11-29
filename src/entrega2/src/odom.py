#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class odomNode(object):
    def __init__(self):
        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.initScanState = False  # To get the initial scan data at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        # Initialise publishers
        self.cmdVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialise subscribers
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCallback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    
    def odomCallback(self, msg):
        pose = msg.pose.pose
        self.postn = pose.position
        self.oritn = pose.orientation
        #print(postn.x, postn.y, postn.z)
        #print(oritn.x, oritn.y, oritn.z)
        self.initScanState = True
        self.updateCallback()
        
    def updateCallback(self):
        if self.initScanState is True:
            print('Position: ', self.postn.x, ' | ', self.postn.y, ' | ',  self.postn.z)
            print('Orientation: ', self.oritn.x, ' | ',  self.oritn.y, ' | ',  self.oritn.z)
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('odomNode')
    node = odomNode()
    node.main()



