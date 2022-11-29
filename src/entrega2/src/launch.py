#!/usr/bin/python3
#coding=utf-8
from os import system
from time import sleep
# system("gnome-terminal --tab -- bash -c 'roscore'")
# sleep(4)
system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch'")
sleep(6)

# system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch'")
# sleep(4)
# system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration'")
# sleep(4) 
# system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_autorace_detect detect_lane.launch'")
# sleep(4)
# system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch'")
# sleep(4)
system("gnome-terminal --tab -- bash -c 'roslaunch turtlebot3_gazebo turtlebot3_autorace_mission.launch '")
sleep(5)