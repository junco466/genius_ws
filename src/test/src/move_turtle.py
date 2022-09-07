#!/usr/bin/python3
# coding=utf-8
#Simplificacion del movimiento del robot la prueba
import rospy
import time
from geometry_msgs.msg import Twist

rospy.init_node('move_robot')
rate = rospy.Rate(10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
i = 0
while not rospy.is_shutdown():
    msg = Twist()
    time.sleep(1.0)
    
    msg.linear.x = 1.0 # Movimiento en x
    pub.publish(msg) # Publicación del mensaje
    time.sleep(0.5) # Tiempo de espera para realizar el movimiento
    
    msg.linear.x = 0.0 # Movimiento en x
    pub.publish(msg) # Publicación del mensaje
    time.sleep(2.0) # Tiempo de espera para realizar el movimiento
    
    msg.angular.z = 1 # Movimiento en z
    pub.publish(msg) # Publicación del mensaje
    time.sleep(2.0)
    
    msg.angular.z = 0.0 # Movimiento en z
    pub.publish(msg) # Publicación del mensaje
    time.sleep(2.0)
    
    
    i += 1 # Contador de operaciones
    if i>=4: # El robot realiza cuatro veces el procedimiento
        break # Despues de esto termina

