#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import math
import pylab


linStep = 0.1
angStep = 0.3


cmd_speed = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
x = 0.0
y = 0.0
z = 0.0


    
def move(t,linear,angular,movement):
    '''
    Input: Twist() handler (t = Twist())
    Input: list linear: Contains linear movements of x,y,z
    Input: list angular: Contains angular movements of x,y,z
    Input: string movement (example: ang-threshold)
    '''
    #Set Movement 
    t.linear.x=linear[0]
    t.linear.y=linear[1]
    t.linear.z=linear[2]
    t.angular.x=angular[0]
    t.angular.y=angular[1]
    t.angular.z=angular[2]
    #Controls movement on threshold
    movType = movement
    if "lin" in movType: #linear movement
        actualX = x
        if linear[0] > 0: #moving forward
            while x < actualX + linStep:
                cmd_speed.publish(t)
        else:               #moving backward
            while x > actualX - linStep:
                cmd_speed.publish(t)
    elif "ang" in movType: #angular movement
        actualZ = z
        if angular[2] > 0:  #turning counterclock
            while z < actualZ + angStep:
                cmd_speed.publish(t)
        else:               #turning cloclwise
            while z > actualZ - angStep:
                cmd_speed.publish(t)
        
        
    
    return True

    
def vocal_move_callback(cmd):
    t = Twist()
    
    
    if 'destra' in cmd.data:
        move(t,[0,0,0],[0,0,-1.0],'ang')
    
    elif 'sinistra' in cmd.data:
        move(t,[0,0,0],[0,0,1.0],'ang')
    
    elif '180' in cmd.data:
        move(t,[0,0,0],[0,0,-1.0],'ang')
        move(t,[0,0,0],[0,0,-1.0],'ang')
        
    elif 'indietro' in cmd.data:
        move(t,[-0.2,0,0],[0,0,0],'lin')

    elif 'avanti' in cmd.data:
        move(t,[0.2,0,0],[0,0,0],'lin')
    elif 'stop' in cmd.data:
        move(t,[0,0,0],[0,0,0],'lin')
    print (cmd.data)
        
#updates x,y position in the floor plane and orientation from odometry
# z is theta, positive counterclock
def odom_callback(msg):    
    global x
    global y
    global z
    x = msg.x
    y = msg.y
    z = msg.z
    
     
def vocal_movement():
    #Init the node
    rospy.init_node('vocal_move', anonymous = True)
    rospy.Subscriber('qbo_arduqbo/my_odom', Point32, odom_callback)
    rospy.Subscriber('vocal_move', String, vocal_move_callback) 
    #rospy.loginfo("subscriber subscribed")
    rospy.spin()
 
if __name__ == '__main__':
    start =  vocal_movement()
    
    
    
    
'''
    def vocal_move_callback(cmd):
    t = Twist()
    
    
    if 'destra' in cmd.data:
        move(t,[0,0,0],[0,0,-1.0])
        t.linear.x = 0.0
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z =  -1.0
        my_z = z
        rospy.loginfo("my_z: " + str(my_z))
        while z > my_z - 0.3:
            cmd_speed.publish(t)
 #           rospy.loginfo("z: %s -- x: %s y: %s" %(str(z),str(x),str(y)))
        
    elif '180' in cmd.data:
        t.linear.x = 0
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = -1.0
        my_z = z
        while z > my_z - 1.9:
            cmd_speed.publish(t)
 #       rospy.loginfo("turning 180")
                     
    elif 'indietro' in cmd.data:
        t.linear.y = 0
        t.linear.z = 0
        t.linear.x = -0.2
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        my_x = x
        while x > my_x - 0.1:
            cmd_speed.publish(t)
  #          rospy.loginfo("z: %s -- x: %s y. %s" %(str(z),str(x),str(y)))

    elif 'sinistra' in cmd.data:
        t.linear.x = 0
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 1.0
        my_z = z
        while z < my_z +0.3:
            cmd_speed.publish(t)

    elif 'avanti' in cmd.data:
        t.linear.y = 0
        t.linear.z = 0
        t.linear.x = 0.2
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        my_x = x
        while x < my_x + 0.1:
            cmd_speed.publish(t)
'''
