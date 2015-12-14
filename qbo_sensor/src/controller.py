#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point32

constant_left_wrong_value_enable = 1
constant_left_wrong_value = 0.0

def callback(data):
#    rospy.loginfo('HO LETTO IL MESSAGGIO: ' + str(data.data))
    t = Twist()
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    global wdrs  #allowed wall distance right sensor
    wdrs = 0.3
   	
    if constant_left_wrong_value_enable == 1: 
        constant_left_wrong_value_assignment(data.y)
    
    if data.y != constant_left_wrong_value:
        rospy.loginfo('OSTACOLO !!  VADO INDIETRO E GIRO A DESTRA')
        t.angular.z = -0.5
        t.linear.x = -0.15
    elif data.x == 0.0:
        rospy.loginfo('STO ANDANDO DRITTO: data.x = 0')
        t.linear.x = 0.1
        t.angular.x = 0.0
    elif data.x >= wdrs and data.z >= 0.23 and data.z < 0.26:
        rospy.loginfo('STO ANDANDO DRITTO')
        t.linear.x = 0.15
        t.angular.x = 0.0
    elif data.x < wdrs or data.z < 0.23 or data.z >= 0.26:
        rospy.loginfo('OSTACOLO !!  VADO INDIETRO E GIRO A SINISTRA')
     #   t.angular.x = 1.0
     #   t.angular.y = 1.0
        t.angular.z = +0.5
        t.linear.x = -0.15
    cmd_vel.publish(t) 
    
def constant_left_wrong_value_assignment(d):
    global constant_left_wrong_value
    constant_left_wrong_value = d
    global constant_left_wrong_value_enable 
    constant_left_wrong_value_enable = 0
    
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('wall_distance', Point32, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
