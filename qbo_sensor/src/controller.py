#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point32

constant_left_wrong_value = 0.0
zero_flag = True

def callback(data):

    t = Twist()
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    global wdrs  #allowed wall distance right sensor
    wdrs = 0.3
    
    global zero_flag
    global constant_left_wrong_value
    if data.y != 0 and zero_flag:
        constant_left_wrong_value = data.y
        zero_flag = False
        
    if constant_left_wrong_value != data.y:
        rospy.loginfo('OSTACOLO !!  VADO INDIETRO E GIRO A DESTRA')
        t.angular.z = -0.5
        t.linear.x = -0.15      
    elif (data.x >= wdrs or data.x == 0.0) and data.z >= 0.23 and data.z < 0.26:
        rospy.loginfo('STO ANDANDO DRITTO')
        t.linear.x = 0.15
        t.angular.x = 0.0
    elif data.x < wdrs or data.z < 0.23 or data.z >= 0.26:
        rospy.loginfo('OSTACOLO !!  VADO INDIETRO E GIRO A SINISTRA')
        t.angular.z = +0.5
        t.linear.x = -0.15
    cmd_vel.publish(t) 
    
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('wall_distance', Point32, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
