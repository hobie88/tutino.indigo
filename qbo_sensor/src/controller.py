#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

def callback(data):
#    rospy.loginfo('HO LETTO IL MESSAGGIO: ' + str(data.data))
    t = Twist()
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    if data.data != 0.0 and data.data >= 0.4:
        print 'STO ANDANDO DRITTO'
        t.linear.x = 0.15
        t.angular.x = 0.0
    elif data.data < 0.4:
        print 'GIROGIROTONDO'
     #   t.angular.x = 1.0
     #   t.angular.y = 1.0
        t.angular.z = -0.5
        t.linear.x = 0.0
    cmd_vel.publish(t) 
    
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('wall_distance', Float32, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
