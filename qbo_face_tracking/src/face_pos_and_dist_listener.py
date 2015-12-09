#!/usr/bin/env python
# coding: utf-8

import rospy
from qbo_face_msgs.msg import FacePosAndDist
from sensor_msgs.msg import ImageRaw


def listen():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/qbo_face_tracking/face_pos_and_dist',FacePosAndDist,callback)
    rospy.Subscriber('/stereo/left/image_raw', ImageRaw, camera_callback)
    rospy.spin()


def camera_callback():
    print '&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&'
    print 'Camera info received'
    print '&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&'
    
def callback(msg):
    print '******************************************'
    print 'U= ' + msg.u + ' V= ' + msg.v + ' distance to head= ' + msg.distance_to_head
    print 'face detected: ' + msg.face_detected + ' type of tracking: '+ msg.type_of_tracking
    print '******************************************'
    

if __name__=='__main__':
    listen()
