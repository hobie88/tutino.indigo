#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard data.data: %s", data.data)
    
    rospy.loginfo("Solo il messaggio data: %s", data)


def conversation():
    rospy.init_node('conversation',anonymous=True)
    rospy.Subscriber('listened', String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    conversation()
