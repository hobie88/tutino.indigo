#!/usr/bin/env python
# coding: utf-8

import rospy
import os
from std_msgs.msg import String
import sys


def main():
    rospy.init_node('keyboard_input',anonymous=True)
    pub=rospy.Publisher('listened',String,queue_size=20)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        name = raw_input("Insert name: ")
        pub.publish(name)
        rate.sleep()


if __name__ == '__main__':
    main()
