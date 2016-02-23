#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud

def floor_callback(msg):
    print("floor_sensor: " + str(msg.points[0].x))
    
def right_callback(msg):
    print("right_sensor: " + str(msg.points[0].x))
    
def left_callback(msg):
    print("left_sensor: " + str(msg.points[0].x))


if __name__=="__main__":
    rospy.init_node("read_sensor")
    floor_sub = rospy.Subscriber("/distance_sensors_state/floor_sensor",PointCloud,floor_callback)
    right_sub = rospy.Subscriber("/distance_sensors_state/front_right_srf10",PointCloud,right_callback)
    left_sub = rospy.Subscriber("/distance_sensors_state/front_left_srf10",PointCloud,left_callback)
    rospy.spin()
    
