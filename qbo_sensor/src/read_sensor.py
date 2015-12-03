#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32 

wall_distance = 0.3
left_distance = 0.0
right_distance = 0.0
count = 0

class read_sensor:

    def __init__(self):
        rospy.Subscriber('/distance_sensors_state/floor_sensor', PointCloud, self.floorSensorCallback)
        rospy.Subscriber('/distance_sensors_state/front_left_srf10', PointCloud, self.leftSensorCallback)
        rospy.Subscriber('/distance_sensors_state/front_right_srf10', PointCloud, self.rightSensorCallback)
        pub = rospy.Publisher('wall_distance', Float32, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
#            wall_distance = min(left_distance, right_distance)
            wall_distance = right_distance
            pub.publish(wall_distance)
            rate.sleep()          

    def floorSensorCallback(self, data):
        print 'FLOOR SENSOR = ' + str(data.points[0].x)
        
    def leftSensorCallback(self, data):
        global left_distance
        left_distance = data.points[0].x
        print 'LEFT SENSOR = ' + str(data.points[0].x)
        
    def rightSensorCallback(self, data):
        global right_distance 
        right_distance = data.points[0].x
        print 'RIGHT SENSOR = ' + str(data.points[0].x)
        global count
        count = count +1
        print 'COUNT = ' + str(count)
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('read_sensor')
     #   cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        nodo = read_sensor()
        rospy.spin()
    except rospy.ROSInterruptException: 
        print 'Errore: nodo non inizializzato'   
        
