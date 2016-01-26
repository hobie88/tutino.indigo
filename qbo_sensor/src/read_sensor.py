#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32 
from decimal import Decimal

wall_distance = Point32()
# wall_distance.x=right_distance, wall_distance.y=left_distance, wall_distance.z=floor_distance 
floor_distance = 0.0
left_distance = 0.0
right_distance = 0.0

class read_sensor:

    def __init__(self):
        rospy.Subscriber('/distance_sensors_state/floor_sensor', PointCloud, self.floorSensorCallback)
        rospy.Subscriber('/distance_sensors_state/front_left_srf10', PointCloud, self.leftSensorCallback)
        rospy.Subscriber('/distance_sensors_state/front_right_srf10', PointCloud, self.rightSensorCallback)
        pub = rospy.Publisher('wall_distance', Point32, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            wall_distance.x = right_distance
            wall_distance.y = left_distance
            wall_distance.z = floor_distance
            pub.publish(wall_distance)
            rate.sleep()          

    def floorSensorCallback(self, data):
        global floor_distance
        floor_distance = data.points[0].x
        print 'FLOOR SENSOR = ' + str(data.points[0].x)
        
    def leftSensorCallback(self, data):
        global left_distance
        left_distance = data.points[0].x
        print 'LEFT SENSOR = ' + str(data.points[0].x)
        
    def rightSensorCallback(self, data):
        global right_distance 
        right_distance = data.points[0].x
        print 'RIGHT SENSOR = ' + str(data.points[0].x)   
        
if __name__ == '__main__':
    try:
        rospy.init_node('read_sensor')
        nodo = read_sensor()
        rospy.spin()
    except rospy.ROSInterruptException: 
        print 'Errore: nodo non inizializzato'   
        
