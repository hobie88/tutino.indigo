#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('vocal_move', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        command = raw_input("Command: ")
        if (command == 'destra') or (command == 'sinistra') or (command =='180') or (command == 'avanti') or (command =='indietro'):
            rospy.loginfo('COMMAND: ' + command)
            pub.publish(command)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass