#!/usr/bin/env python

import rospy
import cv2
import signal
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
#from rospy.exceptions import ROSException
from rospy import ROSSerializationException

frame = False

class QboCam:
    def __init__(self):
        #Handle Keyboard interrupt through signal module
        signal.signal(signal.SIGINT, self.close)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        #init ROS node
        rospy.init_node("qbo_cam",anonymous=True)
        #init Publishers
        self.image_pub = rospy.Publisher("/qbo_cam/image",Image,queue_size=10)
        if not self.cap.isOpened():
            self.cap.open()
        while True:
            self.capture()
            

    def capture(self):
        frameCheck = False
        while frameCheck==False:
            try:
                #capture frame
                #print("capturing frame")
                ret, frame = self.cap.read()
                frameCheck = True
            except Exception as e:
                print e
                frameCheck=False
            
        try:
            #print("converting msg")
            image_message = self.cv_bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            #print("height: "+ str(image_message.height) + " width: " + str(image_message.width))
        except CvBridgeError as e:
            print e
            pass
        try:
            #print("sending msg")
            self.image_pub.publish(image_message)
        except ROSSerializationException as e:
            print e
            pass
        
    def close(self, signal, frame):
        self.cap.release()
        cv2.destroyAllWindows() 
        rospy.shutdown()
        sys.exit(0)
        

def main():
    cam = QboCam()
            

if __name__ == '__main__':
    main()
