#!/usr/bin/env python

import rospy
import cv2
import signal
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
#from rospy.exceptions import ROSException
from rospy import ROSSerializationException
#from numpy import zeros
import numpy

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
        self.info_pub = rospy.Publisher("/qbo_cam/info",CameraInfo,queue_size=5)
        if not self.cap.isOpened():
            self.cap.open()
        while True: #start capturing video
            self.send_camera_info()
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
        
    def send_camera_info(self):
        info_msg = CameraInfo()
        info_msg.width = 640
        info_msg.height= 480
        # P is the projection Matrix
        info_msg.P=numpy.zeros(12)
        info_msg.P[0]=485.706413
        info_msg.P[2]=162.982482
        info_msg.P[5]=485.706413
        info_msg.P[6]=142.542517
        info_msg.P[10]=1.0000
        
        self.info_pub.publish(info_msg)
        
    def close(self, signal, frame):
        self.cap.release()
        cv2.destroyAllWindows() 
        #rospy.signal_shutdown() #not needed
        sys.exit(0)
        

def main():
    cam = QboCam()
            

if __name__ == '__main__':
    main()
