#!/usr/bin/python

import Image as imgModule
import StringIO
import time
import thread
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

#Import for ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


busyFlag = True	#Initially no frames are available for streaming
currentFrame = False
timeBetweenFrame = time.time()

class CamHandler(BaseHTTPRequestHandler):
	'''
	'''
	def do_GET(self):
		'''
		wfile: Represents an handle to the Stream. 
		'''
		global busyFlag
		global currentFrame
		global timeBetweenFrame
		if self.path.endswith('.mjpg'):
			self.send_response(200)
			self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
			self.end_headers()
			while True:
				try:
					try:
						jpg = imgModule.fromarray(currentFrame)
					except:
						busyFlag = False
						continue
					tmpFile = StringIO.StringIO()
					jpg.save(tmpFile,'JPEG')
					self.wfile.write("--jpgboundary")
					self.send_header('Content-type','image/jpeg')
					self.send_header('Content-length',str(tmpFile.len))
					self.end_headers()
					jpg.save(self.wfile,'JPEG')
					busyFlag = False
				except KeyboardInterrupt:
					break
			return
		if self.path.endswith('.html'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			self.wfile.write('<html><head></head><body>')
			self.wfile.write('<img src="http://127.0.0.1:8080/cam.mjpg"/>')
			self.wfile.write('<p>' + str(timeBetweenFrame) + '</p>')
			self.wfile.write('</body></html>')
			return

def httpServerThread(address=''):
	'''
	Iniialize HTTP Server on LocalHost:8080
	'''
	print("Init HTTP Server on %s" %address)
	try:
		server = HTTPServer((address,8080),CamHandler)
		print "server started"
		server.serve_forever()
	except KeyboardInterrupt:
		server.socket.close()

def getFrame(data):
	global currentFrame
	global busyFlag
	global timeBetweenFrame
	if not busyFlag:
		busyFlag = True
		try:
			cv_bridge = CvBridge()
			currentFrame = cv_bridge.imgmsg_to_cv2(data, "bgr8")
			timeBetweenFrame = time.time() - timeBetweenFrame
		except CvBridge as e:
			print(e)
		busyFlag = False
		
def main():
	#Starting Threads
	try:
		thread.start_new_thread(httpServerThread,('',))
	except:
		print("Error occurred while executing threads")
	print("Init ROS NODE")
	rospy.init_node("video_streamer", anonymous=True, disable_signals=True)
#	rospy.Subscriber("/usb_cam/image_raw", Image, getFrame)
	rospy.Subscriber("/qbo_cam/image", Image, getFrame)
	rospy.spin()
	

if __name__ == '__main__':
	main()
