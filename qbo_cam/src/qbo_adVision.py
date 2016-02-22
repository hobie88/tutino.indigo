#!/usr/bin/python

import time
import thread
import cv2
import signal
import sys	


class EdgeDetector(object):
	def __init__(self):
		signal.signal(signal.SIGINT,self.close)
		self.frames = {}	#keys->timestamps, values->frames
		self.lowTh = 35	#from pyimagesearch
		self.highTh = 125
		
	def getFrame(self,cameraID=0):
		'''
		Alpha: Open camera and get a frame.
		Goal: Get frame from ROS Topic and store it inside internal attributes
		'''
		raw_input("Push for frame")
		self.video = cv2.VideoCapture(cameraID)
		while True:
			frame = self.video.read()	#Returns a list where 0->boolean, 1->actual frame array
			if frame[0]:
				frame = frame[1]
				break
		self.frames[time.time()] = frame
		self.video.release()
		return frame
	
	def edges(self,frame,writeToFile=False):
		'''
		Input:
			matrix(array of arrays) frame: represents a frame captures from ROS or local camera
		Return:
			points where an edge has been detected
		'''
		edges = cv2.Canny(frame, self.lowTh, self.highTh)
		if (writeToFile):
			cv2.imwrite(str(time.time())+".jpg",frame)
			cv2.imwrite(str(time.time())+"_edges.jpg",edges)
		points = []
		rows = len(edges)
		columns = len(edges[0])
		startTime = time.time()
		for row in range(0,rows):
			for col in range(0,columns):
				if edges[row][col] > 0:
					points.append((row,col))
		stopTime = time.time()
		print("Edges detected in %s" %str((stopTime-startTime)))
		return points
	
	
	def find_marker(self,frame):
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray,(5,5), 0)
		edged = cv2.Canny(gray,35, 125)
		(cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		c = max(cnts, key = cv2.contourArea)
		return cv2.minAreaRect(c)
	
	def distance(self, knownW,focalL, pixelW):
		return (knownW * focalL) / pixelW
	
	def focalLength(self,pixelPerWidth, knownDistance,  knownWidth):
		return pixelPerWidth * knownDistance/knownWidth
		
	
	def close(self):
		sys.exit(0)

	
if __name__ == "__main__":
	a = EdgeDetector()
	#Calibrate focal Length
	fl = {0:[]}
	widthK = float(raw_input("Insert Obstacle Width:"))	#Known Width of the object
	frames = {0:[]}
	for i in range(0,2):
		for cameraN in frames:
			print("Frame %s of Camera %s" %(str(i),str(cameraN)))
			frame = a.getFrame(cameraN)
			distanceK = float(raw_input("Insert Obstacle Distance:"))
			marker = a.find_marker(frame)
			focalLength = (marker[1][0] * distanceK )/ widthK
			frames[cameraN] = {"distance":distanceK,
								"frame":frame}
			fl[cameraN].append(focalLength)
			print("Focal Length is: %s" %str(focalLength))
	#Let's try our data
	for cameraN in fl:  #average of focal lengths
		focalLength = {0:0}
		for focalValue in fl[cameraN]:
			focalLength[cameraN] = focalLength[cameraN] + focalValue
		frames[cameraN]["focal"] = focalLength[cameraN]/len(fl[cameraN])
		print("Camera %s has a focal length of %s" %(str(cameraN),str(frames[cameraN]["focal"])))
	#Evaluating distance with above retrieved data
	for cameraN in fl:
		for i in range(0,2):
			frame = a.getFrame()
			marker = a.find_marker(frame)
			distance = a.distance(widthK,frames[cameraN]["focal"],marker[1][0])
			print("Distance: %s" %str(distance))
			
		
