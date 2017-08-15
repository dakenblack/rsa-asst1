#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import cv2
from cv_bridge import CvBridge, CvBridgeError

import itertools
import numpy as np
import math


class locator:

	def __init__(self):

		self.rgbSubscriber = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgbCallback) # subscribe to rgb images from the Kinect
		self.depthSubscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.depthCallback) # subscribe to depth images from the Kinect
		self.beaconPublisher = rospy.Publisher("/comp3431/beacons", Marker, queue_size=10) # publish beacons for RVIZ on the required topic
		self.foundStatus = rospy.Publisher("/beacon_locator_node/status", String, queue_size=10)

		self.bridge = CvBridge() # create a bridge to convert from ROS images to OpenCV images
		self.depthImage = None # stores the depth image set by the Kinect
		
		# define the colours which occur as halves of the beacons
		# colour is used when rendering during testing (in BGR format).
		# lowerBound and upperBound are used when detecting beacons (in HSV format).
		self.colours = {
			"yellow": { "colour": (  0, 255, 255), "lowerBound": [ 20, 30, 30], "upperBound": [ 40, 255, 255] },
			"pink":   { "colour": (255,   0, 255), "lowerBound": [120, 30, 30], "upperBound": [170, 255, 255] },
			"blue":   { "colour": (255,   0,   0), "lowerBound": [ 95, 30, 30], "upperBound": [120, 255, 255] },
			"green":  { "colour": (  0, 255,   0), "lowerBound": [ 40, 30, 30], "upperBound": [ 95, 255, 255] }
		}
		
		self.beacons = []
		self.numBeaconsFound = 0
		self.totalBeaconsToFind = 3
		tmpBeacons = rospy.get_param("/beacon_locator_node/beacons") # load the valid beacons from the ros param
		for b in tmpBeacons:
			self.beacons.append({ "id": b["id"], "top": b["top"], "bottom": b["bottom"], "found": 0 })
		

		self.robotHeading = 0 # heading of the robot in radians
		self.robotX = 0 # x coord of the robot in global frame
		self.robotY = 0 # y coord of the robot in global frame


	# callback function for when an rgb image is received from the Kinect sensor
	def rgbCallback(self, img):
		try:
			cvImg = self.bridge.imgmsg_to_cv2(img, "bgr8") # load image into OpenCV
		except CvBridgeError as e:
			print(e)

		cvImg = cv2.cvtColor(cvImg,cv2.COLOR_BGR2HSV)

		# detect all possible coloured rectangles within the from
		rects  = self.getColourRects(cvImg, "yellow")
		rects += self.getColourRects(cvImg, "pink")
		rects += self.getColourRects(cvImg, "blue")
		rects += self.getColourRects(cvImg, "green")

		# create a black image for visualising detected rectangles
		# caution: only leave uncommented for testing purposes
#		output = np.zeros((480,640,3), np.uint8)			

		#print "Frame Processed"
		# iterate through all pairs of detected rectangles
		for rect1, rect2 in itertools.permutations(rects, 2):
			# if the rect1 is positioned directly on top of rect2
			if rect1["centerY"] < rect2["centerY"] and self.doesOverlap(rect1["minX"],rect1["maxX"],rect2["minX"],rect2["maxX"]) and abs(rect1["maxY"] - rect2["minY"]) < 10:
				# check if beacon is a valid colour combination
				for b in self.beacons:
					if b["top"] == rect1["colour"] and b["bottom"] == rect2["colour"]:
						distance = self.getDepth(min(rect1["minX"], rect2["minX"]), max(rect1["maxX"], rect2["maxX"]), rect1["minY"], rect2["maxY"]) / 1000
						bearing = self.getBearing(int((rect1["centerX"]+rect2["centerX"])/2))
						position = self.convertReferenceFrame(distance, bearing)

						if b["found"] == 0 and distance > 0.5 and self.numBeaconsFound != self.totalBeaconsToFind: # beacon for the first time been found			
							print "Beacon %d: %s / %s [depth = %.2f(m) at bearing = %d (deg) (x: %.2f, y: %.2f)]" % (b["id"], rect1["colour"], rect2["colour"], distance, bearing, position["x"], position["y"]) # beacon has successfully identified
							self.publishBeacon(b["id"], rect1["colour"], rect2["colour"], position["x"], position["y"])
							b["found"] = 1

						# draw visualisation of possible beacon colour candidates
						# caution: only leave uncommented for testing purposes
#						cv2.rectangle(output, (rect1["minX"], rect1["minY"]), (rect1["maxX"], rect1["maxY"]), self.colours[rect1["colour"]]["colour"], 3)
#						cv2.rectangle(output, (rect2["minX"], rect2["minY"]), (rect2["maxX"], rect2["maxY"]), self.colours[rect2["colour"]]["colour"], 3)
						
		# display the visualisation
		# caution: only leave uncommented for testing purposes
#		cv2.imshow("images", output)
#		cv2.waitKey(0)



	# returns a list of rectangles the are possible candidates
	def getColourRects(self, img, colour):

		lower = np.array(self.colours[colour]["lowerBound"], dtype = "uint8") # create a np array for the lower colour bound
		upper = np.array(self.colours[colour]["upperBound"], dtype = "uint8") # create a np array for the upper colour bound

		mask = cv2.inRange(img, lower, upper) # create a mask of pixels within the colour bounds

		element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) # create a 3x3 pixel element used for filtering operations
		mask = cv2.erode(mask, element, iterations=2) # reduce noise in image (eg isolated pixels within the colour bounds)
		mask = cv2.dilate(mask, element, iterations=2)	# fill in any small holes in larger blocks of pixels within the colour bounds
		mask = cv2.erode(mask, element) # reduce noise in image (eg single pixels within the colour bounds)

		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # find contours in the mask

		rects = [] # list to store found contours matching basic conditions

		for contour in contours:
			x, y, w, h = cv2.boundingRect(contour) # get the bounding rectangle
			rectArea = w * h # get the area of the contour
			if y > 100 and y + h < 400 and rectArea > 800 and rectArea < 14400:
				rects.append({ "colour": colour, "minX": x, "minY": y, "maxX":x+w, "maxY":y+h, "centerX":x+w/2, "centerY":y+h/2 }) # add rect to list of candidate colour rectangles

		return rects # return any colour rectangles


	# returns boolean indicating if two provided ranges overlap
	def doesOverlap(self, r1_min, r1_max, r2_min, r2_max):
		return ((r1_min >= r2_min and r1_min <= r2_max) or (r1_max >= r2_min and r1_max <= r2_max)) or ((r2_min >= r1_min and r2_min <= r1_max) or (r2_max >= r1_min and r2_max <= r1_max))


	# callback function for when a depth image is received from the Kinect sensor
	def depthCallback(self, img):
		img.encoding="mono16"
		try:
			self.depthImage = self.bridge.imgmsg_to_cv2(img, "passthrough") # load image into OpenCV
		except CvBridgeError as e:
			print(e)


	# gets the average depth of the specified region (in mm)
	def getDepth(self, minX, maxX, minY, maxY):
		if self.depthImage is not None:
			depths=[]
			for y in range(minY+int((maxY-minY)/4), maxY-int((maxY-minY)/4)): # for the middle half of y values
				for x in range(minX+int((maxX-minX)/4), maxX-int((maxX-minX)/4)): # for the middle half of x values
					if self.depthImage[y, x] != 0: # if there is a depth value for the specified pixel
						depths.append(self.depthImage[y, x]) # store the depth value
			if len(depths) > 0:
				return int(sum(depths) / len(depths)) # get the average depth value
		return 0
	

	# gets the bearing of the beacon relative to the robots reference frame (in degrees)
	def getBearing(self, x):
		return int(((x-320)/640)*62)


	# converts the reference frame of the beacon from local to global
	def convertReferenceFrame(self, distance, bearing):
		rad = (self.robotHeading + bearing) * math.pi/180
		return { "x": self.robotX + distance*math.sin(rad), "y": self.robotY + distance*math.cos(rad) }


	# publish a detected beacon
	def publishBeacon(self, beaconId, top, bottom, x, y):
		marker = Marker()
		marker.id = beaconId
		marker.header.frame_id = "/map"
		marker.type = 11
		marker.action = 0
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0

		# compute the required colours
		baseCol = ColorRGBA(0.0, 0.0, 0.0, 1.0)
		topCol = ColorRGBA(self.colours[top]["colour"][2]/255, self.colours[top]["colour"][1]/255, self.colours[top]["colour"][0]/255, 1.0)
		bottomCol = ColorRGBA(self.colours[bottom]["colour"][2]/255, self.colours[bottom]["colour"][1]/255, self.colours[bottom]["colour"][0]/255, 1.0)

		# compute the bottom and top center points
		bottomCenter = Point(0, 0, 0)
		topCenter = Point (0, 0, 0.4)

		incAngle = math.pi/8
		curAngle = 0
		lastPoints = [None, None, None, None, None, None, None]

		for i in range(0, 17):
			# produce a list of points representing the radial cross section of the marker
			points = [None, None, None, None, None, None, None]
			xComp = math.sin(curAngle)
			yComp = math.cos(curAngle)
			points[0] = Point(xComp*0.05, yComp*0.05, 0)
			points[1] = Point(xComp*0.05, yComp*0.05, 0.01)
			points[2] = Point(xComp*0.01, yComp*0.01, 0.01)
			points[3] = Point(xComp*0.01, yComp*0.01, 0.2)
			points[4] = Point(xComp*0.05, yComp*0.05, 0.2)
			points[5] = Point(xComp*0.05, yComp*0.05, 0.3)
			points[6] = Point(xComp*0.05, yComp*0.05, 0.4)

			if lastPoints[0] is not None: # if we can construct a fae
				self.markerAddTri(marker, baseCol, bottomCenter, lastPoints[0], points[0]) # bottom face
				self.markerAddRect(marker, baseCol, lastPoints[0], points[0], points[1], lastPoints[1])
				self.markerAddRect(marker, baseCol, lastPoints[1], points[1], points[2], lastPoints[2])
				self.markerAddRect(marker, baseCol, lastPoints[2], points[2], points[3], lastPoints[3])
				self.markerAddRect(marker, bottomCol, lastPoints[3], points[3], points[4], lastPoints[4])
				self.markerAddRect(marker, bottomCol, lastPoints[4], points[4], points[5], lastPoints[5])
				self.markerAddRect(marker, topCol, lastPoints[5], points[5], points[6], lastPoints[6])
				self.markerAddTri(marker, topCol, topCenter, lastPoints[6], points[6]) # top face

			curAngle += incAngle # increment the angle for each iteration
			lastPoints = points # store the calculated cross section for the next iteration


		self.numBeaconsFound += 1
		if self.numBeaconsFound == self.totalBeaconsToFind:
			print "Beacon discovery is Complete"
			self.foundStatus.publish("complete")

		self.beaconPublisher.publish(marker)


	# adds a triangle to a specified marker
	def markerAddTri(self, marker, col, p0, p1, p2):
		marker.points.append(p0)
		marker.points.append(p1)
		marker.points.append(p2)
		marker.colors.append(col)
		marker.colors.append(col)
		marker.colors.append(col)


	# adds a rectangle to a specified marker
	def markerAddRect(self, marker, col, p0, p1, p2, p3):
		marker.points.append(p0)
		marker.points.append(p1)
		marker.points.append(p2)
		marker.points.append(p0)
		marker.points.append(p2)
		marker.points.append(p3)
		marker.colors.append(col)
		marker.colors.append(col)
		marker.colors.append(col)
		marker.colors.append(col)
		marker.colors.append(col)
		marker.colors.append(col)
		


if __name__ == '__main__':
	rd = locator()
	rospy.init_node('beacon_locator_node') # create node
	rospy.spin()
	
