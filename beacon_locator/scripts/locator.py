#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import itertools
import numpy as np


class locator:

	def __init__(self):

		self.rgbSubscriber = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgbCallback) # subscribe to rgb images from the Kinect
		self.depthSubscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.depthCallback) # subscribe to depth images from the Kinect

		self.bridge = CvBridge() # create a bridge to convert from ROS images to OpenCV images
		self.beacons = rospy.get_param("/beacon_locator_node/beacons") # load the valid beacons from the ros param
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

		print "Frame Processed"
		# iterate through all pairs of detected rectangles
		for rect1, rect2 in itertools.permutations(rects, 2):
			# if the rect1 is positioned directly on top of rect2
			if rect1["centerY"] < rect2["centerY"] and self.doesOverlap(rect1["minX"],rect1["maxX"],rect2["minX"],rect2["maxX"]) and abs(rect1["maxY"] - rect2["minY"]) < 10:
				# check if beacon is a valid colour combination
				for b in self.beacons:
					if b["top"] == rect1["colour"] and b["bottom"] == rect2["colour"]:
						print "Beacon %d: %s / %s [depth = %d (mm) at bearing = %d (deg)]" % (b["id"], rect1["colour"], rect2["colour"], self.getDepth(min(rect1["minX"], rect2["minX"]), max(rect1["maxX"], rect2["maxX"]), rect1["minY"], rect2["maxY"]), self.getBearing(int((rect1["centerX"]+rect2["centerX"])/2))) # beacon has successfully identified

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


if __name__ == '__main__':
	rd = locator()
	rospy.init_node('beacon_locator_node') # create node
	rospy.spin()
	
