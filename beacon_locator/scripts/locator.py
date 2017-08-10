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
		self.rgbSubscriber = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgbCallback) # subscribe to images from the Kinect
		self.bridge = CvBridge() # create a bridge to convert from ROS images to OpenCV images

		self.beacons = rospy.get_param("/beacon_locator_node/beacons")
		
		# define the colours which occur as halves of the beacons
		self.colours = {
			"yellow": {
				"colour": (0,255,255),
				"lowerBound": [20, 30, 30],
				"upperBound": [40, 255, 255]
			},
			"pink": {
				"colour": (255,0,255),
				"lowerBound": [120, 30, 30],
				"upperBound": [170, 255, 255]
			},
			"blue": {
				"colour": (255,0,0),
				"lowerBound": [95, 30, 30],
				"upperBound": [120, 255, 255]
			},
			"green": {
				"colour": (0,255,0),
				"lowerBound": [40, 30, 30],
				"upperBound": [95, 255, 255]
			}
		}

	# callback function for when an rgb image is received from the Kinect sensor
	# note: all colours are in BGR format
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
		# note: only leave uncommented for testing purposes
#		output = np.zeros((480,640,3), np.uint8)

		# draw visualisation of possible beacon colour candidates
		# note: only leave uncommented for testing purposes
#		for rect in rects:
#			cv2.rectangle(output, (rect[1], rect[2]), (rect[3], rect[4]), self.colours[rect[0]]["colour"], 3)

		print "Frame Processed"
		# iterate through all pairs of detected rectangles
		for rect1, rect2 in itertools.permutations(rects, 2):
			# if the rect1 is positioned directly on top of rect2
			if rect1[6] < rect2[6] and self.doesOverlap(rect1[1],rect1[3],rect2[1],rect2[3]) and abs(rect1[4] - rect2[2]) < 10:
				# check if beacon is a valid colour combination
				for b in self.beacons:
					if b["top"] == rect1[0] and b["bottom"] == rect2[0]:
						print "Beacon %d: %s / %s" % (b["id"], rect1[0], rect2[0]) # beacon has successfully identified

		# display the visualisation
		# note: only leave uncommented for testing purposes
#		cv2.imshow("images", output)
#		cv2.waitKey(0)


	# returns a list of rectangles the are possible candidates [colour, minX, minY, maxX, maxY, centerX, centerY]
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
			x,y,w,h = cv2.boundingRect(contour) # get the bounding rectangle
			sideRatio = w/h	# calculate the ratio between sides of the contours' bounding rectangle
			rectArea = w*h # get the area of the contour
			if y > 100 and y + h < 400 and rectArea > 800 and rectArea < 14400:
				rects.append([colour, x, y, x+w, y+h, x+w/2, y+h/2]) # add rect to list of candidate colour rectangles

		return rects # return any colour rectangles
	
	# returns boolean indicating if two provided ranges overlap
	def doesOverlap(self, r1_low, r1_high, r2_low, r2_high):
		return ((r1_low >= r2_low and r1_low <= r2_high) or (r1_high >= r2_low and r1_high <= r2_high)) or ((r2_low >= r1_low and r2_low <= r1_high) or (r2_high >= r1_low and r2_high <= r1_high))

if __name__ == '__main__':
	rd = locator()
	rospy.init_node('beacon_locator_node') # create node
	rospy.spin()
	
