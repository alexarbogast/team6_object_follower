#!/usr/bin/env python
import cv2
import imutils

class ContourTracker:
	def __init__(self, colorLower, colorUpper):
		self.set_color_thresh(colorLower, colorUpper)

	def set_color_thresh(self, colorLower, colorUpper):
		self.colorLower = colorLower
		self.colorUpper = colorUpper

	def mask_frame(self, frame):
		blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, self.colorLower, self.colorUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		return mask

	def track(self, frame, min_radius=10):
		mask = self.mask_frame(frame)

		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		
		center = None
		radius = None
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
				cv2.circle(frame, center, 4, (0, 0, 255), -1)

		return (center, frame)
