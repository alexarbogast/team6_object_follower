#!/usr/bin/env python
############################################################
# Parameter Assistance:
# 
# This script is used to find the HSV thresholds for coutour
# based object tracking
############################################################

import cv2
import imutils
import numpy as np

from object_tracking import ContourTracker

def nothing(x):
	pass

if __name__=="__main__":
	colorLower = (0, 0, 0)
	colorUpper = (255, 255, 255)

	ct = ContourTracker(colorLower, colorUpper)

	# create tracker bars, windows, videos
	cv2.namedWindow('Parameter Assistant')
	cv2.createTrackbar('H1','Parameter Assistant',0,180,nothing)
	cv2.createTrackbar('S1','Parameter Assistant',0,255,nothing)
	cv2.createTrackbar('V1','Parameter Assistant',0,255,nothing)

	cv2.createTrackbar('H2','Parameter Assistant',0,180,nothing)
	cv2.createTrackbar('S2','Parameter Assistant',0,255,nothing)
	cv2.createTrackbar('V2','Parameter Assistant',0,255,nothing)

	vid = cv2.VideoCapture(0)
	while True:
		success, frame = vid.read()

		## set parameters
		colorLower = (cv2.getTrackbarPos('H1','Parameter Assistant'),
					  cv2.getTrackbarPos('S1','Parameter Assistant'),
					  cv2.getTrackbarPos('V1','Parameter Assistant'))      
		
		colorUpper = (cv2.getTrackbarPos('H2','Parameter Assistant'),
					  cv2.getTrackbarPos('S2','Parameter Assistant'),
					  cv2.getTrackbarPos('V2','Parameter Assistant'))

		ct.set_color_thresh(colorLower, colorUpper)

		mask = ct.mask_frame(frame)
		center, frame = ct.track(frame)

		mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
		numpy_horizontal_concat = np.concatenate((mask, frame), axis=1) # display images side by side

		cv2.imshow('Parameter Assistant', numpy_horizontal_concat)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		
	vid.release()
	cv2.destroyAllWindows()
