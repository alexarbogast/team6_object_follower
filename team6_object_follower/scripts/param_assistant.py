#!/usr/bin/env python

############################################################
# Parameter Assistance:
# 
# This script is used to find the HSV thresholds for coutour
# based object tracking
############################################################

import rospy
import cv2
import imutils
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from object_tracking import ContourTracker

WINDOW_TITLE = "Parameter Assistant"

def nothing(x):
		pass

if __name__=='__main__':
	rospy.init_node('param_assist', anonymous=True)
	cv2.namedWindow(WINDOW_TITLE)

	colorLower = (0, 0, 0)
	colorUpper = (255, 255, 255)
	ct = ContourTracker(colorLower, colorUpper)
	# create tracker bars, windows, videos
	cv2.namedWindow('Parameter Assistant')
	cv2.createTrackbar('H1',WINDOW_TITLE,0,180,nothing)
	cv2.createTrackbar('S1',WINDOW_TITLE,0,255,nothing)
	cv2.createTrackbar('V1',WINDOW_TITLE,0,255,nothing)
	cv2.createTrackbar('H2',WINDOW_TITLE,0,180,nothing)
	cv2.createTrackbar('S2',WINDOW_TITLE,0,255,nothing)
	cv2.createTrackbar('V2',WINDOW_TITLE,0,255,nothing)

	bridge = CvBridge()
	
	try:
		while not rospy.is_shutdown():
			frame = rospy.wait_for_message("/raspicam_node/image/compressed",CompressedImage) 
			frame = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")

			## set parameters
			colorLower = (cv2.getTrackbarPos('H1',WINDOW_TITLE),
						  cv2.getTrackbarPos('S1',WINDOW_TITLE),
						  cv2.getTrackbarPos('V1',WINDOW_TITLE))
			#		
			colorUpper = (cv2.getTrackbarPos('H2',WINDOW_TITLE),
						  cv2.getTrackbarPos('S2',WINDOW_TITLE),
						  cv2.getTrackbarPos('V2',WINDOW_TITLE))

			ct.set_color_thresh(colorLower, colorUpper)

			mask = ct.mask_frame(frame)
			center, frame = ct.track(frame)

			mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
			numpy_horizontal_concat = np.concatenate((mask, frame), axis=1) # display images side by side
			cv2.imshow(WINDOW_TITLE, numpy_horizontal_concat)
			cv2.waitKey(1)
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()