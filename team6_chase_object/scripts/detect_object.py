#!/usr/bin/env python

############################################################
# detect object:
# 
# This node recieves an image from the raspi camera.
# The image is processed to find the angle of an object in
# the camera frame 
############################################################

import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from object_tracking import ContourTracker

# https://www.raspberrypi.org/documentation/accessories/camera.html
HORIZONTAL_FOV = 62.2
IMAGE_WIDTH = 320.0
EMPTY_VAL = 101

class ObjectDetector:
	def __init__(self):
		self.contour_tracker = ContourTracker((82, 91, 105), (162, 205, 201))
		#self.contour_tracker = ContourTracker((0, 0, 0), (0, 89, 48)) # gazebo
		self.bridge = CvBridge()

		self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.callback, queue_size=1, buff_size=2**24)
		self.heading_pub = rospy.Publisher("/heading", Float32, queue_size=10)

   		self.debug_img_pub = rospy.Publisher("/output/debug_image/compressed", CompressedImage, queue_size=1)


	def callback(self, data):
		frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		center, frame = self.contour_tracker.track(frame)

		msg = self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")
		self.debug_img_pub.publish(msg)

		x, y = center if center is not None else (-1, -1)

		heading = Float32()
		if x is None or y is None:
			heading.data = EMPTY_VAL; 
		else:
			heading.data =  (IMAGE_WIDTH/2 - x)/IMAGE_WIDTH * HORIZONTAL_FOV

		self.heading_pub.publish(heading)
		rospy.loginfo(heading)


if __name__=='__main__':
	rospy.init_node('detect_object', anonymous = True)
	try:
		detector = ObjectDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass