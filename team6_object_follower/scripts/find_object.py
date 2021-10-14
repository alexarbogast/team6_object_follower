#!/usr/bin/env python

import rospy
import cv2

from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from object_tracking import ContourTracker

class ObjectFinder:
	def __init__(self):
		self.contour_tracker = ContourTracker((18, 68, 58), (35, 152, 224))
		self.bridge = CvBridge()

		self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.callback, queue_size=1, buff_size=2**24)

		self.debug_img_pub = rospy.Publisher("/output/debug_image/compressed", CompressedImage, queue_size=1)
		self.target_pub = rospy.Publisher("/target_loc", Point, queue_size=1)

	def callback(self, data):
		frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		center, frame = self.contour_tracker.track(frame)

		msg = self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")
		self.debug_img_pub.publish(msg)

		target = Point()
		x, y = center if center is not None else (-1, -1)
		
		target.x, target.y, target.z = x, y, 0
		
		self.target_pub.publish(target)
		rospy.loginfo(target)

if __name__=='__main__':
	rospy.init_node('find_object', anonymous=False)
	of = ObjectFinder()
	rospy.spin()