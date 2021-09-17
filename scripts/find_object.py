#!/usr/bin/env python
import rospy
import cv2

from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from object_tracking import ContourTracker

#ct = ContourTracker((17, 34, 104), (75, 234, 184))
#bridge = CvBridge()
#
#image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
#target_pub = rospy.Publisher("/target_loc", Point)
#
#def callback(data):
#	frame = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
#
#	mask = ct.mask_frame(frame)
#	center, frame = ct.track(frame)
#	
#	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#	numpy_horizontal_concat = np.concatenate((mask, frame), axis=1)
#
#	msg = bridge.cv2_to_compressed_imgmsg(frame, "jpeg")
#	image_pub.publish(msg)
#
#	target = Point()
#	x, y = center
#	target.x, target.y, target.z = x, y, 0
#	print(target)
	
class ObjectFinder:
	def __init__(self):
		self.contour_tracker = ContourTracker((17, 34, 104), (75, 234, 184))
		self.bridge = CvBridge()

		self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.callback)

		self.debug_img_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
		self.target_pub = rospy.Publisher("/target_loc", Point)

	def callback(self, data):
		frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		center, frame = self.contour_tracker.track(frame)

		msg = self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")
		self.debug_img_pub.publish(msg)

		target = Point()
		x, y = center
		target.x, target.y, target.z = x, y, 0
		self.target_pub.publish(target)
		print(target)

if __name__=='__main__':
	rospy.init_node('find_object', anonymous=True)
	#img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,callback)
	of = ObjectFinder()
	rospy.spin()
	cv2.destroyAllWindows()