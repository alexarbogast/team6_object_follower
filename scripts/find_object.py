#!/usr/bin/evn python
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
	of = ObjectFinder()
	rospy.spin()
	cv2.destroyAllWindows()