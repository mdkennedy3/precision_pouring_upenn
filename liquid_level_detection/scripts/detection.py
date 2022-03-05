# Kendall J. Queen
# 2/6/17
#Liquid level detection for cylinders. Detect the lines of the cylinder and use the liquid to do that


import cv2
import sys
import numpy as np
import rospy
import tf
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



class image_converter:

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback)
		self.image = None
		self.processed = False

	def callback(self, data):
		try:
			if type(data) is Image:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				self.image = cv_image
				self.processed = False
		except CvBridgeError as e:
			print (e)

		print "In Image Processor"

		cv_image = self.image
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		cv2.imshow('Raw image',gray)
		
		



		print "END OF THE SHOW"


def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous = True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

