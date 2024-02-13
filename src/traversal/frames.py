#!/usr/bin/env python
 
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter:
 
 	def __init__(self):
 		self.image_pub=rospy.Publisher("image_pub_1",Image,queue_size=10)
 		self.bridge=CvBridge()
 		self.cap=cv2.VideoCapture(0)
 		
 	def convert(self):
 		ret,image=self.cap.read()
 		if ret:
 			image_msg=self.bridge.cv2_to_imgmsg(image)
 			self.image_pub.publish(image_msg)
 		
def main():
	rospy.init_node('image_pub_1',anonymous=True)
	rate=rospy.Rate(10)
	ic=image_converter()
	while not rospy.is_shutdown():
 		ic.convert()
 		rate.sleep()
 	
if __name__=='__main__':
 	main()
