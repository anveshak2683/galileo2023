#!/usr/bin/env python
import rospy
from traversal.msg import WheelRpm
from traversal.srv import *
from geometry_msgs.msg import Twist

class auto():
	
	def __init__(self):
		rospy.init_node('auto')
		self.pub_auto=rospy.Publisher('motion',WheelRpm, queue_size=10)
		rospy.Subscriber('cmd_vel', Twist, self.autoCallback)
		self.vel=0
		self.omega=0
		
	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub()
			rate.sleep()
			
	def autoCallback(self, msg):
		if msg.linear.x==-0.1:
	        	self.vel=int(msg.linear.x*10)
	        else:
                        self.vel=int(msg.linear.x/0.9*25)
	        	self.omega=int(msg.angular.z/2*75)
		
	def pub(self):
		if True:	
			rpm = WheelRpm()
			rpm.hb = False
			
			rpm.vel = 127- self.vel
			rpm.omega = 127 - self.omega

			#print(rpm) 
			#print('--------------')
			self.pub_auto.publish(rpm)
			
if __name__=='__main__':
	run=auto()
	run.spin()

