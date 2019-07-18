#!/usr/bin/env python
import rospy
from man_ctrl.msg import WheelRpm
from sensor_msgs.msg import Joy
import numpy
import math

class drive():

	def __init__(self):

		rospy.init_node("drive")

		self.pub_motor = rospy.Publisher("motion",WheelRpm,queue_size=10)

		rospy.Subscriber("/joy",Joy,self.joyCallback)

		self.straight = 0
		self.zero_turn = 0
		self.d = 1
		self.brake = False
		self.s_arr = [700,1150,1700,2500]

	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.main()
			rate.sleep()

	def main(self):

		rpm = WheelRpm()
		rpm.hb = self.brake
		
		if(abs(self.straight)>0.05 or abs(self.zero_turn)>0.05):

			rpm.vel = self.straight*s_arr[self.d-1]
			rpm.omega = self.zero_turn*s_arr[self.d-1]
			print rpm ; print '--------------\n'
			rpm.vel = int(rpm.vel>0)*4095 + (int(rpm.vel>0)*2-1)self.straight*s_arr[self.d-1]
			rpm.omega = int(rpm.omega>0)*4095 + (int(rpm.omega>0)*2-1)self.straight*s_arr[self.d-1]

		else:
			
			rpm.vel = 0
			rpm.omega = 0
			print rpm ; print '--------------\n'

		self.pub_motor.publish(rpm)

	def joyCallback(self,msg):
		
		self.straight  = msg.axes[1]
		self.zero_turn = msg.axes[2]

		if(msg.buttons[7]==1):
			self.brake = True
		else:
			self.brake = False

		if(msg.buttons[5]==1):
			if self.d < 4:
				self.d = self.d + 1
				print("Max pwm is {}".format(s_arr[self.d-1]))
		
		elif(msg.buttons[4]==1):
			if self.d >1:
				self.d = self.d - 1
				print("Max pwm is {}".format(s_arr[self.d-1]))

if __name__ == '__main__':
	run = drive()
	run.spin()