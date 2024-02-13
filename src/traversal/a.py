#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from traversal.msg import WheelRpm
import numpy as np
import cv2
import cv2.aruco as aruco


class auto():

	def __init__(self):
		
		#self.pipeline = rs.pipeline()
		#config = rs.config()
		#pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
		#pipeline_profile = config.resolve(pipeline_wrapper)
		#device = pipeline_profile.get_device()
		#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
		#self.pipeline.start(config)
		self.cap = cv2.VideoCapture(0)
		rospy.init_node("arc")
		self.loc = [-108,135,-135,174,-80,-174]
		rospy.Subscriber("/yaw",Float32,self.rotate)
		self.pub_auto = rospy.Publisher("motion",WheelRpm,queue_size=10)
		self.pub_led = rospy.Publisher("led",Int8,queue_size=10)
		self.i = 0
		self.start = 0
		self.current_theta = 0
		self.go_straight = True
		self.reached = True
		self.t = True
		self.aruco_dict = aruco.Dictionary(5, 5)
		self.aruco_dict.bytesList = np.empty(shape = (10, 4, 4), dtype = np.uint8)
		mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
		self.aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1],[0,1,0,0,1]], dtype = np.uint8)
		self.aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0]], dtype = np.uint8)
		self.aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1]], dtype = np.uint8)
		self.aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
		self.aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,1,1,1]], dtype = np.uint8)
		self.aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,1,1,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
		self.aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,0,0,0],[0,1,0,0,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1]], dtype = np.uint8)
		self.aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,1,1,1]], dtype = np.uint8)
		self.aruco_dict.bytesList[8] = aruco.Dictionary_getByteListFromBits(mybits)
		mybits = np.array([[1,0,1,1,1],[0,1,1,1,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
		self.aruco_dict.bytesList[9] = aruco.Dictionary_getByteListFromBits(mybits)
	
	def get_frame(self):
	
		#frames = self.pipeline.wait_for_frames()
		#depth_frame = frames.get_depth_frame()
		#color_frame = frames.get_color_frame()
		#align=rs.align(rs.stream.color)
		#frames=align.process(frames)
		#depth_image = np.asanyarray(depth_frame.get_data())
		#color_image = np.asanyarray(color_frame.get_data())
		ret, frame = self.cap.read()
		#if not depth_frame or not color_frame:
		if not ret:
			return False, None
		#return True, depth_image, color_image, depth_frame
		return True, frame
        
	def rotate(self,msg):
        	self.current_theta=msg.data
        	
	def run(self):
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub_led.publish(1)
			rpm = WheelRpm()
			if self.start == 0:
				rpm.vel = 77
				rpm.omega = 127
				rpm.hb = False
				self.pub_auto.publish(rpm)
				rospy.sleep(0.5)
				self.start = 1
			if self.t:
				self.t = self.turn()
				self.t=False
			elif self.go_straight:
				rpm.vel = 77
				rpm.omega = 127
				rpm.hb = False
				self.pub_auto.publish(rpm)
				self.go_straight,_,_ = self.aruco()
			elif self.reached:
				_,pix,distance = self.aruco()
				#self.align(pix,distance)
			else:
				self.i = self.i+1
				self.t = True
				self.go_straight = True
				self.reached = True
				rospy.sleep(15)
				if self.i<len(self.loc):
					self.loc[self.i] = self.loc[self.i]+self.current_theta
				else:	
					rpm.vel = 127
					rpm.omega = 127
					rpm.hb = False
					self.pub_auto.publish(rpm)
					break	
			rate.sleep()
				
	def turn(self):
		
		omega = int((-self.loc[self.i]+self.current_theta)/180*60)
		if omega>0 and omega<25:
			omega=25
		elif omega<0 and omega>-25:
			omega=-25
		rpm = WheelRpm()
		rpm.vel = 127
		rpm.omega = 127 + omega
		rpm.hb = False
		self.pub_auto.publish(rpm)
		if abs((self.loc[self.i]-self.current_theta)) <= 10:
			return False
		else:
			return True
		
	def aruco(self):
	
		#ret, depth_image, frame, depth=self.get_frame()
		rpm=WheelRpm()
		ret, frame=self.get_frame()
		h, w, _ = frame.shape
		width = 1000
		height = int(width*(h/w))
		if ret:
			arucoParams = cv2.aruco.DetectorParameters()
			detector = cv2.aruco.ArucoDetector(self.aruco_dict,arucoParams)
			frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
			corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
			distance=2.5
			if len(corners)>0:
				frame = aruco.drawDetectedMarkers(frame, corners, ids)
				#cv2.imshow('frame',frame)
				#cv2.waitKey(1)
				print(corners)
				point = ((corners[0][0][0][0]+corners[0][0][1][0])/2,(corners[0][0][0][1]+corners[0][0][1][1])/2)
				#camera_info = depth.profile.as_video_stream_profile().intrinsics
				#distance = depth.get_distance(int(point[0]),int(point[1]))
				#centre = rs.rs2_deproject_pixel_to_point(camera_info,point,distance)
				#distance = centre[2]
				#print(distance)
				rpm.vel=102
				rpm.omega=127
				rpm.hb=false
				self.pub_auto.publish(rpm)
				rospy.sleep(7)
				return False, point[0], distance
			else:
				return True, 320, 5
		else:
			return True, 320, 5
        		
	def align(self,pix,distance):
		
		rpm = WheelRpm()
		if distance>1.5:
			if pix>400:
				rpm.vel = 127
				rpm.omega = 112
				self.pub_motor.publish(rpm)
			elif pix<240:
				rpm.vel=127
				rpm.omega=142
				self.pub_motor.publish(rpm)
			else:
				rpm.vel=127+min(25,int(25*distance/2.5))
				rpm.omega=127
				self.pub_auto.publish(rpm)
			self.reached=True
		else:
			rpm.vel=127
			rpm.omega=127
			self.pub_auto.publish(rpm)
			self.reached=False 
		
		
if __name__ == '__main__':
    node=auto()
    node.run()
