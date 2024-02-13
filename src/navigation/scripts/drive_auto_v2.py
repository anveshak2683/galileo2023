#!/usr/bin/env python
import pyrealsense2 as rs
import rospy
from navigation.msg import gps_data
import math
from time import *
import cv2
import numpy as np
import imutils
from navigation.msg import imu_angle
from traversal.msg import WheelRpm
from traversal.srv import *
import std_msgs.msg as std_msgs
import threading


class auto():

    def __init__(self):
        #self.cap=cv2.VideoCapture(0)
        rospy.on_shutdown(self.close)
        self.pipeline = rs.pipeline()
        config = rs.config()
        rospy.init_node("arrowdetect")
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.template_r=cv2.imread('Template.png',0)
        self.template_l=cv2.imread('Template_l.png',0)
        self.template_r=cv2.resize(self.template_r,(60,40),cv2.INTER_AREA)
        self.template_l=cv2.resize(self.template_l,(60,40),cv2.INTER_AREA)
        self.h,self.w=self.template_r.shape
        self.z_angle = 0
        self.rotate_incr = 0
        self.turn = False
        self.min_dist=1.5
        self.distance=None
        self.direction=None
        self.current_latitude=None
        self.current_longitude=None
        self.ret=False
        self.enc_data = 0
        self.start_angle = 50
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        self.init = False
        self.start_time = time.time()
        self.time_thresh = 5 #threshold time

        try:
            rospy.Subscriber('imu_angles', imu_angle, self.angles_callback)
            rospy.Subscriber('gps_data', gps_data, self.gps_callback)
            rospy.Subscriber('enc_auto', std_msgs.Int8, self.enc_callback)
        except Exception(e):
            print("error")  


    def enc_callback(self,msg):
        self.enc_data = msg.data

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        align=rs.align(rs.stream.color)
        frames=align.process(frames)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None, None
        return True, depth_image, color_image, depth_frame
    
    def arrowdetect(self,image,depth):
        #_,image=self.cap.read()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found=None
        found_l=None
        for scale in np.linspace(0.06, 1.0, 70)[::-1]:
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
            if resized.shape[0] < self.h or resized.shape[1] < self.w:
                break
            #cv2.imshow("",resized)
            result = cv2.matchTemplate(resized, self.template_r, cv2.TM_CCOEFF_NORMED)
            minVal,maxVal,minLoc,maxLoc = cv2.minMaxLoc(result)
            result = cv2.matchTemplate(resized, self.template_l, cv2.TM_CCOEFF_NORMED)
            minVal_l,maxVal_l,minLoc_l,maxLoc_l = cv2.minMaxLoc(result)
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
            if found_l is None or maxVal_l > found_l[0]:
                found_l = (maxVal_l, maxLoc_l, r)
        (maxVal, maxLoc, r) = found
        (maxVal_l, maxLoc_l, r) = found_l

        if maxVal_l>maxVal: 
            maxVal=maxVal_l
            maxLoc=maxLoc_l
            direction='left'
        
        else:
            direction='right'
        
        if maxVal>0.70:
            (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
            (endX, endY) = (int((maxLoc[0]+self.w)*r), int((maxLoc[1]+self.h)*r))
            cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
            point=(int((startX+endX)/2),int((startY+endY)/2))
            camera_info=depth.profile.as_video_stream_profile().intrinsics
            distance=depth.get_distance(point[0],point[1])
            centre=rs.rs2_deproject_pixel_to_point(camera_info,point,distance)
            distance=centre[2]
            print(point[0])
            return True, direction,point[0],distance
        return False,None,None,None

    def search(self):
        if(time.time() - self.start_time < self. time_thresh):
            return
        msg = std_msgs.Int32MultiArray()

        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0

        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'

        while self.init == False and self.enc_data < abs(self.start_angle) :
            msg.data = [-255,0,0,0,0,0]
            self.pub.publish(msg)
        self.init = True
        if self.init == True and self.enc_data < abs(self.start_angle) and not self.ret:
            msg.data = [255,0,0,0,0,0]
            self.pub.publish(msg)
        elif self.init == True and self.enc_data < abs(self.start_angle) and self.ret:
            msg.data = [0,0,0,0,0,0]
            self.start_time = time.time()
            self.init = False
        




            
    
    def main(self):
        if(not self.turn):
            ret, depth_frame, color_frame, depth = self.get_frame()     
            self.ret,self.direction,pix,self.distance=self.arrowdetect(color_frame,depth)
            self.search()   
     
            if (self.ret):
                print("arrow detected at distance: "+str(self.distance))
                print(self.direction)
                print("Angle of detection: "+str(self.enc_data))
            self.pub.publish(msg)
            #self.move_straight()
        else:
            if(self.direction=="left"):
                self.rotate(-1)
            else:
                self.rotate(1)
    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

    def angles_callback(self,msg):
        self.z_angle = msg.Yaw - self.rotate_incr * 90

    def gps_callback(self,msg):
        self.current_latitude=msg.latitude
        self.current_longitude=msg.longitude

    def write_coordinates(self):
        file_object=open("coordinates.txt","a")
        file_object.write("latitude: "+str(self.current_latitude)+" longitude: "+ str(self.current_longitude))
        file_object.close()

    def move_straight(self):
        msg =WheelRpm()
        msg.hb=False
        if(self.ret and self.distance<=self.min_dist):
            msg.vel = 127
            msg.omega = 127
            wheelrpm_pub.publish(msg)
            rospy.sleep(10)
            self.turn = True
            self.write_coordinates()
        else:
            msg.vel = 102
            msg.omega = 127
            wheelrpm_pub.publish(msg)
            self.turn=False

    def rotate(self,dir):
        msg = WheelRpm()
        msg.vel = 127
        msg.hb=False  
        error = 5
        if(-(90-error)<self.z_angle<(90-error)):
            msg.omega=127+(-50*dir)
            print("Calling Rotate, printing Z angle below")
            print(self.z_angle)
            wheelrpm_pub.publish(msg)
        else:
            self.rotate_incr+=dir
            print(self.rotate_incr)
            msg.omega=127
            wheelrpm_pub.publish(msg)
            self.z_angle=0
            print("****ROTATE DONE*****")
            self.turn=False
            rospy.sleep(2)

    def close(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=127
        wheelrpm_pub.publish(msg_stop)

if __name__ == '__main__':
    rospy.init_node("arrowdetect")
    rate=rospy.Rate(10)
    wheelrpm_pub=rospy.Publisher('motion',WheelRpm,queue_size=10)
    run=auto()
    run.spin()
