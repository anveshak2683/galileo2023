#!/usr/bin/env python
import cv2
import time
import os

time.sleep(1)
# define a video capture object
vid = cv2.VideoCapture(1)
directory = r'/home/nvidia/caesar2020/src/vision/images'

os.chdir(directory)

for i in range(29):
    ret, frame=vid.read()
    filename="img"+str(i)+".jpg"
    cv2.imwrite(filename, frame)
    time.sleep(1.5)
 
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
