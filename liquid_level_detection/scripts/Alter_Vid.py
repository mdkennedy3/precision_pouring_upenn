import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import cv2
import sys


cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/april_pouring_2/april_pouring_2.avi')

offset = 50
Iline2 = 323

count = 0000
#fourcc = cv2.VideoWriter_fourcc(*'XVID')

video = cv2.VideoWriter("new_april_pouring_2.avi",-1,30.0,(640,480),True)


while(True):
	ret, frame = cap.read()
	
	if(ret == False):
		print "Video is finished"
		break

	cv2.line(frame,(Iline2,150),(Iline2,420), (0,255,0)) #Green middle line
	cv2.line(frame,(Iline2+offset,150),(Iline2+offset,420), (0,0,255)) #Blue right line
	cv2.line(frame,(Iline2-offset,150),(Iline2-offset,420), (255,0,0)) #Red left Line
	name = "frame"+str(count)+".jpg"
	cv2.imshow("frame",frame)
	cv2.imwrite(name,frame)
	count+=1
	cv2.waitKey(30)
	video.write(frame)
video.release()