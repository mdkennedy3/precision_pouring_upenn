import cv2
import numpy as np
import cv2.cv as cv

boxes = []

cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/pouring_vid.avi')

ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

left_pt = (150,390) #left_pt = (110,425)
right_pt = (300,390)#right_pt = (330,425)
line_thickness = 5

#cv2.circle(old_gray, (110,405), 25, (0,255,0))
#cv2.circle(old_gray, (330,405), 25, (0,255,0))
#cv2.line(old_gray,left_pt,right_pt,(255,0,0),line_thickness)
#cv2.imshow("Frame", old_gray)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )


# Parameters for lucas kanade optical flow
#Notes I found that the 200,200 winsize increases the tensile strength of the line which may be beneficial
lk_params = dict( winSize  = (40,40),
                  maxLevel = 50,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1, 0.75))#.03

#Need to make a list of points 
#y = np.float32(left_pt[1])
holder = list([[np.float32(x),np.float32(y)]]for x in range(left_pt[0],right_pt[0],1) for y in range(left_pt[1], left_pt[1]+5,1))
pt_arr = np.array(holder,ndmin =3)

#print pt_arr
#p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
#print p0
#print pt_arr[3][0][0]
#print type(p0[0])
#MUST CONVERT THE PIXEL POINTS INTO THIS FORM !!!!!!!!

#cv2.waitKey(0)

#Start running the optical flow algorithm to make it work.


while(True):
	ret, frame = cap.read()
	
	if(ret == False):
		print "Video is finished"
		break
	mask_image = np.zeros_like(frame)
	print type(mask_image)
	#cv2.imshow("New Frame", frame)

	#cv2.waitKey(0)
	frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	

	#Calculate OpFlow
	p1 , st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, pt_arr, None, **lk_params)


	#Select the good points
	good_new = p1[st == 1]
	good_old = pt_arr[st == 1]



	print good_new

	#draw the tracks
	for i, (new,old) in enumerate(zip(good_new,good_old)):
		a,b = new.ravel()
		c,d = old.ravel()
		cv2.arrowedLine(frame, (a,b), (c,d), (0,255,0),5)

	print type(mask_image)
	print type(frame)
	#img = cv2.add(frame, mask_image)

	cv2.imshow('Frame', frame)
	cv2.waitKey(10)


	#Update the previous frame and points
	old_gray = frame_gray.copy()
	pt_arr = good_new.reshape(-1,1,2)


cv2.waitKey(0)
	#print p1
	#print st





