import cv2
import numpy as np
import cv2.cv as cv
import sys

boxes = []

cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/pouring_vid.avi')

ret, old_frame = cap.read()
print ret
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

left_pt = (120,415) #left_pt = (110,425)
right_pt = (335,415)#right_pt = (330,425)
line_thickness = 5 #Changes the area of the points we are looking at.

#cv2.circle(old_gray, (110,405), 25, (0,255,0))
#cv2.circle(old_gray, (330,405), 25, (0,255,0))
#cv2.line(old_gray,left_pt,right_pt,(255,0,0),line_thickness)
cv2.imshow("Frame", old_gray)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )


# Parameters for lucas kanade optical flow
#Notes I found that the 200,200 winsize increases the tensile strength of the line which may be beneficial
lk_params = dict( winSize  = (25,25),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))#.03

#Need to make a list of points 
#y = np.float32(left_pt[1])
holder = list([[np.float32(x),np.float32(y)]]for x in range(left_pt[0],right_pt[0],1) for y in range(left_pt[1], left_pt[1]+line_thickness,1))
#pt_arr = np.array(holder,ndmin =3)

#Create some colors
color = np.random.randint(0,255,(100,3))

#print pt_arr
pt_arr = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
#print p0
#print pt_arr

newList = list()

#Filter out the unnecessary points
for i in range(len(pt_arr)):
	if pt_arr[i][0][1]> 300.:
		newList.append([pt_arr[i][0]])
		#print np.delete(pt_arr,i,None) 
	
#print "New List"
#print len(pt_arr)
pt_arr = np.array(newList)


##print type(p0[0])
#MUST CONVERT THE PIXEL POINTS INTO THIS FORM !!!!!!!!
#sys.exit(0)
#cv2.waitKey(0)

#Start running the optical flow algorithm to make it work.

mask_image = np.zeros_like(old_frame)
while(True):
	ret, frame = cap.read()
	
	if(ret == False):
		print "Video is finished"
		break
	
	print type(mask_image)
	#cv2.imshow("New Frame", frame)

	#cv2.waitKey(0)
	frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	

	#Calculate OpFlow
	p1 , st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, pt_arr, None, **lk_params)

	#Eliminate the x motion in the optical flow points
	for i in range(len(p1)):
		p1[i][0][0] = pt_arr[i][0][0]

	#sys.exit(0)
	#Select the good points
	good_new = p1[st == 1 ]
	good_old = pt_arr[st == 1]



	#print good_new

	#draw the tracks
	for i, (new,old) in enumerate(zip(good_new,good_old)):
		a,b = new.ravel()
		c,d = old.ravel()
		cv2.arrowedLine(mask_image, (40,d), (40,b), color[i].tolist(),2)
		cv2.circle(frame,(a,b),5,color[i].tolist(),-1)

	img = cv2.add(frame, mask_image)

	cv2.imshow('Frame', img)
	cv2.waitKey(10)


	#Update the previous frame and points
	old_gray = frame_gray.copy()
	pt_arr = good_new.reshape(-1,1,2)


cv2.waitKey(0)
	#print p1
	#print st





