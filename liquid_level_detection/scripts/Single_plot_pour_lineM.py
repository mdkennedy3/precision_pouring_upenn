import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import cv2
import sys

#This is an attempt to isolate one of the lines along the cylinder and use that in conjuction with the LPF


#Pull in video and use each frame

#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/pouringproject/liquid_level_detection/bagfile/april_pouring_1/april_pouring_1.avi')

cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/pouringproject/liquid_level_detection/bagfile/april_pouring_2/april_pouring_2.avi')
#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/pouringproject/liquid_level_detection/bagfile/april_pouring_3/april_pouring_3.avi')
#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/pouringproject/liquid_level_detection/bagfile/april_pouring_4/april_pouring_4.avi')
ret, old_frame = cap.read()


count = 00000
offset = 50

Iline1 = 289
Iline2 = 323
Iline3 = 153
Iline4 = 440


threshold1 = 20 #20
threshold2 = 150 #200
#Use this frame to isolate the background and omit it
frame_num = 0
cutoff = 200
alpha = .04

#Frame Lists for lines
Iline = list()
middleL = list()
leftL = list()
rightL = list()
full_edgeL = list()
full_edgeR = list()
full_edgeM = list()
full_edgeP = list()
el = 0
er = 0
em = 0
#Setup processing for the LP filter
Ib = old_frame[:,:,2]

spec_lineR = Ib[:,Iline2]
spec_lineRC = spec_lineR[cutoff:]
edgeR = cv2.Canny(spec_lineRC,threshold1,threshold2)
er_ind = [idx for idx in range(480-cutoff) if edgeR[idx]>0]

if (len(er_ind)>0):
	er = er_ind[0]

pick = er
prevPick = pick+cutoff

#loop through the frames
while(True):
	ret, frame = cap.read()
	
	if(ret == False):
		print "Video is finished"
		break
	else:
		frame_num+=1

####### Frame Processing ##########################
	frame = cv2.subtract(old_frame, frame)

	#cv2.imshow("Frame", frame)

	I = frame
	Ib = I[:,:,2]
	
	
	spec_lineR = Ib[:,Iline2]

	#print spec_lineM
	#Eliminate issues about 200
	
	spec_lineRC = spec_lineR[cutoff:]
	edgeR = cv2.Canny(spec_lineRC,threshold1,threshold2)
	edgeP = np.zeros(480-cutoff,)
	#print edgeP
	
	er_ind = [idx for idx in range(480-cutoff) if edgeR[idx]>0]
	if (len(er_ind)>0):
		er = er_ind[0]

	
	print "frameM #"+str(frame_num)
	
	pick = er 
	edgeP[pick] = 255
	newPick = pick+cutoff

#IMPLEMENT THE LOW PASS FILTER
	pickbar = int(alpha*newPick + (1-alpha)*prevPick)

	print pickbar

	cv2.circle(frame,(Iline2,pickbar),5,(0,255,0),-1)
	cv2.imshow("Frame", frame)
	#cv2.imshow("Blue Frame", Ib)
	cv2.waitKey(30)

	name = 'frame %04d.jpg'%count
	#cv2.imshow("frame",frame)
	cv2.imwrite(name,frame)
	count+=1

	
	edgeR = cv2.Canny(spec_lineR,threshold1,threshold2)
	
	edgeP = np.hstack([np.zeros(cutoff,),edgeP])
	edgeP = list(map(int,edgeP))
	

	full_edgeP.append(edgeP)
	#Saving the edge images into lists
	
	full_edgeR.append(edgeR)


	#Saving the lines into lists for animation
	
	rightL.append(spec_lineR)
	
	#print Iline

	#Iline.append(Ib[:,Iline2])
	prevPick = pickbar

cv2.waitKey(1)
####### Frame Processing ##########################






#### Figure animation Details#################################
fig1 = plt.figure()
ax = plt.axes(xlim=(0, 480), ylim=(0, 255))
ax.set_xlabel('Y-Pixel in Image')
ax.set_ylabel('Pixel Intensity')
plt.title('Liquid Level Estimation Focused on Middle w/LP Alpha = 0.04')
#linem, = ax.plot([],[], color='green', lw =2, label='Middle Line')
#linel, = ax.plot([],[], color='red', lw =2, label='Left Line')
#liner, = ax.plot([],[], color='blue', lw =2,label='Right Line')
linep, = ax.plot([],[], color='green', lw =2,label='True Line')
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles,labels)
plt.legend(loc=2)
#### Figure animation Details#################################





#print type(line)
#print Iline
#plt.figure(count)
def init():
	#linem.set_data([],[])
	#linel.set_data([],[])
	#liner.set_data([],[])
	linep.set_data([],[])
	return linep,#linem,linel,liner,linep,

def animate(i):
	x = np.linspace(0,480,480) 
	
	yr = full_edgeR[i]
	yp = full_edgeP[i]
	#ym = middleL[i]
	#yl = leftL[i]
	#yr = rightL[i]
	#print y
	#linem.set_data(x,ym)
	#linel.set_data(x,yl)
	#liner.set_data(x,yr)
	linep.set_data(x,yp)
	return linep,#linem,linel,liner,linep,

#print np.linspace(0,480, 480)
#ani = animation.ArtistAnimation(fig, Iline, interval=50, blit=True,repeat_delay=1000)
#for i in range(len(Iline)):
ani = animation.FuncAnimation(fig1, animate, init_func=init, frames=frame_num-1, interval=30, blit=True, repeat=False)
#	count+=1
#print len(Iline[0])
#plt.plot(Iline)
plt.show()
mywriter = animation.MencoderWriter()
mywriter.fps = 30

ani.save('MiddleL_Pouring_LPA04.mp4',writer=mywriter,fps=1, extra_args=['-vcodec', 'libx264']) 
#ani.save('Plot_Pouring_2_Anim.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

