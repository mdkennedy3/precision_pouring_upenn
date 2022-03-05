import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import cv2

#Pull in video and use each frame

#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/april_pouring_1/april_pouring_1.avi')

cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/april_pouring_2/april_pouring_2.avi')
#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/april_pouring_3/april_pouring_3.avi')
#cap = cv2.VideoCapture('/home/sawyer/queen_ws/src/liquid_level_detection/bagfile/april_pouring_4/april_pouring_4.avi')
ret, old_frame = cap.read()


count = 0
offset = 8

Iline1 = 289
Iline2 = 323
Iline3 = 153
Iline4 = 440

#Use this frame to isolate the background and omit it


#loop through the frames
Iline = list()
while(True):
	ret, frame = cap.read()
	
	if(ret == False):
		print "Video is finished"
		break


	frame = cv2.subtract(old_frame, frame)

	cv2.imshow("Frame", frame)

	I = frame
	Ib = I[:,:,2]
	cv2.imshow("Blue Frame", Ib)
	cv2.waitKey(30)
	left = Ib[:,Iline2-offset]
	right = Ib[:,Iline2+offset]
	Iline.append(Ib[:,Iline2])
	
cv2.waitKey(1)
fig = plt.figure()
ax = plt.axes(xlim=(0, 480), ylim=(0, 255))
line, = ax.plot([],[], lw =2)
print type(line)
#print Iline
#plt.figure(count)
def init():
	line.set_data([],[])
	return line,

def animate(i):
	x = np.linspace(0,480,480) 
	y = Iline[i]
	#print y
	line.set_data(x,y)
	return line,
#print np.linspace(0,480, 480)
#ani = animation.ArtistAnimation(fig, Iline, interval=50, blit=True,repeat_delay=1000)
#for i in range(len(Iline)):
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(Iline), interval=30, blit=True, repeat=False)
#	count+=1
#print len(Iline[0])
#plt.plot(Iline)
plt.show()
mywriter = animation.MencoderWriter()
mywriter.fps = 30

ani.save('Plot_Pouring_2_Anim.mp4',writer=mywriter,fps=1, extra_args=['-vcodec', 'libx264']) 
#ani.save('Plot_Pouring_2_Anim.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

