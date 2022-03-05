import time
import argparse
import roslib
import sys
import rospy
import math
import tf
import cv2
import geometry_msgs.msg
import numpy as np
from apriltag_msgs.msg import ApriltagArrayStamped
from cv_bridge import CvBridge, CvBridgeError
from liquid_level_detection.msg import LDHeight
from sensor_msgs.msg import Image

#Global Variables


#sub_image = None

class Liquid_Level_Detector:


	def __init__(self):

		self.bridge = CvBridge()
		self.image = None
		self.processed = False
		self.apriltag = None
		self.el = 0
		self.er = 0
		self.em = 0
		self.count = 0 #Keeps track of the base image for the subtraction
		self.sub_image = None
		self.prevPick = 0
		self.newPick = 0
		self.selection = None
		self.AT_Plength = 0.0
		self.cyl_bottomP = 0 
		self.cylinderLength = .096 #Real world measurements are in meters
		self.AT_data = False
		self.threshold1 = 20 #20
		self.threshold2 = 150 #200
		self.AT_size = .017
		self.cutoff = 200
		self.offset = 50
		self.alpha = .05
		self.cutArea = .025 #RW Area to remove and replace from detection
		self.pick = 0 #Place holder for the estimation of the cut detection
		self.Iline = list()
		self.middleL = list()
		self.leftL = list()
		self.rightL = list()
		self.full_edgeL = list()
		self.full_edgeR = list()
		self.full_edgeM = list()
		self.full_edgeP = list()
		self.pub = rospy.Publisher('/LDHeight', LDHeight, queue_size = 10, latch = True)
		self.tag_sub = rospy.Subscriber('/camera/apriltags', ApriltagArrayStamped, self.Aprilcallback)
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.Imagecallback)

		#self.line_sub = rospy.Subscriber("", , self.callback)
		
		
	def Aprilcallback(self, data):
	

		if type(data) is ApriltagArrayStamped:
			
			if(self.AT_data is False):
				self.apriltag = data
				self.AT_data = True
				#AT Calculations
				bottomRight = self.apriltag.apriltags[0].corners[1]
				topRight = self.apriltag.apriltags[0].corners[2]
			
				self.AT_Plength = math.sqrt(math.pow((bottomRight.x - topRight.x),2) + math.pow((bottomRight.y - topRight.y),2))
				

				#Setting new Cutoff area to simplify pouring
				self.cutoff = int(self.apriltag.apriltags[0].center.y +((self.cutArea*self.AT_Plength)/self.AT_size))

				pix_difference = (self.AT_Plength*self.cylinderLength)/self.AT_size
				self.cyl_bottomP = int(self.apriltag.apriltags[0].center.y)+ pix_difference

		
	def Imagecallback(self, data):
	
		if(self.AT_data == False):
			return
		else:
			if type(data) is Image:
					self.Image_Data = data
					cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
					if(self.count == 0):
						self.sub_image = cv_image
						self.count+=1
						self.preProcess()
					self.image = cv_image
					self.processed = False
					self.processFrame()

	def preProcess(self):
		#print "In preProcess"
	
		
		Iline2 = int(self.apriltag.apriltags[0].center.x)
		
		if(self.sub_image is None):
			return
		else:
			Ib = self.sub_image[:,:,2]
			spec_lineL = Ib[:,Iline2-self.offset]
			spec_lineR = Ib[:,Iline2+self.offset]
			spec_lineM = Ib[:,Iline2]
			spec_lineLC = spec_lineL[self.cutoff:]
			spec_lineRC = spec_lineR[self.cutoff:]
			spec_lineMC = spec_lineM[self.cutoff:]
			edgeL = cv2.Canny(spec_lineLC,self.threshold1,self.threshold2)
			edgeR = cv2.Canny(spec_lineRC,self.threshold1,self.threshold2)
			edgeM = cv2.Canny(spec_lineMC,self.threshold1,self.threshold2)
			el_ind = [idx for idx in range(480-self.cutoff) if edgeL[idx]>0]
			er_ind = [idx for idx in range(480-self.cutoff) if edgeR[idx]>0]
			em_ind = [idx for idx in range(480-self.cutoff) if edgeM[idx]>0] 
			if(len(el_ind)>0):
				self.el = el_ind[0]
			if (len(er_ind)>0):
				self.er = er_ind[0]
			if (len(em_ind)>0):
				self.em = em_ind[0]
				print self.selection
			if(self.selection == "A"):
				check = [self.el,self.em,self.er]
				self.pick = np.max(check)
			elif(self.selection == "L"):
				self.pick = self.el
			elif(self.selection == "R"):
				self.pick = self.er
			elif(self.selection == "M"):
				self.pick = self.em
			self.prevPick = self.pick+self.cutoff
			

		





	def processFrame(self):
		
		Iline2 = int(self.apriltag.apriltags[0].center.x)
		#print "In process frame"
		
		#Accessing the data of the april tag is below
		#print self.apriltag.apriltags[0].center.x
		

		new_frame = self.image 
		
		frame = cv2.subtract(self.sub_image, new_frame)
		Ib = frame[:,:,2]
		#cv2.imshow("Ib", Ib)
		cv2.waitKey(30)
		spec_lineL = Ib[:,Iline2-self.offset]
		spec_lineR = Ib[:,Iline2+self.offset]
		spec_lineM = Ib[:,Iline2]
		spec_lineLC = spec_lineL[self.cutoff:]
		spec_lineRC = spec_lineR[self.cutoff:]
		spec_lineMC = spec_lineM[self.cutoff:]
		edgeL = cv2.Canny(spec_lineLC,self.threshold1,self.threshold2)
		edgeR = cv2.Canny(spec_lineRC,self.threshold1,self.threshold2)
		edgeM = cv2.Canny(spec_lineMC,self.threshold1,self.threshold2)
		edgeP = np.zeros(480-self.cutoff,)
		el_ind = [idx for idx in range(480-self.cutoff) if edgeL[idx]>0]
		er_ind = [idx for idx in range(480-self.cutoff) if edgeR[idx]>0]
		em_ind = [idx for idx in range(480-self.cutoff) if edgeM[idx]>0] 
		if(len(el_ind)>0):
			self.el = el_ind[0]
		if (len(er_ind)>0):
			self.er = er_ind[0]
		if (len(em_ind)>0):
			self.em = em_ind[0]

			print self.selection
		if(self.selection == "A"):
			check = [self.el,self.em,self.er]
			self.pick = np.max(check)
		elif(self.selection == "L"):
			self.pick = self.el
		elif(self.selection == "R"):
			self.pick = self.er
		elif(self.selection == "M"):
			self.pick = self.em
		edgeP[self.pick] = 255
		self.newPick = self.pick+self.cutoff
		
		#IMPLEMENT THE LOW PASS FILTER
		pickbar = int(self.alpha*self.newPick + (1-self.alpha)*self.prevPick)
		print "Pick Bar: "+str(pickbar)
		cv2.circle(new_frame,(Iline2,pickbar),5,(0,255,0),-1)
		cv2.imshow("Updated Frame", new_frame)
		cv2.waitKey(30)
		#pull in 
		name = 'frame %04d.jpg'%self.count
		cv2.imshow("frame",new_frame)
		#cv2.imwrite(name,new_frame)
		self.count+=1

		#Build the message to publish 
		msg = LDHeight()

		#Build the image data
		msg.adjusted_image = self.Image_Data
		


		heightP = self.cyl_bottomP - pickbar
		
		heightCM = (heightP*self.AT_size)/self.AT_Plength
		print heightCM*100
		msg.h = heightCM*100
		self.pub.publish(msg)
		self.prevPick = self.newPick
		self.processed = True
		
	#sys.exit(0)

def main():
	SelecMade = False
	parser = argparse.ArgumentParser(description='Please make a selection of which lines to use with low pass filter (L)eft, (R)ight (M)iddle (A)ll')
	parser.add_argument('--selection', help='Selection for the %(prog)s Low Pass Filter focus')
	args = parser.parse_known_args()
	selection = args[0].selection
	if(selection.isalpha()):
		selection = str(selection.upper())
		if(selection == "L" or selection == "R" or selection == "M" or selection == "A"):
				print "Valid Selection"
				SelecMade = True
		else: 
				print "Your selection ("+str(selection)+") is invalid. Program Exiting"
				sys.exit(0)
	else:
		print "Your selection ("+str(selection)+") is invalid. Program Exiting"
		sys.exit(0)
	rospy.init_node('Liquid_Level_Detector_Node', anonymous = True)
	sleeper = rospy.Rate(10)
	listener = tf.TransformListener()
	# SelecMade = False
	
	#FUll proof the selection process
	# while(SelecMade is False):
	# 	selection = raw_input("Please make a selection of which lines to use with low pass filter (L)eft, (R)ight (M)iddle (A)ll : ")
	# 	if(selection.isalpha()):
	# 		selection = str(selection.upper())
	# 		if(selection == "L" or selection == "R" or selection == "M" or selection == "A"):
	# 			print "Valid Selection"
	# 			SelecMade = True
	# 		else: 
	# 			print "Your selection ("+str(selection)+") is invalid. Try again."
	# 	else:
	# 		print "Your selection ("+str(selection)+") is invalid. Try again."
	lld = Liquid_Level_Detector()
	lld.selection = selection
	
	
	#Thinking that I do a seperate function that will run to get the info from the first image to use.
	#and use that as the first prev pick. Will need to get the april tag data in that function as well
	#Check to make sure everything is in place otherwise you might be fucked somewhere!
	
	#prevPick, AT_Plength = lld.preProcess(selection)

	#print "Prev_Pick: "+str(prevPick)
	#print "April Tag Length: "+str(AT_Plength)

	while not rospy.is_shutdown():
		if lld.processed is False:
		#	prevPick = lld.processFrame(prevPick,selection)
			sleeper.sleep()
		else:
			print "Already Processed"



	


if __name__ == '__main__':
	main()

	