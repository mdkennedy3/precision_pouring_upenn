#!/usr/bin/env python2
import time
import argparse
import roslib
import sys
import rospy
import math
import tf
import cv2
import geometry_msgs.msg
import numpy as np, copy
from apriltag_msgs.msg import ApriltagArrayStamped
from cv_bridge import CvBridge, CvBridgeError
from liquid_level_detection.msg import LDHeight
from sensor_msgs.msg import Image
from std_msgs.msg import Float64



#Global Variables


#sub_image = None

class Liquid_Level_Detector:


    def __init__(self, debug, selection):

        #gradient approach
        self.gradient_threshold = 50 #this is tunable for the threshold value (sensitive to lighting)

        #for median filter
        self.pickbar_median_set = 10 #at 30hz this  #was 3  #use these many points for median filter
        self.pickbar_median_list = []

        self.debug = debug
        self.image_pub = rospy.Publisher('/LDImage', Image, queue_size = 1, latch = True)


        self.bridge = CvBridge()
        self.image = None
        self.image_width = None
        self.image_height = None
        self.processed = False
        self.apriltag = None
        self.el = 0
        self.er = 0
        self.em = 0
        self.count = 0 #Keeps track of the base image for the subtraction
        self.sub_image = None
        self.prevPick = 0
        self.newPick = 0
        self.selection = selection # None
        self.AT_Plength = 0.0
        self.cyl_bottomP = 0
        # self.cylinderLength = .096 #Real world measurements are in meters

        #Tag over the markers
        # self.cylinderLength = .0905 #Real world measurements are in meters

        #offset for side tag
        #self.cylinderLength = .0879 #Real world measurements are in meters

        #offset for side tag, with kinect on ground
        self.cylinderLength = .0996 #Real world measurements are in meters



        self.AT_data = False
        self.threshold1 = 20 #20
        self.threshold2 = 100 #200
        self.AT_size = .017
        self.cutoff = 0
        self.offset = 50
        self.alpha = .05
        self.cutArea = .025 #RW Area to remove and replace from detection
        self.pick = 0 #Place holder for the estimation of the cut detection
        self.desHeightCM = 0.0
        self.line_data = False
        self.Iline = list()
        self.middleL = list()
        self.leftL = list()
        self.rightL = list()
        self.full_edgeL = list()
        self.full_edgeR = list()
        self.full_edgeM = list()
        self.full_edgeP = list()

        self.fgbg = cv2.createBackgroundSubtractorMOG2(20)

        self.pub = rospy.Publisher('/LDHeight', LDHeight, queue_size = 10, latch = True)
        self.tag_sub = rospy.Subscriber('/camera/apriltags', ApriltagArrayStamped, self.Aprilcallback)
        self.image_sub = rospy.Subscriber("image", Image, self.Imagecallback)
        self.line_sub = rospy.Subscriber("/des_height", Float64 , self.lineCallback)

        self.learned_background = []
        self.added_fgmask = []


    def Aprilcallback(self, data):

        if type(data) is ApriltagArrayStamped:
            if data.apriltags:
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
            print "No AT Data"
            return
        elif(self.line_data == False):
            print "No Line Data"
            return
        else:
            if type(data) is Image:
                self.header= data.header
                self.image_width = data.width
                self.image_height = data.height
                cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                if(self.count < 10):
                    self.sub_image = cv_image
                    self.count+=1
                    self.preProcess()
                    return

                self.image = cv_image
                self.processed = False
                self.processFrame()

    def lineCallback(self,req):
        self.desHeightCM = req.data
        self.line_data = True
        #print self.desHeight


    def calc_grad2(self,Pt_list):
        #Gradient Calculation (replaces canny usage)
        Pgrad = [np.zeros(Pt_list.shape[1]),Pt_list[1,:]-Pt_list[0,:]]
        Pgrad_add = [(Pt_list[jdx,:]-Pt_list[jdx-2,:])/2. for jdx in range(2,Pt_list.shape[0])]
        Pg = np.vstack([np.vstack(Pgrad),np.vstack(Pgrad_add)])
        Pg_list = Pg.tolist()

        #print Pg_list
        #Use a hard maximum:
        # Pidx = np.argmax(np.abs(Pg_list))
        # Pg_return = [255*(idx == Pidx) for idx in range(len(Pg_list))]
        #Use threshold
        # Pg_return = [[255*(Pg_list[idx][jdx] >= self.gradient_threshold) for idx in range(Pg.shape[0])] for jdx in range(Pg.shape[1])]
        
        # for loop
        '''
        Pg_return = []
        for jdx in range(Pg.shape[1]):
            Pgtemp = []
            for idx in range(Pg.shape[0]):
                Pgtemp.append(255*(np.abs(Pg_list[idx][jdx])>=self.gradient_threshold) )
            Pg_return.append(Pgtemp)
		'''
        #For loop vectorized in comprehension
        Pg_return = [[ 255*(np.abs(Pg_list[idx][jdx])>=self.gradient_threshold) for idx in range(Pg.shape[0])] for jdx in range(Pg.shape[1]) ]

        return  Pg_return

    def calc_grad(self,Pt_list):
        #Gradient Calculation (replaces canny usage)
        Pgrad = [0,Pt_list[1]-Pt_list[0]]
        Pgrad_add = [(Pt_list[jdx]-Pt_list[jdx-2])/2. for jdx in range(2,len(Pt_list))]
        Pg = np.hstack([np.array(Pgrad),np.array(Pgrad_add)])
        Pg_list = Pg.tolist()

        #print Pg_list
        #Use a hard maximum:
        # Pidx = np.argmax(np.abs(Pg_list))
        # Pg_return = [255*(idx == Pidx) for idx in range(len(Pg_list))]
        #Use threshold
        Pg_return = [255*(Pg_list[idx] >= self.gradient_threshold) for idx in range(len(Pg_list))]
        return  Pg_return


    def preProcess(self):
        #print "In preProcess"
        Iline2 = int(self.apriltag.apriltags[0].center.x)

        if(self.sub_image is None):
            return
        else:
            Ib = self.sub_image[:,:,2]

            fgmask = self.fgbg.apply(self.sub_image[:,:,2])
            cv2.imshow('fgblue', fgmask)
            print 'learning background'

            self.learned_background = self.fgbg.getBackgroundImage()
            #frame = cv2.subtract(background, new_frame[:,:,2])

            Ib = fgmask

            spec_lineL = Ib[:,Iline2-self.offset]
            spec_lineR = Ib[:,Iline2+self.offset]
            spec_lineM = Ib[:,Iline2]

            spec_lineL[1:self.cutoff] = 0
            spec_lineR[1:self.cutoff] = 0
            spec_lineM[1:self.cutoff] = 0

            edgeL = self.calc_grad(spec_lineL)
            edgeR = self.calc_grad(spec_lineR)
            edgeM = self.calc_grad(spec_lineM)

            el_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeL[idx]>0]
            er_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeR[idx]>0]
            em_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeM[idx]>0]
            if(len(el_ind)>0):
                self.el = el_ind[0]
            if (len(er_ind)>0):
                self.er = er_ind[0]
            if (len(em_ind)>0):
                self.em = em_ind[0]
                #print self.selection
            if(self.selection == "A"):
                check = [self.el,self.em,self.er]
                self.pick = np.max(check)
            elif(self.selection == "L"):
                self.pick = self.el
            elif(self.selection == "R"):
                self.pick = self.er
            elif(self.selection == "M"):
                self.pick = self.em

            self.prevPick = self.pick

            # spec_lineLC = spec_lineL[self.cutoff:]
            # spec_lineRC = spec_lineR[self.cutoff:]
            # spec_lineMC = spec_lineM[self.cutoff:]

            # # edgeL = cv2.Canny(spec_lineLC,self.threshold1,self.threshold2)
            # # edgeR = cv2.Canny(spec_lineRC,self.threshold1,self.threshold2)
            # # edgeM = cv2.Canny(spec_lineMC,self.threshold1,self.threshold2)
            # edgeL = self.calc_grad(spec_lineLC)
            # edgeR = self.calc_grad(spec_lineRC)
            # edgeM = self.calc_grad(spec_lineMC)


            # el_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeL[idx]>0]
            # er_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeR[idx]>0]
            # em_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeM[idx]>0]
            # if(len(el_ind)>0):
            #     self.el = el_ind[0]
            # if (len(er_ind)>0):
            #     self.er = er_ind[0]
            # if (len(em_ind)>0):
            #     self.em = em_ind[0]
            #     #print self.selection
            # if(self.selection == "A"):
            #     check = [self.el,self.em,self.er]
            #     self.pick = np.max(check)
            # elif(self.selection == "L"):
            #     self.pick = self.el
            # elif(self.selection == "R"):
            #     self.pick = self.er
            # elif(self.selection == "M"):
            #     self.pick = self.em
            # self.prevPick = self.pick+self.cutoff



    def processFrame(self):

        Iline2 = int(self.apriltag.apriltags[0].center.x)
        #print "In process frame"

        #Accessing the data of the april tag is below
        #print self.apriltag.apriltags[0].center.x

        new_frame = self.image

        new_frame = self.image 

        frame = cv2.subtract(self.sub_image, new_frame)
        Ib = frame[:,:,2]
        if self.debug:
            cv2.imshow("Ib", Ib)
            cv2.waitKey(30)

            Canny1 = cv2.Canny(Ib,self.threshold1,self.threshold2)
            cv2.imshow("Canny", Canny1)
            cv2.waitKey(30)

            cv2.imshow("sub", self.sub_image)
            cv2.waitKey(30)

            #fgmask = self.fgbg.apply(new_frame)
            #cv2.imshow('fgmask', fgmask)

            fgmask = self.fgbg.apply(new_frame[:,:,2])
            cv2.imshow('fgblue', fgmask)
            Ib = fgmask

            if not len(self.added_fgmask):
                self.added_fgmask = fgmask
            self.added_fgmask = self.added_fgmask + fgmask

            #cv2.imshow('added_fgblue', self.added_fgmask + fgmask)


            #background = self.fgbg.getBackgroundImage()
            #frame = cv2.subtract(background, new_frame[:,:,2])
            #Ib = frame
            #cv2.imshow('new', Ib)

        spec_lineL = Ib[:,Iline2-self.offset]
        spec_lineR = Ib[:,Iline2+self.offset]
        spec_lineM = Ib[:,Iline2]

        spec_area = Ib[:,Iline2-self.offset:Iline2+self.offset]
        spec_area[1:self.cutoff,:] = 0

        #edgeA = self.calc_grad2(spec_area)

        spec_lineL[1:self.cutoff] = 0
        spec_lineR[1:self.cutoff] = 0
        spec_lineM[1:self.cutoff] = 0

        edgeL = self.calc_grad(spec_lineL)
        edgeR = self.calc_grad(spec_lineR)
        edgeM = self.calc_grad(spec_lineM)

        el_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeL[idx]>0]
        er_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeR[idx]>0]
        em_ind = [idx for idx in range(self.cutoff, int(self.cyl_bottomP)) if edgeM[idx]>0]

        if(len(el_ind)>0):
            self.el = el_ind[-1]
        if (len(er_ind)>0):
            self.er = er_ind[-1]
        if (len(em_ind)>0):
            self.em = em_ind[-1]
            #print self.selection
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

        self.newPick = self.pick

        print self.newPick

        # spec_lineL = Ib[:,Iline2-self.offset]
        # spec_lineR = Ib[:,Iline2+self.offset]
        # spec_lineM = Ib[:,Iline2]
        # spec_lineLC = spec_lineL[self.cutoff:]
        # spec_lineRC = spec_lineR[self.cutoff:]
        # spec_lineMC = spec_lineM[self.cutoff:]

        # # edgeL = cv2.Canny(spec_lineLC,self.threshold1,self.threshold2)
        # # edgeR = cv2.Canny(spec_lineRC,self.threshold1,self.threshold2)
        # # edgeM = cv2.Canny(spec_lineMC,self.threshold1,self.threshold2)
        # #print spec_lineL
        # edgeL = self.calc_grad(spec_lineLC)
        # edgeR = self.calc_grad(spec_lineRC)
        # edgeM = self.calc_grad(spec_lineMC)

        # #print edgeL

        # edgeP = np.zeros(self.image_height-self.cutoff,)
        # el_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeL[idx]>0]
        # er_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeR[idx]>0]
        # em_ind = [idx for idx in range(self.image_height-self.cutoff) if edgeM[idx]>0]
        # if(len(el_ind)>0):
        #     self.el = el_ind[0]
        # if (len(er_ind)>0):
        #     self.er = er_ind[0]
        # if (len(em_ind)>0):
        #     self.em = em_ind[0]
        #     if self.debug:
        #         print self.selection
        # if(self.selection == "A"):
        #     check = [self.el,self.em,self.er]
        #     self.pick = np.max(check)
        # elif(self.selection == "L"):
        #     self.pick = self.el
        # elif(self.selection == "R"):
        #     self.pick = self.er
        # elif(self.selection == "M"):
        #     self.pick = self.em
        # edgeP[self.pick] = 255
        # self.newPick = self.pick+self.cutoff

        # print "\n Cutoff used:", self.cutoff  ::verified constant::

        #IMPLEMENT THE LOW PASS FILTER
        # pickbar = int(self.alpha*self.newPick + (1-self.alpha)*self.prevPick)


        ''' Implement median filter '''
        pickbar = int(self.newPick) #default case
        # self.pickbar_median_list.append(pickbar)
        # if len(self.pickbar_median_list) >= self.pickbar_median_set:
        #     pickbar = int(np.median(self.pickbar_median_list))
        #     self.pickbar_median_list = self.pickbar_median_list[1:] #pluck off first index




        # print "PickBar: "+str(pickbar)
        #print "CylinderB: "+str(self.cyl_bottomP)
        #print "Cutoff: "+str(self.cutoff)
        #print self.desHeightCM
        #Calculating the designated height line from the published topic
        desHeightP = int(self.cyl_bottomP-((self.desHeightCM*self.AT_Plength)/(self.AT_size*100)))
        #print desHeightP
        #accuracy circle
        cv2.circle(new_frame,(Iline2,desHeightP),5,(0,0,255),-1)

        #Level Circle
        cv2.circle(new_frame,(Iline2,pickbar),5,(0,255,0),-1)

        #print (Iline2, int(self.cyl_bottomP))
        cv2.line(new_frame, (Iline2, int(self.cyl_bottomP)), (Iline2, self.cutoff),(0,0,255))
        cv2.line(new_frame, (Iline2-self.offset, int(self.cyl_bottomP)), (Iline2-self.offset, self.cutoff),(0,0,255))
        cv2.line(new_frame, (Iline2+self.offset, int(self.cyl_bottomP)), (Iline2+self.offset, self.cutoff),(0,0,255))

        if self.debug:
            cv2.imshow("Updated Frame", new_frame)
            cv2.waitKey(30)

        #pull in
        # name = 'frame %04d.jpg'%self.count
        #cv2.imshow("frame",frame)
        # cv2.imwrite(name,new_frame)
        self.count+=1


        self.image_pub.publish(self.bridge.cv2_to_imgmsg(new_frame,encoding="bgr8"))
        #Build the message to publish
        msg = LDHeight()

        msg.header = self.header
        heightP = self.cyl_bottomP - pickbar
        heightCM = ((heightP*self.AT_size)/self.AT_Plength)*100
        if self.debug:
            print "Liquid Height :" +str(heightCM)+"cm"
        msg.h = heightCM
        self.pub.publish(msg)
        self.prevPick = self.newPick
        self.processed = True

    #sys.exit(0)

def main():
    debug = True #if true then print
    SelecMade = False
    parser = argparse.ArgumentParser(description='Please make a selection of which lines to use with low pass filter (L)eft, (R)ight (M)iddle (A)ll')
    parser.add_argument('-selection', help='Selection for the %(prog)s Low Pass Filter focus')
    args = parser.parse_known_args()
    #print args[0]
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
    #listener = tf.TransformListener()
    SelecMade = False


    lld = Liquid_Level_Detector(debug, selection)

    rospy.spin()


if __name__ == '__main__':
    main()

