#!/usr/bin/env python2
import numpy as np, math, copy
import rospy
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot

#bring in messages for scale or height
#from usb_scale.msg import Scale

from liquid_level_detection.msg import LDHeight
from pouring_control_pkg.msg import Hfilt

from std_msgs.msg import Float64

class Measuring_Class(object):
    """docstring for Measuring_Class"""
    def __init__(self, scale_or_vision):
        self.scale_or_vision = scale_or_vision
        self.counter = 1
        self.mh_pub = rospy.Publisher('mh_mh_dot', MeasMsg, queue_size=1, latch=False)

        self.pub_raw_h_ml = rospy.Publisher('raw_h_ml', Float64, queue_size=1, latch=False)
        self.pub_raw_h_clust_ml = rospy.Publisher('raw_h_clust_ml', Float64, queue_size=1, latch=False)
        self.mh_list = []
        self.time_list = []

        #lowpass filter scale weight
        # self.alpha = 0.5 #was 0.5
        # self.alpha = 0.75 #was 0.5
        self.alpha = 0.8 #was 0.5
        if rospy.has_param('mh_low_pass_filter_constant'):
            self.alpha = float(rospy.get_param("mh_low_pass_filter_constant"))

        self.Area = 41.66 #cm**2
        if rospy.has_param('recieving_container_area'):
            self.Area = float(rospy.get_param("recieving_container_area"))

        rospy.logwarn("Receiving container area %g", self.Area)

        # self.alpha = 0.95 #was 0.5
        self.mh_prev = []

        self.h_median = []
        self.h_med_lp_prev = []
        self.h_med_lp_alpha = 0.2
        self.h_filt_pub = rospy.Publisher('h_filtered', Hfilt, queue_size=1, latch=False)


    def mh_callback(self,req):
        if self.scale_or_vision in 'scale':
            if req.units not in 'g': print "CHANGE UNITS TO GRAMS"
            if type(self.mh_prev) == list:
                self.mh = req.weight
                self.mh_prev = copy.copy(self.mh)
            else:
                self.mh = self.alpha*req.weight + (1-self.alpha)*self.mh_prev
                self.mh_prev = copy.copy(self.mh)

            self.t_curr = req.header.stamp.to_sec()
            print "current time: ", self.t_curr
        elif self.scale_or_vision in 'vision':
            '''FIX FOR VISION '''

            self.t_curr = req.header.stamp.to_sec()

            #this is to handle the fact that for the low pass filter it is bad for the requested height to start so high
            h_req = req.h_clust #req.h
            if h_req > 8. or h_req < 0.:
                try:
                    h_req = self.h_req
                except:
                    h_req = 0
            self.h_req = h_req #store for thresh
            self.mh = h_req * self.Area  #want units in ml


            try:
                if len(self.h_med_lp_prev) == 0:
                    self.mh_prev = self.mh
            except:
                pass
            self.mh = self.h_med_lp_alpha * self.mh + (1. - self.h_med_lp_alpha)*self.mh_prev
            self.mh_prev = copy.copy(self.mh)



        else:
            print "need either scale or vision"

        self.publish_mh_mh_dot(header=req.header) #now publish these values

    def publish_mh_mh_dot(self, header=None):

        if len(self.mh_list) > 2:
            self.mh_list = self.mh_list[1:]
            self.time_list = self.time_list[1:] #pop off the front value
        self.mh_list.append(self.mh)
        self.time_list.append(self.t_curr)
        #calculate the first derivative
        if len(self.mh_list) > 1:
            #calc derivative from last three
            if self.time_list[-1] - self.time_list[0] > 0.001:
                self.mh_dot = (self.mh_list[-1] - self.mh_list[0] )/(self.time_list[-1] - self.time_list[0])
        else:
            self.mh_dot = 0.001 #small positive value (just for first reading) (this node can run first)
        #print "mh_list",self.mh_list
        #print "time_list",self.time_list
        #print "mh_dot %f"%(self.mh_dot)
        #publish these values on internal msg type
        mh_msg = MeasMsg()
        if header:
            mh_msg.header = header
        mh_msg.mh = self.mh
        mh_msg.mh_dot = self.mh_dot
        self.mh_pub.publish(mh_msg)


    def h_callback(self,req):
        #this is a callback to filter the height callback message

        #this is to handle the fact that for the low pass filter it is bad for the requested height to start so high
        h_req = req.h_clust #req.h
        if h_req > 8. or h_req < 0.:
            try:
                h_req = self.h_fix
            except:
                h_req = 0
        else:
            self.mh_req = h_req #store for thresh

        self.h_fix = h_req * self.Area  #want units in ml


        #perform low pass filter
        try:
            if len(self.h_med_lp_prev) == 0:
                self.h_med_lp_prev = self.h_fix
        except:
            pass
        self.h_fix = self.h_med_lp_alpha * self.h_fix + (1. - self.h_med_lp_alpha)*self.h_med_lp_prev
        self.h_med_lp_prev = copy.copy(self.h_fix)


        self.publish_filtered_h(self.h_fix, req.header)

        h_raw_clust_ml = Float64(); h_raw_clust_ml.data = self.Area*req.h_clust
        h_raw_ml = Float64(); h_raw_ml.data = self.Area*req.h


        self.pub_raw_h_clust_ml.publish(h_raw_clust_ml)
        self.pub_raw_h_ml.publish(h_raw_ml)


    def publish_filtered_h(self, h, header):

        msg = Hfilt()
        msg.hfilt = h
        msg.header = header

        self.h_filt_pub.publish(msg)


def main():
    rospy.init_node('meas_state_node')

    scale_or_vision = 'vision'

    meas_cls_obj = Measuring_Class(scale_or_vision)

    '''
    if scale_or_vision in 'scale':
        mh_sub = rospy.Subscriber('/scale_node/scale', Scale, meas_cls_obj.mh_callback)
    elif scale_or_vision in 'vision':
        mh_sub = rospy.Subscriber('LDHeight', LDHeight  ,meas_cls_obj.mh_callback)
    '''
    mh_sub = rospy.Subscriber('LDHeight', LDHeight  ,meas_cls_obj.mh_callback)

    h_filtered = rospy.Subscriber('LDHeight', LDHeight  ,meas_cls_obj.h_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
