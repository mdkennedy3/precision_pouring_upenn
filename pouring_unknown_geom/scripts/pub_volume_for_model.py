#!/usr/bin/env python
import rospy
import numpy as np
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import animation
from scipy.optimize import minimize
import cPickle as pickle
import json
import os
import sys
import argparse
import rospkg


from pouring_unknown_geom.container_classification import generate_classification

# from pouring_unknown_geom.msg import PouringMsg, PourModelPred, PourVolAng
from pouring_msgs.msg import PouringMsg, PourModelPred, PourVolAng

from std_srvs.srv import Trigger, TriggerResponse

class PubVolumeClass(object):
  """docstring for PubVolumeClass"""
  def __init__(self):

    self.pouring_msg_pub = rospy.Publisher('pouring_msg',PouringMsg,queue_size=1,latch=False)

    self.setup_variables()

  def setup_variables(self):

    #rospy.loginfo("Setting up variables")

    self.start_angle = None
    if rospy.has_param('start_j6_angle'):
      self.start_angle = float(rospy.get_param("start_j6_angle"))

    #Obtain time delay
    self.pour_time_delay = 2.0
    if rospy.has_param('pour_time_delay'):
      self.pour_time_delay = float(rospy.get_param("pour_time_delay"))

    #running list of parameters
    self.volume = [] #volume data
    self.angles = [] #angle data
    self.dvolume = [] #derivative volume data
    self.dangles = [] #angle corresponding to differentiated volume
    self.stamps = []
    self.seq = 0
    self.pouring_started_bool = False

    self.start_volume = None
    self.latest_volume = None

    self.ml_vol_start_opt = 5.0 #change in 1ml starts tracking

    self.theta_trigger_delta = 0.1 * np.pi/180

    self.last_pub_theta_val = None
    self.last_pub_vol_val = None

  def test_with_example_data(self):
    #This script tests the published model with example data (storing the reported model for plotting once all data has been published)
    #1. Load the data, and put in the form (th,dV)
    rospack = rospkg.RosPack()
    gen_cls =  generate_classification.Container_classifier_class()
    file_num = '001'
    curr_dir = rospack.get_path("pouring_unknown_geom")+'/data/'+file_num
    for fn in os.listdir(curr_dir):
      if fn != 'INFO.json' and fn[-1] != "~":
        curr_data = json.load(open(curr_dir + '/' + fn))
        MAX_VOLUME = json.load(open( curr_dir + '/INFO.json'))["max_volume"]
        # angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc,  last_ang = generate_classification.filter_volume_data(curr_data, MAX_VOLUME, last_ang=0, angles_dec=[], volumes_dec=[])
        angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc, angle_derv_dec, volume_derv_dec,  last_ang = gen_cls.filter_volume_data(curr_data, MAX_VOLUME, last_ang=0, angles_dec=[], volumes_dec=[])
        break
    # th,dV = calculate_dV_dth(curr_data)
    th,dV = angle_derv_inc, volume_derv_inc


    #Now make this work with th,V_inc


    #2. loop through the data (with rate cntrl), publishing the list of [th],[dV] on the appropriate msg and listening
    print "length: ", len(dV), len(th)
    for idx in range(0,len(dV)):
      #calculate the derivative (discrete deriavtive to model taking in height directly)
      #add and publish msg
      if np.abs(dV[idx]) > 0.01:
        #Ensure that the change in volume is significant to avoid plotting these data
        self.publish_model(th[idx],dV[idx])
        print "printed, ",idx
      #pause
      rospy.sleep(0.2) #sleep for 0.2 seconds


  def publish_model(self,th=0,dV=0,V=0,header=None):
    pour_msg = PouringMsg()
    # print "recieved: ", th,dV

    pour_msg.volume = V
    pour_msg.dvolume = dV
    pour_msg.angle = th


    if header:
      pour_msg.header = header
    else:
      self.seq += 1
      pour_msg.header.seq = int(self.seq)
      pour_msg.header.stamp = rospy.get_rostime()

    #now publish
    # print "pour msg: ", pour_msg
    self.pouring_msg_pub.publish(pour_msg)


  def calc_dv_dth_th(self,v_init=[], v_final=[],th_init=[], th_final=[]):
    dV = (v_final - v_init)/(th_final-th_init)
    #th = 0.5*(th_init + th_final)
    th = th_final
    return th,dV


  def dv_angle_callback(self,req):
    # time = []
    angle = req.angle
    volume = req.volume
    self.stamps.append(req.header.stamp)
    # print "in callback loop"
    self.volume.append(volume) #volume data
    self.angles.append( angle) #angle data

    if not self.start_volume:
      self.start_volume = volume
    self.latest_volume = volume


    if len(self.angles) > 1:
      #rospy.loginfo("len of angles is greater than one")
      #differentiate volume

      #Find index of the angle at the time delay before the current measurement
      t_curr = self.stamps[-1].to_sec()
      time_idx = -1
      #dt_ = 0.0
      while (time_idx > -len(self.stamps)+1 ) and np.abs(t_curr - self.stamps[time_idx].to_sec()) < self.pour_time_delay:
        time_idx -= 1
      #dt = t_curr - self.stamps[time_idx].to_sec()
      '''
      if self.start_angle:
        th2 = self.angles[time_idx] - self.start_angle
      else:
        th2 = self.angles[time_idx] - self.angles[0]
      '''
      th2 = self.angles[time_idx]
      v2 = self.volume[-1]

      if not self.last_pub_theta_val:
        self.last_pub_theta_val = th2
      th1 = self.last_pub_theta_val

      if not self.last_pub_vol_val:
        self.last_pub_vol_val = volume
      v1 = self.last_pub_vol_val

      if np.abs(th1 -th2)>self.theta_trigger_delta:
        #rospy.loginfo("th diff triggered")
        th,dV = self.calc_dv_dth_th(v_init=v1, v_final=v2,th_init=th1, th_final=th2)

        if np.abs(self.latest_volume - self.start_volume) > self.ml_vol_start_opt:
          #rospy.loginfo("volume diff triggered")
          self.pouring_started_bool = True

        if self.pouring_started_bool and dV>0.0:

          self.publish_model(th=th,dV=dV, V=volume, header=req.header)
          self.last_pub_theta_val = th2
          self.last_pub_vol_val = volume

  def reset_callback(self, req):

    rospy.logwarn("reset_pour_vol_ang Reset requested")
    self.setup_variables()
    return TriggerResponse(True,"Reset complete")

  def height_angle_to_dv_th(self):
    #This function:
    #1. Runs the subscribers for listening to height and angle messages, then
    #2. Calculates the dV/dth, th for measurements  (if np.abs(dV)<thresh then dont send) (however there may be noise so dont inforce strictly positive)
    print "ready to play"
    self.sub = rospy.Subscriber('pour_vol_ang',PourVolAng,self.dv_angle_callback)  #check this with cpp code that generates message
    self.reset_srv = rospy.Service('reset_pour_vol_ang', Trigger, self.reset_callback)

    rospy.spin()

def main():
  # parser = argparse.ArgumentParser(description="Analyze and plot clusters in pouring data")
  # parser.add_argument("-e", "--example_data", action="store_true")
  # args = parser.parse_args()

  rospy.init_node('pub_volume_for_model_node')

  cls_obj = PubVolumeClass()


  #The following are not both run, choose whether to load old data are listen for new data
  # if args.example_data:
  #   #Test with saved pour first (publish)
  # cls_obj.test_with_example_data()
  # else:
  #   #Take in height and angle data and publish neccessary msg for model prediction node


  print "about to execute the cls handle"
  cls_obj.height_angle_to_dv_th()


if __name__ == '__main__':
  main()
