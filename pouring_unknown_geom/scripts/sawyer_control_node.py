#!/usr/bin/env python2
import numpy as np, math
import rospy, rospkg
import sys, getopt
from intera_core_msgs.msg import JointCommand  #to publish types of joint commands
import threading
import actionlib

import copy
from pouring_unknown_geom.sawyer_control import sawyer_cntrl_cls
# from pouring_control_pkg.sawyer_control import sawyer_cntrl_cls
from pouring_unknown_geom.hybrid_control_pour import hybrid_cntrl_pouring
from pouring_unknown_geom.traj_gen import min_jerk_traj, trapezoidal_traj
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot  (this is right and produces h, dh/dt)
from pouring_control_pkg.msg import Hfilt #contains (header, hfilt(float64))
from sensor_msgs.msg import JointState
from pouring_unknown_geom.msg import PourModelPred, HybCntrlData, PourAngStateMsg

from pouring_unknown_geom.msg import PourControlAction, PourControlGoal, PourControlResult

from std_msgs.msg import Empty
from std_srvs.srv import *


class SawyerControlClass(object):
  def __init__(self):
    self.thread_lock = threading.Lock()
    self.sawyer_cntrl_cls = sawyer_cntrl_cls.Sawyer_Control() #velocity control, has a sawyer_cntrl_cls.cmd_vel(self,omega, joints)
    self.pouring_control_pub = rospy.Publisher('pour_angle_state', PourAngStateMsg, queue_size=1, latch=False)
    self.single_joint_name = 'right_j6'  #saywer: right_j6,  kuka: iiwa_joint_7
    #self.start_theta =  float(rospy.get_param("~theta_start", 0.0)) 
    self.start_theta =  float(rospy.get_param("~theta_start", -1.158375)) 
    rospy.logwarn("theta start is: %f"%self.start_theta)
    self.done = False
    self.started = False
    
    self.set_theta_start = False
    self.theta_start_tigger = rospy.Service('reset_theta_start', Trigger, self.pouring_frames_service_callback)


  def pouring_frames_service_callback(self,req):

    self.thread_lock.acquire()
    self.set_theta_start = True
    self.thread_lock.release()

    resp = TriggerResponse()
    resp.success = True
    resp.message = "Set theta start"
    return resp



  def hyb_control_callback(self,req):
    omega= req.cmd_omega
    self.sawyer_cntrl_cls.cmd_joints({'right_j6':omega},'velocity')

  def joint_state_callback(self,req):

    self.thread_lock.acquire()
    set_theta_start = copy.deepcopy(self.set_theta_start)
    if self.set_theta_start: self.set_theta_start = False
    self.thread_lock.release()
    #retrieve the theta/omega of single joint
    theta = req.position[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    omega = req.velocity[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)

    if set_theta_start:
      #self.start_theta = theta
      #self.start_theta = -1.158375 #theta
      print "thstart: ", self.start_theta
      rospy.logwarn("in sawyer_control_node theta start is reset")
    #now publish this for the controller
    pour_msg = PourAngStateMsg()
    pour_msg.header = req.header
    pour_msg.theta = theta - self.start_theta
    pour_msg.omega = omega
    self.pouring_control_pub.publish(pour_msg)
    '''
    if theta - self.start_theta > 2*np.pi/3.:
      self.done = True
    #This triggers the control in main node
    '''
  def begin(self, msg):
    self.started = True



def main():
  rospy.init_node('publish_sawyer_control')
  cls_obj = SawyerControlClass()
  rospy.Subscriber('Hybrid_controller_data',HybCntrlData,cls_obj.hyb_control_callback, queue_size=1)
  rospy.Subscriber('joint_states',JointState, cls_obj.joint_state_callback, queue_size=1) #joint angle callback
  cls_obj.started = True #this can be added in later
  #rospy.Subscriber('/setup_finished', Empty, cls_obj.begin, queue_size=1)
  rospy.spin()


if __name__ == '__main__':
  main()
