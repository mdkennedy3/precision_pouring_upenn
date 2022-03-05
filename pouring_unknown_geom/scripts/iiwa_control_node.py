#!/usr/bin/env python2
import numpy as np, math
import rospy, rospkg
import sys, getopt
import threading
import actionlib
import copy

from std_msgs.msg import Float64

from pouring_unknown_geom.hybrid_control_pour import hybrid_cntrl_pouring
from pouring_unknown_geom.traj_gen import min_jerk_traj, trapezoidal_traj
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot  (this is right and produces h, dh/dt)
from pouring_control_pkg.msg import Hfilt #contains (header, hfilt(float64))
from sensor_msgs.msg import JointState
from pouring_unknown_geom.msg import PourModelPred, HybCntrlData, PourAngStateMsg

from pouring_unknown_geom.msg import PourControlAction, PourControlGoal, PourControlResult

from std_srvs.srv import *

class IiwaControlClass(object):
  def __init__(self):
    self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()

    self.iiwa_vel_pub = rospy.Publisher('command', Float64, queue_size=1, latch=False)

    self.pouring_control_pub = rospy.Publisher('pour_angle_state', PourAngStateMsg, queue_size=1, latch=False)
    self.single_joint_name = 'iiwa_joint_7'  #saywer: right_j6,  kuka: iiwa_joint_7

    self.hybrid_sub = rospy.Subscriber('Hybrid_controller_data',HybCntrlData,self.hyb_control_callback, queue_size=1)
    self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_callback, queue_size=1) #joint angle callback

    self.theta_start = float(rospy.get_param("~theta_start", 0.0))
    rospy.logwarn("theta start is: %f"%self.theta_start)

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
    omega = req.cmd_omega
    self.iiwa_vel_pub.publish(omega)

  def joint_state_callback(self,req):

    self.thread_lock.acquire()
    set_theta_start = copy.deepcopy(self.set_theta_start)
    if self.set_theta_start: self.set_theta_start = False
    self.thread_lock.release()

    #retrieve the theta/omega of single joint
    theta = req.position[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    omega = req.velocity[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    #now publish this for the controller

    if set_theta_start:
      self.theta_start = theta

    pour_msg = PourAngStateMsg()
    pour_msg.header = req.header
    pour_msg.theta = theta - self.theta_start
    pour_msg.omega = omega
    self.pouring_control_pub.publish(pour_msg)
    #This triggers the control in main node

def main():
  rospy.init_node('publish_sawyer_control')
  cls_obj = IiwaControlClass()
  rospy.spin()

if __name__ == '__main__':
  main()
