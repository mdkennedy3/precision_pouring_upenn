#!/usr/bin/env python
from __future__ import division
import numpy as np, math
import rospy, rospkg
from pouring_unknown_geom.sawyer_control import sawyer_cntrl_cls
from pouring_unknown_geom.msg import ScaleMsg
import threading


class VerificationClass(object):
  def __init__(self, final_theta_angle=0.4,omega_rate=0.15 ):
    self.sawyer_cntrl = sawyer_cntrl_cls.Sawyer_Control() #expects inputs .cmd_joints(input_dictionary, control_type_str); where inpt_dict = {'right_j6':0.3, ...} and cntrl_type_str = 'torque' or 'velocity' or 'position'
    self.single_joint_name = "right_j6"

    self.start_angle = None

    self.final_theta_angle = final_theta_angle
    self.omega_rate = omega_rate
  
    self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()
    self.weight_tare = None
    self.weight = None
    
    #self.data_pub = rospy.Publisher('verification_pub',,queue_size=1,latch=False)

  def angle_diff(self,th1,th2):
      def rotmat(th): return np.matrix([[np.cos(th),np.sin(th)],[-np.sin(th),np.cos(th)]])
      R1 = rotmat(th1)
      R2 = rotmat(th2)
      Rd = R1*R2.T
      th_diff = np.arctan2(Rd[0,1],Rd[0,0])
      return th_diff

  def joint_state_callback(self,req):
    #retrieve the theta/omega of single joint
    theta = req.position[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    omega = req.velocity[req.name.index(self.single_joint_name)] #locking is occuring on subs_var_mh (to avoid memory accessing issues)

    if type(self.start_angle) == type(None):
        self.start_angle = theta #only populate in the beginning
    
    theta_diff = self.angle_diff(theta,self.start_angle)
    
    if np.abs(theta_diff) < np.abs(self.final_theta_angle):
        #case that less than final theta, keep going
        cntrl_dict = {self.single_joint_name:self.omega_rate}
        cntrl_type = 'velocity'
        self.sawyer_cntrl_cls(cntrl_dict, cntrl_type)
    else:
        rospy.loginfo("maximum angle reached")


  def scale_callback(self,req):

      raw_weight = req.mass
      if req.units in ['g','grams']:
          raw_weight = raw_weight*1e-3 #convert to kg

      #automatic taring
      if type(self.weight_tare) == type(None):
          self.weight_tare = weight

      self.thread_lock.acquire()      
      self.weight = raw_weight - self.weight_tare
      self.thread_lock.release()


def main():

  #this script makes constant pours at specified velocities from the start angle to final angle
  final_theta_angle = 60.*np.pi/180. #radians
  omega_rate = 0.2
  rospy.init_node('sawyer_vol_time_delay_verification_node')
   
  cls_obj = VerificationClass(final_theta_angle=final_theta_angle, omega_rate=omega_rate)

  rospy.Subscriber('joint_states',JointState, cls_obj.theta_callback, queue_size=1)
  rospy.Subscriber('arduino_scale',ScaleMsg, cls_obj.scale_callback,queue_size=1)
  rospy.spin()


if __name__ == '__main__':
  main()

