#!/usr/bin/env python2
import numpy as np, math
import rospy
import matplotlib.pyplot as plt

class Volume_Kalman_Filter(object):
  '''
  This class allows volume estimation by combining measurements from scale and vision
  '''
  def __init__(self, state=None, covariance=None,state_dim=1, qscale=None, qvision=None, rval=None):
    
    if type(state)==type(None): self.state = np.matrix(np.zeros([state_dim,1]))
    else: self.state=np.matrix(state)

    if type(covariance) == type(None): self.cov = np.matrix(10.0*np.diag(np.ones(state_dim)))
    else: self.cov = np.matrix(covariance)

    if type(qscale) == type(None): self.qscale = 0.01
    else:  self.qscale = qscale

    if type(qvision) == type(None): self.qvision = 0.01
    else: self.qvision = qvision

    if type(rval)==type(None): self.R = np.matrix(0.01*np.diag(np.ones(self.state.size)))
    else: self.R = np.matrix(rval*np.diag(np.ones(self.state.size)))

    #setup other variables
    self.A = np.matrix(np.diag(np.ones(state_dim)))
    # self.C = np.matrix(np.diag(np.ones(state_dim))) #assumes identity mapping from meas to state (state is fully observable)
    self.K = None
    self.C = None
    self.z = None

  def process_update(self): 
    '''
    This updates the state and covariance
    x = Ax + Bu
    S = ASA' + R
    '''
    self.state = self.A*self.state
    self.cov = self.A*self.cov*self.A.T + self.R


  def measurement_update(self, vision_volume=None, scale_volume=None):
    '''
    This update the kalman gain, for n dim state, 
    [z_vis; z_scale]_{2nx1} = C_{2nxn} x_{nx1} + [qv_n; qs_n]; Hence C = [1;1]  when both are present and c=[1] when only one is present
    K = S C' (C S C' + Q)^{-1}
    '''
    if type(vision_volume) != type(None) and type(scale_volume) != type(None):
      #both are available
      self.C = np.matrix(np.ones([2,1]))
      self.Q = np.matrix(np.diag([self.qvision,self.qscale]))
      self.z = np.matrix([vision_volume, scale_volume]).T
    elif (type(vision_volume) != type(None)) and type(scale_volume) == type(None):
      #vision is available but scale is not
      self.C = np.matrix(np.ones([1,1]))
      self.Q = np.matrix([self.qvision])
      self.z = np.matrix([vision_volume])
    elif (type(vision_volume) == type(None)) and type(scale_volume) != type(None):
      #scale is available but vision is not
      self.C = np.matrix(np.ones([1,1]))
      self.Q = np.matrix([self.qscale])
      self.z = np.matrix([scale_volume])
    else:
      #Neither are available
      rospy.logwarn("Neither measurement available")
      return
    #calculate the kalman gain
    self.K = self.cov*self.C.T * np.linalg.inv(self.C*self.cov*self.C.T + self.Q)
    '''
    Measurement update
    x = x + K (z - Cx)
    S = (I - K C) S
    '''
    self.state = self.state + self.K*(self.z - self.C*self.state)
    self.cov = (1 - self.K*self.C)*self.cov




def main():
  
  x_est = [0.0]
  S = [10.0]
  qscale = 0.01
  qvision = 0.03
  rval = 0.01

  kf_obj = Volume_Kalman_Filter(state=x_est[0], covariance=S[0],state_dim=1,  qscale=qscale, qvision=qvision, rval=rval)

  t1= np.linspace(0.0,10,11)
  x1 = t1**3
  t2 = 10*np.ones([1,10])[0]
  x2 = t2**3
  t = np.linspace(0.0,20,21) #'time'
  # x = t**3 #true state
  x = np.hstack([x1,x2])

  x_est = []
  S_est = []
  t_est = []
  for idx in range(len(x)):
    #process update
    kf_obj.process_update()
    #measurement update
    if idx%1 == 0:
      #this makes every other measurement visible
      if idx%3==0:
        z_scl = x[idx] + np.random.rand()*qscale
        kf_obj.measurement_update(scale_volume=z_scl)
        print "meas scale only"
      elif idx%4==0:
        z_vis = x[idx] + np.random.rand()*qvision
        kf_obj.measurement_update(vision_volume=z_vis)
        print "meas vision only"
      else:
        z_vis = x[idx] + np.random.rand()*qvision
        z_scl = x[idx] + np.random.rand()*qscale
        kf_obj.measurement_update(vision_volume=z_vis, scale_volume=z_scl)
        print "meas both"
    #save output
    x_est.append(kf_obj.state.item(0))
    S_est.append(kf_obj.cov.item(0))
    t_est.append(t[idx])



  print "x_est", len(x_est), len(t_est)
  print "\n",x_est, t_est
  #Now plot and visualize the output
  plt.scatter(t,x,c='b',label="true state")
  plt.scatter(t_est,x_est,c='r',label="est state")
  plt.legend()
  plt.show()



if __name__ == '__main__':
  main()