#!/usr/bin/env python
import rospy
import numpy as np
# from pouring_unknown_geom.msg import PourModelPred, PouringMsg
from pouring_msgs.msg import PourModelPred, PouringMsg
import threading

class VisModel(object):
  """docstring for VisModel"""
  def __init__(self):
    self.model_output_pub = rospy.Publisher('model_volume_output',PouringMsg,queue_size=1)
    self.thread_lock = threading.Lock()
    self.model_object = None

  def generate_subscribers(self):
    rospy.Subscriber('pouring_msg',PouringMsg,self.pouring_msg_callback) # rospy.loginfo("subscribing")
    rospy.Subscriber('Pour_Model_opt',PourModelPred,self.pouring_model_callback) # rospy.loginfo("subscribing")
    rospy.spin()


  def pouring_msg_callback(self,req):
    self.theta = req.angle
    # header = req.header
    # model_object_local = None
    # self.thread_lock.acquire()
    # model_object_local = self.model_object
    # self.thread_lock.release()
    # if model_object_local:
    #   # rospy.loginfo("MP: pouring_msg_callback")
    #   self.publish_model_prediction(header=header, theta=theta, model=model_object_local)


  def pouring_model_callback(self,req):
    model_object = req
    header = req.header
    theta = None
    self.thread_lock.acquire()
    theta = self.theta
    self.thread_lock.release()
    if theta:
      self.publish_model_prediction(header=header, theta=theta, model=model_object)

    # rospy.loginfo("MP: pouring_model_callback")





  def publish_model_prediction(self,header=None,theta=None,model=None):
    model_pred = PouringMsg()
    if header:
      model_pred.header = header
      if theta:
        model_pred.angle = theta
        if model:
          if model.type in "legendre":
            #1. Calculate V given angle, domain, window, coef
            sk = np.polynomial.legendre.Legendre(model.coef_opt, domain=model.domain, window=model.domain)
            V = np.polynomial.legendre.legval(theta, sk.coef)
            #2. Calculate dV given differentiated series object and theta
            sk_d = sk.deriv(1)
            dV = np.polynomial.legendre.legval(theta, sk_d.coef)
          if model.type in "power":
            V = np.polyval(model.coef_opt,theta)
            dV = np.polyval(np.polyder(model.coef_opt),theta)
          #3. Assign values to model prediction:
          model_pred.volume = V
          model_pred.dvolume = dV
          #4. Publish
          # rospy.loginfo("publishing model prediction!!")
          self.model_output_pub.publish(model_pred)



def main():
  rospy.init_node('vis_mod_pred')
  """This script publishes the current estimate of the model for the given joint angle, model at the current time"""

  cls_obj = VisModel()

  cls_obj.generate_subscribers()



if __name__ == '__main__':
  main()
