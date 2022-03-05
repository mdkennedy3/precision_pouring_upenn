#!/usr/bin/env python
import rospy
import numpy as np
import rospkg
import os
import json
import threading
import cPickle as pickle
import copy
from pouring_unknown_geom.gauss_newton_opt import opt_srv_gp_compatible
from pouring_msgs.msg import PourModelPred, PouringMsg, SelectedGPModels
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse


class Model_selection(object):
  """docstring for Model_selection"""
  def __init__(self):
    self.model_pub = rospy.Publisher('Pour_Model_opt', PourModelPred, queue_size=1, latch=True)  #publish the model
    self.volume_solution_mode = rospy.get_param("~volume_solution_mode", "param")
    rospy.loginfo("Using the {0} method to find volume".format(self.volume_solution_mode))
    self.thread_lock = threading.Lock() # self.thread_lock.acquire(); self.thread_lock.release()
    self.setup_variables()

  def setup_variables(self, opt_num_iter=1000, opt_residual_threshold=1e-2, dom_win=[0,2.4], coef_type="power"):
    self.initial_angle = None  #this is required to shift all GP volume models down for accurate comparison
    #Pouring data used for fitting curve and updating model
    self.thread_lock.acquire()
    self.pouring_data = {'v':[], 'dv':[], 'ang':[], 'stamp': []};# self.pouring_data['dv'].append(0.0); self.pouring_data['ang'].append(0.0)
    self.collected_gp_selected_models = False
    self.thread_lock.release()
    #For obtaining gp selected models
    self.gp_selected_models_data = None
    param_coef_order = rospy.get_param("volume_profile_poly_degree",9) #This should be set...
    self.curr_coef = np.polynomial.polynomial.Polynomial(np.zeros([param_coef_order]),domain=dom_win,window=dom_win)
    self.coef_type = coef_type#"legendre" or "power"
    #Make optimizer class: #opt_num_iter: max number of iterations; #opt_residual_threshold: max change in coefficients to show convergence (break convergence loop)
    self.opt_cls = opt_srv_gp_compatible.Optimizer_class(opt_max_iter=opt_num_iter, domain_win=dom_win) #num_iter is max number of iterations to run per opt
    #subscribers
    self.sub = rospy.Subscriber('pouring_msg',PouringMsg,self.pouring_msg_callback, queue_size=1) # rospy.loginfo("subscribing")
    self.sub_gp_mod = rospy.Subscriber('gp_selected_models', SelectedGPModels, self.gp_selected_models_callback ,queue_size=1)
    self.timer = rospy.Timer(rospy.Duration(0.01), self.optimization_callback) #run this at 100hz (will be slower with opt hanging)



  def reset_callback(self, req):
    rospy.logwarn("model_selection_node Reset requested")
    self.setup_variables()
    return TriggerResponse(True,"Reset complete")

  def gp_selected_models_callback(self,data):
    self.thread_lock.acquire()
    local_bool = self.collected_gp_selected_models
    self.thread_lock.release()
    rospy.loginfo("collected gp selected models")
    if local_bool == False:
      print "collecting gp selected models in model select node"
      self.thread_lock.acquire()
      self.gp_selected_models_data = data
      self.collected_gp_selected_models = True
      self.thread_lock.release()
    else:
      pass

  def pouring_msg_callback(self,req):
    self.thread_lock.acquire()
    gp_models_obtained_local_bool = self.collected_gp_selected_models
    self.thread_lock.release()
    print "inside pouring msg callback, and collected gp: ", gp_models_obtained_local_bool
    # if gp_models_obtained_local_bool:
    if True:
      #this function takes in the msg PouringMsg which consists of a) header b) float64 dvolume (derivative volume) c) float64 angle
      rospy.loginfo("inside pouring msg callback")
      #Append this to the running list of dv and ang:
      self.thread_lock.acquire()
      self.pouring_data['v'].append(req.volume)
      self.pouring_data['dv'].append(req.dvolume)
      self.pouring_data['ang'].append(req.angle)
      self.pouring_data['stamp'].append(req.header.stamp)
      self.thread_lock.release()
      if not self.initial_angle:
        #hasn't been filled yet
        self.initial_angle = req.angle
        self.shift_gp_model_curves(init_angle=self.initial_angle)
        rospy.loginfo("Shifted gp models start volume")
    else:
      #no models have been obtained, they should be obtained even if soley parametric method is used...
      pass

  def shift_gp_model_curves(self,init_angle=None):
    #In this function, shift all of the volume curves down based on starting angle
    self.thread_lock.acquire()
    local_data_models = self.gp_selected_models_data
    self.thread_lock.release()
    for idx in range(len(local_data_models.gp_volume_models)):
      print "len of th list: ",len(local_data_models.gp_volume_models[idx].th)
      th_diff = [(th-init_angle)**2 for th in local_data_models.gp_volume_models[idx].th]
      idx_min = np.argmin(th_diff)
      V_zero = local_data_models.gp_volume_models[idx].V[idx_min]
      new_V_list = [vcurr - V_zero for vcurr in local_data_models.gp_volume_models[idx].V]
      local_data_models.gp_volume_models[idx].V = new_V_list
    self.thread_lock.acquire()
    self.gp_selected_models_data = local_data_models
    self.thread_lock.release()

  def optimization_callback(self, event):
    # print "len of pour data", len(self.pouring_data['ang']),"\n gp selected models", self.collected_gp_selected_models
    if len(self.pouring_data['ang']) > 0:
      header = Header()
      header.stamp = self.pouring_data['stamp'][-1]
      self.optimizer(header=header)

  def generate_subscribers(self):
    #get the reported dV, angle for optimization
    # self.sub = rospy.Subscriber('pouring_msg',PouringMsg,self.pouring_msg_callback, queue_size=1) # rospy.loginfo("subscribing")
    # self.sub_gp_mod = rospy.Subscriber('gp_selected_models', SelectedGPModels, self.gp_selected_models_callback ,queue_size=1)
    # self.timer = rospy.Timer(rospy.Duration(0.01), self.optimization_callback) #run this at 100hz (will be slower with opt hanging)
    self.setup_variables()
    self.reset_srv = rospy.Service('reset_model_selection_node', Trigger, self.reset_callback)
    rospy.spin()

  def optimizer(self, header=None):
    #This is running on a seperate thread from the callbacks
    #Note: keep the opt iter down so updated info can reach quickly
    if len(self.pouring_data['ang']) > 1:
      rospy.loginfo('Inside optimizer script')
      self.thread_lock.acquire()
      pouring_data = copy.copy(self.pouring_data)
      gp_volume_models = self.gp_selected_models_data.gp_volume_models
      lowest_model_rmse = self.gp_selected_models_data.lowest_model_rmse
      curr_coef = self.curr_coef #get the current coefficient estimate (this is stored and used as a seed value each round)
      self.thread_lock.release()
      #Extract vars
      v_data = copy.copy(pouring_data['v'])
      dv_data = copy.copy(pouring_data['dv'])
      theta_data = copy.copy(pouring_data['ang'])
      #Optimize (currently expects coef as legendre)
      curr_coef_upd, residuals_avg, residuals_std = self.opt_cls.update_coeff(curr_coef=curr_coef, vol_data=v_data,theta_data=theta_data)
      #Save
      self.thread_lock.acquire()
      self.curr_coef = curr_coef_upd
      self.thread_lock.release()
      if self.coef_type in "legendre":
        #Convert to legendre if desired
        domain =curr_coef_upd.domain
        curr_coef_upd = np.polynomial.legendre.Legendre.cast(curr_coef_upd, window=domain, domain=domain)
      #Unpack:
      coeff = curr_coef_upd.coef.tolist()
      domain = curr_coef_upd.domain.tolist()
      best_residual_avg =residuals_avg
      best_residual_std = residuals_std
      #Publish the current optimal model
      pour_model_msg = PourModelPred()
      pour_model_msg.header = header
      pour_model_msg.coef_opt = coeff
      pour_model_msg.domain = domain
      pour_model_msg.residual_avg = best_residual_avg
      pour_model_msg.residual_std = best_residual_std
      if self.coef_type in "legendre":
        pour_model_msg.type = 'legendre'
      else:
        pour_model_msg.type = 'power'
      #add GP model
      pour_model_msg.gp_volume_models = gp_volume_models
      pour_model_msg.lowest_model_rmse = lowest_model_rmse
      pour_model_msg.volume_solution_mode = self.volume_solution_mode #options are "param"; "semiparam"; "nonparam"
      rospy.loginfo("Publishing pour model")
      self.model_pub.publish(pour_model_msg)
      rospy.loginfo("Model published")
    else:
      rospy.loginfo('No (dv,ang) data has been recieved received')

def main():
  #This node toggles between classification and optimization and publishes the current best estimate of the container volume curve model
  rospy.init_node('model_selection_node')
  cls_obj = Model_selection()
  print "inisde model selection node"
  cls_obj.generate_subscribers()

if __name__ == '__main__':
  main()
