#!/usr/bin/env python
import rospy
import numpy as np
import rospkg
import os
import json
import threading
import cPickle as pickle
import copy
# from pouring_unknown_geom.container_classification import generate_classification
from pouring_unknown_geom.gauss_newton_opt import opt_srv
from pouring_msgs.msg import PourModelPred, PouringMsg
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse


class Model_selection(object):
  """docstring for Model_selection"""
  def __init__(self, filename, est_cont_max_vol=174):
    #Load in the classifier model data
    self.rospack = rospkg.RosPack()
    self.data_path = self.rospack.get_path("pouring_unknown_geom") + '/classification/' + filename  #data path

    self.data = None
    with open(self.data_path, 'rb') as fp:
      self.data = pickle.load(fp)

    #data = None #free up memory

    self.est_cont_max_vol = est_cont_max_vol #this is in milliliters and is the maximum volume the container can hold

    #Make publisher
    self.model_pub = rospy.Publisher('Pour_Model_opt', PourModelPred, queue_size=1)  #publish the model

    #Set the estimated container size (not starting with 1ml containers, hence multiple by estimated volume)
    self.container_estimated_initial_volume = 150.0  #this can be close to the supposed pour (ballpark)
    if rospy.has_param('container_estimated_initial_volume'):
      self.container_estimated_initial_volume = float(rospy.get_param("container_estimated_initial_volume"))

    #Subscribe to msg that provides (theta, dV)
    self.thread_lock = threading.Lock() # self.thread_lock.acquire(); self.thread_lock.release()

    self.setup_variables()

  def setup_variables(self):

    self.initial_angle = None

    self.classifier_data = copy.deepcopy(self.data['clusters'])

    #Now generate field for optimized coefficients:
    for idx in range(len(self.classifier_data['clusters_inc'])):
      # self.classifier_data['clusters_inc'][idx]["opt_coeff"] = self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"].deriv(1) #store the alpha_dv coefficients here
      #instead
      self.classifier_data['clusters_inc'][idx]["opt_coeff"] = self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"] #store the alpha_dv coefficients here

    #Make optimizer class:
    num_iter = 1000 #max number of iterations
    residual_threshold = 1e-2  #max change in coefficients to show convergence (break convergence loop)
    self.opt_cls = opt_srv.Optimizer_class(num_iter, residual_threshold, scaling_constant_low_pass_filt_value=0.5) #num_iter is max number of iterations to run per opt


    #Pouring data used for fitting curve and updating model
    self.thread_lock.acquire()
    self.pouring_data = {'v':[], 'dv':[], 'ang':[], 'stamp': []};# self.pouring_data['dv'].append(0.0); self.pouring_data['ang'].append(0.0)
    self.thread_lock.release()

  def pouring_msg_callback(self,req):
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
      self.shift_cluster_center_curves(init_angle=self.initial_angle)
      rospy.loginfo("Shifted cluster center")
      #After shifting the curves, re-initialize the starting optimal coefficients
      for idx in range(len(self.classifier_data['clusters_inc'])):
        #now scale each such that the final value (domain) is estimated volume of the container
        val_max_init= np.polynomial.legendre.legval(np.pi/2, self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"].coef) #volume reached at 90 deg
        new_val_max_scale = self.est_cont_max_vol/float(val_max_init)
        new_coef = self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"].coef*new_val_max_scale
        self.classifier_data['clusters_inc'][idx]["opt_coeff"] = self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"] #store the alpha_dv coefficients here
        # self.classifier_data['clusters_inc'][idx]["opt_coeff"].coef = new_coef



    #there is no reason to optimize unless new data has been recieved, and since classification is done apriori and classes are used only for a starting point and updates are done on all classes
    #then there is no need for seperate threads


  def optimization_callback(self, event):
    # print "optimization callback"
    #print "updated pouring data", len(self.pouring_data['ang'])
    if len(self.pouring_data['ang']) > 0:
      header = Header()
      header.stamp = self.pouring_data['stamp'][-1]
      # print 'Optimize called at ' + str(event.current_real)
      self.optimizer(header)


  def shift_cluster_center_curves(self,init_angle=None):
    #In this function, shift all of the coefficient curves down based on the starting angle
    for idx in range(len(self.classifier_data['clusters_inc'])):
      alpha_cluster_series = self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"]
      #1. Convert to power series
      a_clust_pow = alpha_cluster_series.convert(kind=np.polynomial.Polynomial, domain=alpha_cluster_series.domain, window = alpha_cluster_series.domain)
      #2. Evaluate the current series at theta_initial
      offset_volume = np.polynomial.polynomial.polyval(init_angle,a_clust_pow.coef)
      #3. Shift whole poly down by this much (change first coefficient of power series)
      a_clust_pow.coef[0] -= offset_volume
      #4. Convert new coefficient (power series) back to legendre coefficients
      a_clust_leg_new = a_clust_pow.convert(kind=np.polynomial.Legendre, domain = alpha_cluster_series.domain, window = alpha_cluster_series.domain)
      #5. Scale for a supposed initial container size
      # a_clust_leg_new.coef = self.container_estimated_initial_volume*a_clust_leg_new.coef
      #6. Store the new legendre coefficients of shifted polynomial back in cluster seriesv
      self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"] = a_clust_leg_new

  def reset_callback(self, req):

    rospy.logwarn("model_selection_node Reset requested")
    self.setup_variables()
    return TriggerResponse(True,"Reset complete")

  def generate_subscribers(self):
    #get the reported dV, angle for optimization
    self.sub = rospy.Subscriber('pouring_msg',PouringMsg,self.pouring_msg_callback, queue_size=1) # rospy.loginfo("subscribing")

    self.timer = rospy.Timer(rospy.Duration(0.01), self.optimization_callback)

    self.reset_srv = rospy.Service('reset_model_selection_node', Trigger, self.reset_callback)

    rospy.spin()


  def return_max_dvdth_model(self,model_selector, angle_lastest):
    dV_dth_list = []
    for idx in range(len(model_selector['model'])):
      sk_obj = model_selector['model'][idx].deriv(1)
      dv_dth_curr = np.polynomial.legendre.legval(angle_lastest, sk_obj.coef)
      dV_dth_list.append(dv_dth_curr)
    arg_min_resid = np.argmax(dV_dth_list)
    return arg_min_resid

  def optimizer(self, header):
    #This is running on a seperate thread from the callbacks
    #Note: keep the opt iter down so updated info can reach quickly

    if len(self.pouring_data['ang']) > 1:
      self.thread_lock.acquire()
      pouring_data = copy.copy(self.pouring_data)
      self.thread_lock.release()

      #Optimize
      model_selector = {'model':[], 'residual':[], 'residual_std':[]}
      for idx in range(len(self.classifier_data['clusters_inc'])):
        # alpha_init_dv = self.classifier_data['clusters_inc'][idx]["opt_coeff"]
        alpha_init_v = copy.copy(self.classifier_data['clusters_inc'][idx]["opt_coeff"])
        alpha_cluster = copy.copy(self.classifier_data['clusters_inc'][idx]["cluster_series_v_inc"])
        #optimize
        v_data = copy.copy(pouring_data['v'])
        dv_data = copy.copy(pouring_data['dv'])
        theta_data = copy.copy(pouring_data['ang'])
        # alpha_opt_dv, residuals_avg, residuals_std = self.opt_cls.update_coeff(alpha_init_dv, alpha_cluster, dv_data, theta_data)
        alpha_opt_v, residuals_avg, residuals_std = self.opt_cls.update_coeff(alpha_init_v, alpha_cluster, v_data, theta_data)
        #store current optimization for next iteration
        # model_selector['model'].append(alpha_opt_dv)
        model_selector['model'].append(alpha_opt_v)
        model_selector['residual'].append(residuals_avg)
        model_selector['residual_std'].append(residuals_std)
        # self.classifier_data['clusters_inc'][idx]["opt_coeff"] = alpha_opt_dv
        self.classifier_data['clusters_inc'][idx]["opt_coeff"] = alpha_opt_v
      #Select the best performing model based on residuals
      #based on most conservative dV/dth
      arg_min_resid = self.return_max_dvdth_model(model_selector, pouring_data['ang'][-1])
      #based on residual
      # arg_min_resid = np.argmin(model_selector['residual'])
      best_model = model_selector['model'][arg_min_resid]
      # best_model = model_selector['model'][0]
      coeff = best_model.coef.tolist()
      domain = best_model.domain.tolist()
      best_residual_avg = model_selector['residual'][arg_min_resid]
      best_residual_std = model_selector['residual_std'][arg_min_resid]
      #Publish the current optimal model
      pour_model_msg = PourModelPred()
      pour_model_msg.header = header
      pour_model_msg.coef_opt = coeff
      pour_model_msg.domain = domain
      pour_model_msg.residual_avg = best_residual_avg
      pour_model_msg.residual_std = best_residual_std
      pour_model_msg.type = 'legendre'
      self.model_pub.publish(pour_model_msg)
    else:
      rospy.loginfo('No (dv,ang) data has been recieved received')

def main():
  #This node toggles between classification and optimization and publishes the current best estimate of the container volume curve model
  rospy.init_node('model_selection_node')
  filename = 'classifier.p'  #this holds info for the cluster centers

  container_est_max_vol = 1#800 #170 #this should be passed in (can be related to requested pour amount, but it would be better to have some approximation from even visual observation)

  cls_obj = Model_selection(filename, est_cont_max_vol=container_est_max_vol)

  #run the subscriber to (dV,ang), which then calls the optimizer which then returns the best and optimized model (published)
  cls_obj.generate_subscribers()

if __name__ == '__main__':
  main()
