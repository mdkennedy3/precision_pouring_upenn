#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import animation
from scipy.optimize import minimize, leastsq
import cPickle as pickle
import rospkg
import json
import os
import sys
import time
from pouring_unknown_geom.container_classification import generate_classification

# TODO: pip install --user cvxpy :: example: http://nbviewer.jupyter.org/github/cvxgrp/cvx_short_course/blob/master/intro/control.ipynb  (MPC)
#import cvxpy


class Optimizer_class(object):
  """docstring for Optimizer_class"""
  def __init__(self, opt_max_iter=1000, domain_win=[0.0,2.4], num_cntrl_pts=100):
    self.opt_max_iter = opt_max_iter
    self.num_cntrl_pts = num_cntrl_pts #points to use for soft constraint
    self.domain_win = domain_win

  def residual_calc(self,v,theta, v_coeff, resid_max_bool=False):
    # r_i = y_i - f(th,alpha)
    #y,th,coeff all np arrays
    f_list = np.polyval(v_coeff,theta) #power series fit
    residuals = v - f_list
    residuals_avg = np.mean(np.abs(residuals))
    residuals_stddev = np.std(np.abs(residuals))
    residual_max = np.max(residuals)
    if resid_max_bool:
      return residuals_avg, residuals_stddev, residuals, residual_max
    else:
      return residuals_avg, residuals_stddev, residuals

  def jacobian_calc (self,coefs,angles,volumes,theta_cntrl_pts,regulator_gamma , sign=1.0):
    v_iters = len(volumes)
    c_iters = len(coefs)
    der_exp_iters = len(theta_cntrl_pts)

    # J = np.zeros([v_iters,c_iters]) #just least sqrs
    # J = np.zeros([v_iters+c_iters,c_iters]) #just least sqrs
    #J = np.zeros([v_iters+der_exp_iters,c_iters]) #just least sqrs
    J = np.zeros([v_iters+c_iters+der_exp_iters,c_iters]) #just least sqrs

    for idx in range(0,v_iters):
      for jdx in range(0,c_iters):
        J[idx,jdx] = angles[idx]**(c_iters-jdx-1)
    #Now for regulator
    for idx in range(0,c_iters):
      for jdx in range(0,c_iters):
        J[v_iters+idx,jdx] = regulator_gamma*float(idx==jdx) #diff produces 1, diag
    #Now for Exponential
    for idx in range(0,der_exp_iters):
      for jdx in range(0,c_iters):
        # print "idx", idx
        val = np.exp(-np.sum( [coefs[kdx]*(c_iters-kdx-1)*theta_cntrl_pts[idx]**(c_iters-kdx-2) for kdx in range(0,c_iters-1)] ))
        if (c_iters-jdx-2) >= 0:
          J[v_iters+c_iters+idx,jdx] = -val*(c_iters-jdx-1)*theta_cntrl_pts[idx]**(c_iters-jdx-2)
        else:
          J[v_iters+c_iters+idx,jdx] = 0.0
    # print "J shape", J.shape
    return J

  def func(self,angles,c): return np.array([np.sum([x**(len(c)-idx-1)*c[idx] for idx in range(0,len(c))]) for x in angles])
  def derv_exp(self,angles,c): return np.array([np.exp(-np.sum( [x**(len(c)-idx-2)*(len(c)-idx-1)*c[idx] for idx in range(0,len(c)-1)] )) for x in angles]) #this is exp(-f'(th_k,beta)) for all k (list)
  def obj_func(self,coefs, angles, volumes,theta_cntrl_pts, regulator_gamma , sign=1.0):
    f = self.func(np.array(angles), coefs) - volumes #residual
    g = regulator_gamma*np.array([coef for coef in coefs]) #regulator
    # e = self.derv_exp(angles,coefs) #derivative cost :: can also look at set points (angles evenly placed in domain)
    e = self.derv_exp(theta_cntrl_pts,coefs) #derivative cost :: can also look at set points (angles evenly placed in domain)
    # f = np.hstack([f,g]) #these are useful when function is strictly increasing:
    #f = np.hstack([f,e])
    f = np.hstack([f,g,e])
    return f



  # def update_coeff(self, coeff_init_v_series, coeff_cluster_center_v_series, v_data, theta_data, num_pts=10):
  def update_coeff(self,curr_coef=None, vol_data=None,theta_data=None):
    #This works with power series only
    alpha_init_v = curr_coef.coef  #coefficeints  V
    curve_domain = curr_coef.domain
    self.domain_win = curve_domain
    self.num_cntrl_pts = self.num_cntrl_pts
    theta_cntrl_pts = np.linspace(self.domain_win[0],self.domain_win[1],self.num_cntrl_pts)
    regulator_gamma = 0.1
    #2. Obtain the constraints
    res, ier = leastsq(self.obj_func, alpha_init_v, args=(theta_data, vol_data,theta_cntrl_pts, regulator_gamma, -1.0,), Dfun=self.jacobian_calc, maxfev=self.opt_max_iter) #debug the jacobian funct
    #res, ier = leastsq(self.obj_func, alpha_init_v, args=(theta_data,vol_data,theta_cntrl_pts, regulator_gamma, -1.0,),maxfev=self.opt_max_iter)
    coeff_updated =  np.polynomial.polynomial.Polynomial(res)
    #This value is the average residual from the fitting
    residuals_avg, residuals_stddev, residuals,residual_max = self.residual_calc(vol_data, theta_data, coeff_updated.coef,resid_max_bool=True)
    return coeff_updated, residuals_avg, residuals_stddev


def main():
  x = np.linspace(0.0,0.4,20)
  y = x**2 + 3*x
  cls_obj = Optimizer_class()
  poly_degree = 9
  dom_win = [0,2.4]
  x0 = np.polynomial.polynomial.Polynomial(np.zeros([poly_degree]),domain=dom_win,window=dom_win)
  coeff_updated, residuals_avg, residuals_stddev = cls_obj.update_coeff(curr_coef=x0,vol_data=y,theta_data=x)
  y_pred = np.polyval(coeff_updated.coef,x)
  e = y - y_pred
  print "resid", residuals_avg#, "\ny",y,"\ny_pred",y_pred, "\nerr",e
  plt.scatter(x,y,label="train",color='r')
  plt.scatter(x,y_pred,label='test',color='b')
  plt.legend()
  plt.show()


if __name__ == '__main__':
  main()

#check time of code $time python opt_srv_gp_compatible.py
