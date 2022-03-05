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
  def __init__(self, num_iter, residual_threshold, weight_funct_k1 = 1.0, weight_funct_k2=10.0, scaling_constant_low_pass_filt_value=0.5, residual_gain=1000, opt_max_iter=20, K_constraint=3.0, constraint_weight=1.0):
    """INfO
    opt_max_iter of 2 iterations was optimal for minimizing avg residual over the entire pour
     """
    self.residual_threshold = residual_threshold
    self.weighting_func_k1 = weight_funct_k1 #1.0
    self.weighting_func_k2 = weight_funct_k2 #if 2.19, then 1/10 of initial value after theta=60deg  (from 2.3=k2*thd)
    self.residual_gain = residual_gain #was 100
    self.opt_max_iter = opt_max_iter
    self.num_cntrl_pts = 10 #points to use for soft constraint
    self.domain_win = [0.0,2.4]#np.pi]
    self.K_constraint = K_constraint #this is for the soft constriants exp(-K dV/dth)
    self.constraint_weight = constraint_weight

    #setting maximum dV/dth
    self.dV_dth_max_con = 2000.0  #This should be a param passed in!!



    self.time_obj_func = 0
    self.num_obj_func = 0
    self.time_jac_calc = 0
    self.num_jac_calc = 0
    self.time_optimize = 0
    self.num_optimize = 0

  def residual_calc(self,v,theta, v_coeff, resid_max_bool=False):
    # r_i = y_i - f(th,alpha)
    #y,th,coeff all np arrays
    f_list = np.polynomial.legendre.legval(theta,v_coeff)
    residuals = v - f_list
    residuals_avg = np.mean(np.abs(residuals))
    residuals_stddev = np.std(np.abs(residuals))
    residual_max = np.max(residuals)
    if resid_max_bool:
      return residuals_avg, residuals_stddev, residuals, residual_max
    else:
      return residuals_avg, residuals_stddev, residuals


  def weighting_function_dth(self,k1,k2,dth):
    return k1*np.exp(-k2*dth)

  def obj_func(self, x,dv_data,theta_data,coef_clust_v, sign=1.0):
    '''
    Objective function: #Define optimization functions: https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html
    Variables:
    x: coefficient:= alpha_v
    v_data, theta_data: V,th
    coef_clust_v: cluster coefficient derivative := alpha_dv_clust
    clust_obj: numpy series object used to obtain the basis
    gamma_t: current estimate of scaling factor (low pass filtered)
    sign: used for optimization handle for increasing/decreasing etc in optimization (required)
    '''
    t0 = time.time()
    #1. Obtain parameters 
    #1a. Normalize coefficients and rescale
    coef_clust_v_hat = np.divide(coef_clust_v.coef,np.linalg.norm(np.array(coef_clust_v.coef)))
    #1b. Fit scaled coefficients to function to obtain function estimate
    f_list  = np.polynomial.legendre.legval(theta_data,x) #function outputs given current coefficient estimates
    r_list = dv_data #actual measurements
    th_delta = np.abs(theta_data[-1] - theta_data[0]) #dTheta window
    weight_func = self.weighting_function_dth(self.weighting_func_k1,self.weighting_func_k2,th_delta)    #need K1, K2, dth for W fnct
    #2. Calculate Objective Function:  J = sum_i ||r_i - f(th,dv_coef)||**2 + W(th)*|| (dv_coef_hat_cluster*||dv_coef|| - dv_coef) ||**2
    # obj_vect  = np.hstack([f_list,coef_clust_v_hat*np.linalg.norm(x)*np.sqrt(weight_func) ]) - np.hstack([r_list, x*np.sqrt(weight_func)])
    obj_vect  = np.hstack([f_list-r_list,coef_clust_v_hat*np.linalg.norm(x)*np.sqrt(weight_func) -  x*np.sqrt(weight_func)])
    #Adding the soft constraints to ensure dV/dth > 0 at select points along the curve
    #1. Make points along the curve th_i
    th_pts = np.linspace(self.domain_win[0],self.domain_win[1], self.num_cntrl_pts)
    #2. For each th_i find dV/dth then exp(-K*dV/dth)
    dV_dth = [np.polynomial.legendre.legval(theta, np.polynomial.legendre.Legendre(x, domain=self.domain_win, window=self.domain_win).deriv(1).coef) for theta in th_pts]
    exp_funct = np.array([self.constraint_weight*(np.exp(-self.K_constraint*dV) + np.exp(self.K_constraint*(dV - self.dV_dth_max_con))) for dV in dV_dth])
    #3. Add to objective
    obj_vect = np.hstack([obj_vect, exp_funct])
    self.time_obj_func += (time.time() - t0)
    self.num_obj_func += 1
    return obj_vect

  def legendre_basis(self,basis=1,theta=0, clust_basis=[]):
    #Given the basis and theta value, evaluate and return
    basis_value = np.polynomial.legendre.legval(theta,clust_basis.basis(basis).coef)
    return basis_value

  def jacobian_calc(self, x,v_data,theta_data,coef_clust_v, sign=1.0):
    """ Derivative of objective function """
    '''
    Variables:
    x: coefficient:= alpha_v
    v_data, theta_data: V,th
    coef_clust_v: cluster coefficient := alpha_v_clust
    coef_clust_v: numpy series object used to obtain the basis
    gamma_t: current estimate of scaling factor (low pass filtered)
    sign: used for optimization handle for increasing/decreasing etc in optimization (required)
    #need to calculate J_j, from summed columns of J_ij where i is the i'th data point (real data), and j is the j'th coefficient
    #1. Obtain neccessary parameters
    '''
    t0 = time.time()
    coef_clust_v_hat = np.divide(coef_clust_v.coef,np.linalg.norm(np.array(coef_clust_v.coef)))
    f_list  = np.polynomial.legendre.legval(theta_data,x) #function outputs given current coefficient estimates
    r_list = v_data #actual measurements
    th_delta = np.abs(theta_data[-1] - theta_data[0]) #dTheta window
    weight_func = self.weighting_function_dth(self.weighting_func_k1,self.weighting_func_k2,th_delta)    #need K1, K2, dth for W fnct
    #Jacobian for leastsq
    r_list = v_data #actual measurements
    i_iter = len(r_list) #for the residual obj
    j_iter = len(x) #for the constraint attraction to cluster center
    l_iter = self.num_cntrl_pts #for the soft constraints
    coef_clust_v_hat = np.divide(coef_clust_v.coef,np.linalg.norm(np.array(coef_clust_v.coef)))
    th_pts = np.linspace(self.domain_win[0],self.domain_win[1], self.num_cntrl_pts)
    dV_dth = [np.polynomial.legendre.legval(theta, np.polynomial.legendre.Legendre(x, domain=self.domain_win, window=self.domain_win).deriv(1).coef) for theta in th_pts]
    exp_funct = [np.exp(-self.K_constraint*dV) for dV in dV_dth]
    exp_funct_upper_bnd = [np.exp(self.K_constraint*(dV - self.dV_dth_max_con)) for dV in dV_dth]
    J_alternate = np.zeros([i_iter+j_iter + l_iter, j_iter])
    # J_alternate = np.zeros([i_iter+j_iter, j_iter])
    for idx in range(i_iter):
      for jdx in range(j_iter):
        J_alternate[idx,jdx] = self.legendre_basis(basis=jdx,theta=theta_data[idx],clust_basis=coef_clust_v) 
    for mdx in range(i_iter,i_iter+j_iter):
      for jdx in range(j_iter):
        mdx_arg = mdx-i_iter #for argument passing
        J_alternate[mdx,jdx] = np.sqrt(weight_func)*(coef_clust_v_hat[mdx_arg]*x[jdx]/np.linalg.norm(x) - int(mdx_arg == jdx))
    for ldx in range(i_iter+j_iter, i_iter+j_iter+l_iter):
      for jdx in range(j_iter):
        ldx_arg = ldx - (i_iter+j_iter)
        #Obtain 
        # np.polynomial.legendre.legval(theta,coef_clust_v.basis(jdx).coef)
        theta_l = th_pts[ldx_arg]
        dV_dth_eval_th_l = np.polynomial.legendre.legval(theta_l,coef_clust_v.basis(jdx).deriv(1).coef)
        # rospy.loginfo(" Values in jacobian dVdth_basis_l: %f, exp(-K dV/dth): %f "%(dV_dth_eval_th_l, exp_funct[ldx_arg]))
        J_alternate[ldx,jdx] = self.constraint_weight*self.K_constraint*dV_dth_eval_th_l*(-exp_funct[ldx_arg] + exp_funct_upper_bnd[ldx_arg])  #contents
    self.time_jac_calc += (time.time() - t0)
    self.num_jac_calc += 1
    return J_alternate



  def define_constraints_dict(self,domain_win=[0,np.pi], num_pts=10):
    #define the constraint dictionary required by scipy minimize
    th_pts = np.linspace(domain_win[0],domain_win[1], num_pts)
    cons = []
    for theta in th_pts:
      cons.append({'type': 'ineq',
                 'fun' : lambda x: np.polynomial.legendre.legval(theta, np.polynomial.legendre.Legendre(x, domain=domain_win, window=domain_win).deriv(1).coef),
                 'jac' : lambda x: np.array([np.polynomial.legendre.legval(theta, np.polynomial.legendre.Legendre(x,domain=domain_win, window=domain_win).deriv(1).basis(mdx).coef) for mdx in range(len(x))]) }) #is this right order?


  def update_coeff(self, coeff_init_v_series, coeff_cluster_center_v_series, v_data, theta_data, num_pts=10):
    #Both coeff objs are numpy series objects, extract coefficients
    # alpha_clust_dv_series = coeff_cluster_center_v_series.deriv(1)  #series object dV for clusters derived from V cluster object
    alpha_clust_v_series = coeff_cluster_center_v_series  #series object dV for clusters derived from V cluster object
    alpha_init_v = coeff_init_v_series.coef  #coefficeints  V
    curve_domain = coeff_cluster_center_v_series.domain
    self.domain_win = curve_domain
    self.num_cntrl_pts = num_pts
    #2. Obtain the constraints 
    #t0 = time.time()
    res, ier = leastsq(self.obj_func, alpha_init_v, args=(v_data, theta_data, alpha_clust_v_series,-1.0,), Dfun=self.jacobian_calc, maxfev=self.opt_max_iter)
    #self.time_optimize += time.time() - t0
    #self.num_optimize += 1
    #if ier == 5:
    #  print 'Leastsq: Number of calls to function has reached maxfev'
    # print "Avg time for optimize: ", self.time_optimize/self.num_optimize
    # print "Num times optimize: ", self.num_optimize
    coeff_updated = np.polynomial.legendre.Legendre(res, domain=curve_domain, window=curve_domain)  #series object returned
    #This value is the average residual from the fitting
    residuals_avg, residuals_stddev, residuals,residual_max = self.residual_calc(v_data, theta_data, coeff_updated.coef,resid_max_bool=True)
    return coeff_updated, residuals_avg, residuals_stddev


def main():
  # rospy.init_node('pour_opt_server_node')
  num_iter = 500
  residual_threshold = 1e-2
  cls_obj = Optimizer_class(num_iter,residual_threshold)
  #Example code on a test example
  #1. Generate example cluster center: 
  end_domain = np.pi
  num_pts = 20  #num of data points
  #Manually Make cluster from toy data learned
  th_test = np.linspace(0,end_domain,num_pts)
  dv_learn = th_test**3
  poly_deg = 4
  sk_clust = np.polynomial.legendre.Legendre.fit(th_test,dv_learn,3, domain=[th_test[0],th_test[-1]], window=[th_test[0],th_test[-1]])
  #Load cluster from trained data
  rospack = rospkg.RosPack()
  filename = 'classifier.p'
  data_path = rospack.get_path("pouring_unknown_geom") + '/classification/' + filename  #data path
  with open(data_path, 'rb') as fp: 
    data = pickle.load(fp)
  classifier_data = data['clusters']
  data = None #free up memory
  #Load the actual cluster
  clust_idx = 1 #choose cluster (0,1,2)
  sk_clust = classifier_data['clusters_inc'][clust_idx]["cluster_series_v_inc"]#.deriv(1)
  #Normalize the cluster
  sk_clust_coef_normed = np.divide(sk_clust.coef, np.linalg.norm(sk_clust.coef))
  sk_clust = np.polynomial.legendre.Legendre(sk_clust_coef_normed, domain=[0,end_domain], window=[0,end_domain])
  #2. Generate initial guess of coefficient derivative
  sk_init_dv = sk_clust.deriv(1)
  #3. Generate Experimental data
  th = np.linspace(0,end_domain,num_pts)
  # th = np.linspace(np.pi/2.,2.,num_pts)
  dV = 6*th**2 + 2*th + 1   #derivative of 2th**3+th**2+th
  gen_cls =  generate_classification.Container_classifier_class()
  #3b. Generate Real data (from actual pour)
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
  #obtain dV,th
  dV = []
  th = []
  for idx in range(1,len(angles_inc)):
    dV.append((volumes_inc[idx]-volumes_inc[idx-1])/(angles_inc[idx]-angles_inc[idx-1]))
    th.append(angles_inc[idx-1] + 0.5*(angles_inc[idx]-angles_inc[idx-1]))
  #4. Loop through the data returning the estimated coefficents  (each time updating an animation of the curve vs real points)  (then subplot the residuals with each measurement last value)
  animiation_dict = {'dV':[], 'th':[], 'coeff_pts':[], 'resid':[], 'coef_clust':[] }
  coeff_dv_upd = sk_init_dv
  for idx in range(len(th)):
    #new data obtained in each loop
    curr_th = th[:idx]
    curr_dV = dV[:idx]
    if len(curr_th) > 2:
      # print "\n start coef_dv: ", coeff_dv_upd.coef
      # print "\n start coef_clust: ",sk_clust.coef
      coeff_dv_upd,res, res_std = cls_obj.update_coeff(coeff_dv_upd, sk_clust, curr_dV, curr_th)
      print "current: ", (float(idx)/len(th))*100., "%"
      #save animation objects:
      animiation_dict['th'].append(curr_th[-1])
      animiation_dict['dV'].append(curr_dV)
      animiation_dict['coeff_pts'].append( coeff_dv_upd.linspace(len(th),domain=[th_test[0],th_test[-1]]))
      animiation_dict['resid'].append(res)
      animiation_dict['coef_clust'].append( sk_clust.deriv(1).linspace(len(th),domain=[th_test[0],th_test[-1]]))
  fig = plt.figure()
  ax = fig.gca()
  # ax.plot(th,dV,label='dV',linewidth=10.0)
  ax.scatter(th,dV,label='dV',linewidth=10.0)
  # print "len of coef pts: ", len(animiation_dict['resid']), "\n interior: ", animiation_dict['resid']
  # ax.plot(animiation_dict['coeff_pts'][-1][0],animiation_dict['coeff_pts'][-1][1],label='a*')
  ax.plot(animiation_dict['coef_clust'][-1][0],animiation_dict['coef_clust'][-1][1],label='clust')
  for idx in range(len(animiation_dict['coeff_pts'])-1):
    ax.plot(animiation_dict['coeff_pts'][idx][0],animiation_dict['coeff_pts'][idx][1],'-.',label=('$a*_{'+str(idx)+"}$"), linewidth=1.0)
  ax.plot(animiation_dict['coeff_pts'][idx][0],animiation_dict['coeff_pts'][-1][1],'r--',label=('$a*_{'+str(idx)+"}$"), linewidth=5.0)
  # print "resid:", animiation_dict['resid'], "\n theta:",animiation_dict['th']
  # print "lengths: ", len(animiation_dict['resid']), "vs", len(animiation_dict['th'])
  ax.plot(animiation_dict['th'],animiation_dict['resid'],label='res')
  # ax.set_xlabel('$\\theta$ (rad)', fontsize=18)
  ax.set_xlim(0,th[-1])#np.pi)
  ax.set_ylim(-1000,1000)
  ax.set_ylabel('$dV$ (ml)', fontsize=18)
  plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
  plt.show()
  # plt.savefig("example_convergence_toy_example.png")
  #Now show animation
  # fig = plt.figure()
  # ax = plt.axes(xlim=(th_test[0], th_test[-1]), ylim=(0, 100))
  # N = 4
  # lines = [plt.plot([], [])[0] for _ in range(N)]
  # def init():    
  #     for line in lines:
  #         line.set_data([], [])
  #     return lines
  # def animate(i):
  #     # for j,line in enumerate(lines):
  #     #     line.set_data([0, 2], [10 * j,i])
  #     lines[0].set_data(animiation_dict['th'][i], animiation_dict['dV'][i])
  #     lines[1].set_data(th, animiation_dict['coeff_pts'][i])
  #     lines[2].set_data(animiation_dict['th'][i], animiation_dict['resid'][:i])
  #     lines[3].set_data(th, animiation_dict['coef_clust'][i])
  #     return lines
  # anim = animation.FuncAnimation(fig, animate, init_func=init,
  #                                frames=len(animiation_dict['th']), interval=2, blit=True)
  # plt.show()

if __name__ == '__main__':
  main()
