#!/usr/bin/env python
import rospy
import numpy as np
from pouring_msgs.srv import * #PyOptimizationServ
from pouring_unknown_geom.gauss_newton_opt import opt_srv_gp_compatible

class coef_class(object):
  def __init__(self,coef=[],domain=[]):
    self.coef = coef
    self.domain = domain

class PyOptProfileCls(object):
  def __init__(self,opt_num_iter=1000,dom_win=[0,2.4]):
      self.opt_cls = opt_srv_gp_compatible.Optimizer_class(opt_max_iter=opt_num_iter, domain_win=dom_win) #num_iter is max number of iterations to run per opt

  def opt_callback(self,req):
    V = req.vol_data
    ang = req.ang_data
    coef = req.coef_init
    coef_opt, r_avg, r_std = self.opt_function(V=V,ang=ang,coef_inpt=coef)
    resp = PyOptimizationServResponse()
    resp.coef_opt = coef_opt
    resp.resid_avg = r_avg
    resp.resid_std = r_std
    return resp


  def opt_function(self,V=[],ang=[],coef_inpt=[]):
    coef = coef_class(coef=coef_inpt, domain=[0,2.4])
    curr_coef_upd, residuals_avg, residuals_std = self.opt_cls.update_coeff(curr_coef=coef, vol_data=V,theta_data=ang)
    return curr_coef_upd, residuals_avg, residuals_std


def main():
  rospy.init_node("profile_opt_py_serv")
  cls_obj = PyOptProfileCls()
  service = rospy.Service("~py_profile_opt_srv", PyOptimizationServ, cls_obj.opt_callback)
  rospy.spin()

if __name__ == '__main__':
  main()

