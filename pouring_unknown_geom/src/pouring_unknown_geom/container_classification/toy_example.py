#!/usr/bin/env python2
import numpy as np
from scipy.optimize import curve_fit, minimize, leastsq

class OptimizationCls(object):
  """docstring for OptimizationCls"""
  def __init__(self, x=None, y=None, poly_degree=3):
    self.x = x
    self.y = y
    self.y_pred = None
    self.poly_degree = poly_degree

  # def func(self,angles,c): 
  #   return np.array([np.sum([x**(len(c)-idx-1)*c[idx] for idx in range(0,len(c))]) for x in angles])

  def func(self,angles,c): 
    return np.array([np.sum([x**(len(c)-idx-1)*c[idx] for idx in range(0,len(c))]) for x in angles])


  def derv_exp(self,angles,c): return np.array([np.exp(-np.sum( [x**(len(c)-idx-2)*(len(c)-idx-1)*c[idx] for idx in range(0,len(c)-1)] )) for x in angles]) #this is exp(-f'(th_k,beta)) for all k (list)



  def jacobian_calc (self,coefs,angles,volumes,sign=1.0):
    v_iters = len(volumes)
    c_iters = len(coefs)

    # J = np.zeros([v_iters,c_iters]) #just least sqrs
    # J = np.zeros([v_iters+v_iters,c_iters]) #just least sqrs
    J = np.zeros([v_iters+c_iters+v_iters,c_iters]) #just least sqrs

    for idx in range(0,v_iters):
      for jdx in range(0,c_iters):
        J[idx,jdx] = angles[idx]**(c_iters-jdx-1)
    #Now for regulator
    for idx in range(0,c_iters):
      for jdx in range(0,c_iters):
        J[v_iters+idx,jdx] = float(idx==jdx) #diff produces 1, diag
    #Now for Exponential
    for idx in range(0,v_iters):
      for jdx in range(0,c_iters):
        val = np.exp(-np.sum( [coefs[kdx]*(c_iters-kdx-1)*angles[idx]**(c_iters-kdx-2) for kdx in range(0,c_iters-1)] ))
        # val = 1.0
        if (c_iters-jdx-2) >= 0:
          J[v_iters+c_iters+idx,jdx] = val*(c_iters-jdx-1)*angles[idx]**(c_iters-jdx-2)
        else:
          J[v_iters+c_iters+idx,jdx] = 0.0
    print "J size", J.shape
    return J


  def obj_func(self,coefs, angles, volumes, sign=1.0):
    print "obj coefs:", coefs
    f = self.func(np.array(angles), coefs) - volumes
    g = np.array([coef for coef in coefs]) #gets squared
    e = self.derv_exp(angles,coefs)
    # f = np.hstack([f,g])
    #these are useful when function is strictly increasing:
    # f = np.hstack([f,e])
    print "len f", len(f), " g", len(g), " e", len(e)
    f = np.hstack([f,g,e])
    return f


  def opt_function(self):
    self.opt_max_iter = 10000
    x0 = list(np.full(self.poly_degree+1 , 0.0))

    res, ier = leastsq(self.obj_func, x0, args=(self.x,self.y, -1.0,), Dfun=self.jacobian_calc)#, maxfev=self.opt_max_iter)
    # res, ier = leastsq(self.obj_func, x0, args=(self.x,self.y, -1.0,), maxfev=self.opt_max_iter)
    # print "output res ", res
    z_poly = np.polynomial.polynomial.Polynomial(res) #set domain here
    self.y_pred = np.polyval(z_poly.coef,self.x)

    # self.y_pred = np.polynomial.polynomial.polyval(self.x,z_poly.coef)
    print "ypred", self.y_pred, "\n y: ", self.y
    rmse = np.sqrt(np.mean((self.y_pred-self.y)**2))
    print "z_poly", z_poly, "type of z_poly",type(z_poly), "res", res
    return rmse


def main():
  x = np.linspace(1,10,1e4)
  # x = np.linspace(-1,1,11)
  # x = np.linspace(-10,10,11)
  # x = np.array([3,3])
  # y = x**2 + 3*x
  y = x**2 
  # y = x

  print "x", x,"\n y",y

  # cls_obj = OptimizationCls(x=x,y=y,poly_degree=9)
  cls_obj = OptimizationCls(x=x,y=y,poly_degree=3)

  rmse = cls_obj.opt_function()
  print "rmse", rmse



if __name__ == '__main__':
  main()