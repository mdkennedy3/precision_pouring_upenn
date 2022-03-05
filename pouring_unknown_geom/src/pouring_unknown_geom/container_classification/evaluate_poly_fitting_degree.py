#!/usr/bin/env python2
#Created by Monroe Kennedy III 
'''
This script evaluates the rmse for fitting real/simulated 
container pouring profiles with different degree polynomials, 
justifying the use of the degree chosen.
'''
import rospy
import numpy as np
from numpy import matlib
import rospkg
import os
import json
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D

import matplotlib
# matplotlib.rcParams['ps.useafm'] = True
# matplotlib.rcParams['pdf.use14corefonts'] = True
# matplotlib.rcParams['text.usetex'] = True

from sklearn import mixture
from sklearn.decomposition import PCA
import sys #to get size of variable sys.getsizeof(s) in bits
import argparse

import cPickle as pickle
from pouring_unknown_geom.tyler_code import curve_fitting as tyler_curve_fit

from scipy.optimize import curve_fit, minimize, leastsq
#For violin plots
import seaborn as sns


class RMSE_Eval(object):
  """docstring for RMSE_Eval"""
  def __init__(self):
    self.rmse_list = []
    self.poly_degree = []


class args_cls(object):
  """Neccessary for tyler code """
  def __init__(self, derivative=0, normalized=False, inverted=False, scaled=False):
    self.derivative = derivative
    self.normalized = normalized
    self.inverted = inverted
    self.scaled = scaled


class Container_classifier_class(object):
  """docstring for ClassName"""
  def __init__(self):
    # self.arg = arg
    self.polynomial_degree = 9  #9 #7 is best for V_dec curve
    self.domain_end = 2.4#2.5#np.pi#2.5#2.3#1.65#2.0  #vs np.pi
    #5th/6th order allows us to keep "006", for 7 order it must be excluded (however the data is better fit by 7th order)
    self.rospack = rospkg.RosPack()
    self.data_path = self.rospack.get_path("pouring_unknown_geom") + '/data/'
    self.data_list = ["001","002","003","004","005","006",
                      "007","008","009","010","011","012",
                      "013","014","015","016","017","018",
                      "019","020","021","022","023",'S000', 'S001', 'S002', 'S003', 'S004', 'S005', 'S006', 'S007', 'S008', 'S009', 'S010', 'S011', 'S012', 'S013', 'S014', 'S015', 'S016', 'S017', 'S018', 'S019', 'S020', 'S021', 'S022', 'S023', 'S024', 'S025', 'S026', 'S027', 'S028', 'S029', 'S030', 'S031', 'S032', 'S033', 'S034', 'S035', 'S036', 'S037', 'S038', 'S039', 'S040', 'S041', 'S042', 'S043', 'S044', 'S045', 'S046', 'S047', 'S048', 'S049', 'S050', 'S051', 'S052', 'S053', 'S054', 'S055', 'S056', 'S057', 'S058', 'S059', 'S060', 'S061', 'S062', 'S063', 'S064', 'S065', 'S066', 'S067', 'S068', 'S069', 'S070', 'S071', 'S072', 'S073', 'S074', 'S075', 'S076', 'S077', 'S078', 'S079', 'S080', 'S081', 'S082', 'S083', 'S084', 'S085', 'S086', 'S087', 'S088', 'S089', 'S090', 'S091', 'S092', 'S093', 'S094', 'S095', 'S096', 'S097', 'S098', 'S099', 'S100', 'S101', 'S102', 'S103', 'S104', 'S105', 'S106', 'S107', 'S108', 'S109', 'S110', 'S111', 'S112', 'S113', 'S114', 'S115', 'S116', 'S117', 'S118', 'S119', 'S120', 'S121', 'S122', 'S123', 'S124', 'S125', 'S126', 'S127', 'S128', 'S129', 'S130', 'S131', 'S132', 'S133', 'S134', 'S135', 'S136', 'S137', 'S138', 'S139', 'S140', 'S141', 'S142', 'S143', 'S144', 'S145', 'S146', 'S147', 'S148', 'S149', 'S150', 'S151', 'S152', 'S153', 'S154', 'S155', 'S156', 'S157', 'S158', 'S159', 'S160', 'S161', 'S162', 'S163', 'S164', 'S165', 'S166', 'S167', 'S168', 'S169', 'S170', 'S171', 'S172', 'S173', 'S174', 'S175', 'S176', 'S177', 'S178', 'S179', 'S180', 'S181', 'S182', 'S183', 'S184', 'S185', 'S186', 'S187', 'S188', 'S189', 'S190', 'S191', 'S192', 'S193', 'S194', 'S195', 'S196', 'S197', 'S198', 'S199', 'S200', 'S201', 'S202', 'S203', 'S204', 'S205', 'S206', 'S207', 'S208', 'S209', 'S210', 'S211', 'S212', 'S213', 'S214', 'S215', 'S216', 'S217', 'S218', 'S219', 'S220', 'S221', 'S222', 'S223', 'S224', 'S225', 'S226', 'S227', 'S228', 'S229', 'S230', 'S231', 'S232', 'S233', 'S234', 'S235', 'S236', 'S237', 'S238', 'S239', 'S240', 'S241', 'S242', 'S243', 'S244', 'S245', 'S246', 'S247', 'S248', 'S249', 'S250', 'S251', 'S252', 'S253', 'S254', 'S255', 'S256', 'S257', 'S258', 'S259', 'S260', 'S261', 'S262', 'S263', 'S264', 'S265', 'S266', 'S267', 'S268', 'S269', 'S270', 'S271', 'S272', 'S273', 'S274', 'S275', 'S276', 'S277', 'S278', 'S279', 'S280', 'S281', 'S282', 'S283']


    #hold these as their curves are incomplete
    self.outliers = ["000", "011", "013", "018", "019", "014", "S266","S002","S007","S038","S045","S048","S049","S051","S074","S076","S078","S080","S084","S098","S103", "S105",'S106', 'S107', 'S109', 'S110', 'S111', 'S112', 'S113', 'S126', 'S129', 'S130', 'S131', 'S133', 'S136', 'S138', 'S140', 'S141', 'S142', 'S143', 'S145', 'S146', 'S147', 'S148', 'S150', 'S158', 'S159', 'S161', 'S163', 'S165', 'S166', 'S167', 'S168', 'S169', 'S170', 'S171', 'S172', 'S173', 'S174', 'S175', 'S176', 'S177', 'S180', 'S186', 'S190', 'S191', 'S193', 'S194', 'S195', 'S197', 'S198', 'S200', 'S201', 'S202', 'S203', 'S204', 'S205', 'S206', 'S208', 'S209', 'S210', 'S211', 'S212', 'S236', 'S257', 'S259', 'S260', 'S271', 'S272', 'S275', 'S276']  #014, #006 didn't have enough near the beginning and this caused very weird angles

    self.fail_list = []
    self.legend_text_size = 20
    self.plot_axis_text_size = 28
    self.rmse_keep_out_list = []

  def filter_volume_data(self,data, MAX_VOLUME, last_ang=np.pi, angles_dec=[], volumes_dec=[]):
    #Written by Tyler Larkworthy
    #this function takes in the list of angles and volumes, and checks that to iterative values do not have 1. same angle, 2. volumes are strictly positive and decreasing
      # data = json.load(open(dir + '/' + file))
      angs = data[0]; vols = data[1]
      p_angs = []; p_vols = []; next_ang = 0.0
      # In some cases, the initial volume of the container was not set correctly, so the data has a nonzero final volume
      # This corrects the problem by shifting the volume data accordingly
      if vols[-1] != 0.0:
          for i in range(0, len(vols)):
              vols[i] -= vols[-1]
      # Now, iterate over the data from the file. But, only store the lowest volume at each angle, stop storing data after
      # the first 0 volume is encountered     
      ignore_zeros = False
      for i in range(0, len(angs) - 1):
          if vols[i] == 0.0:
              if not ignore_zeros:
                  ignore_zeros = True
                  p_angs.append(angs[i])
                  p_vols.append(vols[i])
                  if angs[i] > last_ang:
                      # Keep track of the first angle at which all of the fluid has flowed out
                      last_ang = angs[i]
          elif round(angs[i], 3) >= round(next_ang, 3) and angs[i] < angs[i+1] and (vols[i] < vols[0] or vols[i] == MAX_VOLUME):
              p_angs.append(angs[i])
              p_vols.append(vols[i])
          if round(angs[i+1], 3) > round(next_ang, 3):
              next_ang += 0.05
      # If the first processed angle is not zero for some reason, add in an artificial (0, MAX_VOLUME) data point
      if p_angs[0] != 0.0:
          p_angs.insert(0, 0)
          p_vols.insert(0, MAX_VOLUME)
      if p_vols[-1] > 0.0:
        p_angs.append(p_angs[-1]+0.05)
        p_vols.append(-0.01)


      #ADD THIS TO FIX CURVES
      p_angs.append(np.pi)
      p_vols.append(0.0)


      # For a processed set of data from a single file, each angle should be associated with exactly one volume.
      # This iteration ensures that an angle is not repeated.
      for i in range(0, len(p_angs) - 1):
          if i + 1 < len(p_angs) and p_angs[i] == p_angs[i + 1]:
              p_vols[i] = (p_vols[i] + p_vols[i+1]) / 2.
              p_vols.pop(i+1)
              p_angs.pop(i+1)
      #Now ensure a Volume is not repeated: 
      for i in range(0, len(p_vols) - 1):
          if i + 1 < len(p_vols) and p_vols[i] == p_vols[i + 1]:
              p_angs[i] = (p_angs[i] + p_angs[i+1]) / 2.
              p_angs.pop(i+1)
              p_vols.pop(i+1)
      #find derivative functions immediately 
      volume_derv_dec = []
      angle_derv_dec = []
      for idx in range(1,len(p_angs)-1):
        # #Volume method of [V(i+1) - V(i-1)]/2
        # ang_curr = p_angs[idx] +  (p_angs[idx+1] - p_angs[idx-1])*0.5
        # vol_curr = p_vols[idx] +  (p_vols[idx+1] - p_vols[idx-1])*0.5
        #Regular: dV = V(i+1) - V(i)
        # ang_curr = p_angs[idx] +  (p_angs[idx+1] - p_angs[idx])*0.5
        # vol_curr = p_vols[idx] +  (p_vols[idx+1] - p_vols[idx])*0.5

        ang_curr = p_angs[idx] +  (p_angs[idx+1] - p_angs[idx])*0.5
        vol_curr = ((p_vols[idx+1] - p_vols[idx-1])*0.5)/((p_angs[idx+1] - p_angs[idx-1])/0.5)
        volume_derv_dec.append(vol_curr)
        angle_derv_dec.append(ang_curr)
      # Append the processed data
      angles_dec += p_angs
      volumes_dec += p_vols

      #Now find increasing volume points and derivative points
      volumes_inc = []; volumes_inc.append(0); volume_occ = 0
      angles_inc = []; angles_inc.append(0)


      for idx in range(1,len(angles_dec)):
        angles_inc.append(angles_dec[idx])
        volume_occ += volumes_dec[idx-1] - volumes_dec[idx]
        volumes_inc.append(volume_occ)

      volume_derv_inc = []; volume_derv_inc.append(0)
      angle_derv_inc = []; angle_derv_inc.append(0)
      for idx in range(0,len(angles_dec)-1):
        angle_derv_inc.append( angles_dec[idx])
        volume_derv_inc.append( ((volumes_dec[idx-1] - volumes_dec[idx]))/((angles_dec[idx] - angles_dec[idx-1])) )  #this is the 3 point derivative to assist with any noise




      return angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc, angle_derv_dec, volume_derv_dec,  last_ang

  def read_in_data(self):
    self.data_dict = dict()
    file_name_list = self.data_list
    for file in file_name_list:
      if file not in self.outliers:
        curr_dir = self.data_path+file
        for fn in os.listdir(curr_dir):
            if fn != 'INFO.json' and fn[-1] != "~":
              # print "loading: ", fn
              curr_data = json.load(open(curr_dir + '/' + fn))
              MAX_VOLUME = json.load(open( curr_dir + '/INFO.json'))["max_volume"]
              angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc, angle_derv_dec, volume_derv_dec,  last_ang = self.filter_volume_data(curr_data, MAX_VOLUME, last_ang=0, angles_dec=[], volumes_dec=[])
              #tyler process file
              args = args_cls()
              nangles, nvolumes, nlast_ang = tyler_curve_fit.process_file(curr_dir, fn, MAX_VOLUME, last_ang=last_ang, angles=[], volumes=[])
              angles_dec, volumes_dec = nangles, nvolumes
              # print "\n\nangles: ", angles_dec
              # print "\nvolumes: ", volumes_dec
              # args_inc = args_cls(inverted=True) 
              args_inc = args_cls(inverted=False)

              angles_inc, volumes_inc = tyler_curve_fit.process_directory(curr_dir, args_inc)
              curr_data_dict = {'angs_dec':angles_dec, 'vols_dec':volumes_dec, 'vols_inc':volumes_inc, 'angs_inc':angles_inc, 'vols_dv_inc':volume_derv_inc, 'angs_dv_inc':angle_derv_inc, "max_vol":MAX_VOLUME, 'vols_dv_dec':volume_derv_dec, 'angs_dv_dec':angle_derv_dec}
              self.data_dict[file] = curr_data_dict





  # def func(self,x,c): return np.sum([x**(len(c)-idx)*c[idx] for idx in range(0,len(c))])

  # def derv_exp(self,x,c): return np.exp(-np.sum([x**(len(c)-idx-1)*(len(c)-idx)*c[idx] for idx in range(0,len(c)-1)]))

  # def objective(self,coefs, volumes, angles,sign=1.0): return ((func(np.array(angles), coefs) - volumes) ** 2).sum() #+ np.sum([coef**2 for coef in coefs]) #+ derv_exp(np.array(angles), coefs)

  def func(self,angles,c): 
    return np.array([np.sum([x**(len(c)-idx-1)*c[idx] for idx in range(0,len(c))]) for x in angles])

  def derv_exp(self,angles,c): return np.array([np.exp(-np.sum( [x**(len(c)-idx-2)*(len(c)-idx-1)*c[idx] for idx in range(0,len(c)-1)] )) for x in angles]) #this is exp(-f'(th_k,beta)) for all k (list)



  def jacobian_calc (self,coefs,angles,volumes,sign=1.0):
    v_iters = len(volumes)
    c_iters = len(coefs)

    # J = np.zeros([v_iters,c_iters]) #just least sqrs
    #J = np.zeros([v_iters+v_iters,c_iters]) #just least sqrs
    J = np.zeros([v_iters+c_iters+v_iters,c_iters]) #just least sqrs

    for idx in range(0,v_iters):
      for jdx in range(0,c_iters):
        J[idx,jdx] = angles[idx]**(len(coefs)-jdx)
    #Now for regulator
    # for idx in range(0,c_iters):
    #   for jdx in range(0,c_iters):
    #     J[v_iters+idx,jdx] = float(idx==jdx) #diff produces 1, diag
    #Now for Exponential
    for idx in range(0,v_iters):
      for jdx in range(0,c_iters):
        # J[v_iters+c_iters+idx,jdx] = angles[idx]**(c_iters-jdx-2)*(c_iters-jdx-1)

        val = np.exp(-np.sum( [angles[idx]**(c_iters-kdx-2)*(c_iters-kdx-1)*coefs[kdx] for kdx in range(0,c_iters-1)] ))
        if (c_iters-jdx-2) >= 0:
          J[v_iters+idx,jdx] = val*(c_iters-jdx-1)*angles[idx]**(c_iters-jdx-2)
        else:
          J[v_iters+idx,jdx] = 0.0
        # J[v_iters+idx,jdx] = 0.0

        # if jdx==c_iters-1: print "last column", xangles[idx]**(c_iters-jdx-2)*(c_iters-jdx-1), "jdx,c_iters",jdx,c_iters-1
      print "\nval: ", val
      print "angle:", angles[idx]
      print "sum: ", np.sum( [angles[idx]**(c_iters-kdx-2)*(c_iters-kdx-1)*coefs[kdx] for kdx in range(0,c_iters-1)] )
      print "sum list: ", [angles[idx]**(c_iters-kdx-2)*(c_iters-kdx-1)*coefs[kdx] for kdx in range(0,c_iters-1)]
      print "coefs: ", [coefs[kdx] for kdx in range(0,c_iters)]
      print "angle powers", [angles[idx]**(c_iters-kdx-2)*(c_iters-kdx-1) for kdx in range(0,c_iters-1)]
      print " N:", [(c_iters-kdx-1) for kdx in range(0,c_iters-1)]
      print "N-1", [(c_iters-kdx-2) for kdx in range(0,c_iters-1)]      
      print "raw coefs", coefs
    print "jacobian", J

    return J


  def obj_func(self,coefs, angles, volumes, sign=1.0):
    f = self.func(np.array(angles), coefs) - volumes
    g = np.array([coef for coef in coefs]) #gets squared
    e = self.derv_exp(angles,coefs)
    # f = np.hstack([f,g])
    #these are useful when function is strictly increasing:
    #f = np.hstack([f,e])
    f = np.hstack([f,g,e])
    return f


  def fit_legendre_polynomials(self):
    #fit legendre polynomials to Vol data in domian [0,pi]
    #This fitting is for the derivative function as it is observable and this is required for online classfication (handling basis functions and observability of V)


    for file in self.data_list:
      if file not in self.outliers:
        # try:
        max_vol = self.data_dict[file]['max_vol']
        """ Increasing fit"""
        #Tyler method to insure only increasing function
        angles = self.data_dict[file]['angs_inc']
        volumes = self.data_dict[file]['vols_inc']


        #choose method of fitting
        if True:
          st = np.polynomial.polynomial.polyfit(angles,volumes,self.polynomial_degree)
          z_poly = np.polynomial.Polynomial(st)

        if False:
          x0 = list(np.full(self.polynomial_degree+1 , 0.0))
          # x0 = st.tolist()
          # res = minimize(objective, x0=x0, method='SLSQP', options=dict(maxiter=10000))
          self.opt_max_iter = 10000
          res, ier = leastsq(self.obj_func, x0, args=(angles,volumes, -1.0,))
          # res, ier = leastsq(self.obj_func, x0, args=(angles,volumes, -1.0,), maxfev=self.opt_max_iter)
          # res, ier = leastsq(self.obj_func, x0, args=(angles,volumes, -1.0,), Dfun=self.jacobian_calc)#, maxfev=self.opt_max_iter)
          print "the coef: ", res
          # z_poly = np.polynomial.Polynomial(res.x)
          z_poly = np.polynomial.Polynomial(res)
          # z_poly = np.polynomial.Polynomial(x0)


        domain =[0,self.domain_end]
        s = np.polynomial.legendre.Legendre.cast(z_poly, window=domain, domain=domain)


        # if res.success:
        # s = np.polynomial.Polynomial(res.x)  #this is time series
        # s = np.polynomial.legendre.Legendre(coef_v_inc,domain=[0,self.domain_end], window=[0,self.domain_end])
        ds = s.deriv(1) #first derivative series
        dds = s.deriv(2) #first derivative series
        alpha = s.coef # coeff for V
        d_alpha= ds.coef # coeff for dV
        dd_alpha = dds.coef # coeff for ddV
        self.data_dict[file]['poly_v_inc'] = s  #store series object for further manipulation (may need to save this)
        self.data_dict[file]['poly_dv_inc'] = ds  #store series object for further manipulation (may need to save this)
        self.data_dict[file]['poly_ddv_inc'] = dds
        self.data_dict[file]['alpha_v_inc'] = alpha
        self.data_dict[file]['alpha_dv_inc'] = d_alpha
        self.data_dict[file]['alpha_ddv_inc'] = dd_alpha
        #print "res success, file:", file
        # else:
        #   print "data failed in fitting:",file
        #   self.fail_list.append(file)
        # except:
        #   print "data failed:",file
        #   self.fail_list.append(file)
    print "list of all fails: ", self.fail_list


  def Evaulate_rmse_fitting(self):
    rmse_obj = RMSE_Eval() #contians .rmse_list and .poly_degree
    num_valid_containers = 0
    rmse_max = 0.0
    file_max = 'no file'
    for file in self.data_list:
      if file not in self.outliers and file not in self.rmse_keep_out_list:
        curr_coef = self.data_dict[file]['poly_v_inc']
        #get ground truth:
        x = self.data_dict[file]['angs_inc']
        y = self.data_dict[file]['vols_inc']
        # y_pred = np.polyval(curr_coef.coef,x)
        y_pred = np.polynomial.legendre.legval(x,curr_coef.coef)
        e = y-y_pred
        es = e**2
        mse = np.mean(es)
        rmse = np.sqrt(mse)
        if rmse > 15: print "file with greater than 15rmse is: ", file; self.rmse_keep_out_list.append(file)
        if rmse < 15:
          if rmse > rmse_max: rmse_max = rmse; file_max = file
          rmse_obj.rmse_list.append(rmse)
          rmse_obj.poly_degree.append(self.polynomial_degree)
          num_valid_containers += 1
        #if "023" in file: print "FILE 23 PRESENT"
    print "number of valid containers: ", num_valid_containers
    print "rmse keep out list", self.rmse_keep_out_list
    print "max rmse and file: ", rmse_max, file_max
    return rmse_obj

  def return_data_for_plotting_example(self):
      for file in self.data_list:
          if file not in self.outliers:
              if "021" in file: 
                curr_coef = self.data_dict[file]['poly_v_inc']
                #get ground truth:
                x = self.data_dict[file]['angs_inc']
                y = self.data_dict[file]['vols_inc']
                xtest = np.linspace(x[0],np.max(x),200)
                print "last x real", float(x[-1]), "last x test", xtest[-1]
                y_pred = np.polynomial.legendre.legval(xtest,curr_coef.coef)
                return x,y,xtest,y_pred

def main():
  parser = argparse.ArgumentParser(description="Analyze and plot clusters in pouring data")
  parser.add_argument("-p", "--plot_data", action="store_true")
  parser.add_argument("-s", "--store_data", action="store_true")
  # subparsers = parser.add_subparsers(); parser_clusters = subparsers.add_parser("cluster"); parser_clusters.set_defaults(func=cluster)

  args = parser.parse_args()


  cls_obj = Container_classifier_class()
  #1.Read in data
  cls_obj.read_in_data()
  rmse_net_list = []
  # poly_orders_list = [3,5,7,9,11,13,15]
  poly_orders_list = [5,7,9,11,13,15]
  for poly_order in poly_orders_list:
    cls_obj.polynomial_degree = poly_order
    #2a.Fit Legendre polynomials to V  (derivative coefficients found immediately from numpy.polynomial.legendre.legder: https://docs.scipy.org/doc/numpy/reference/generated/numpy.polynomial.legendre.legder.html#numpy.polynomial.legendre.legder)
    cls_obj.fit_legendre_polynomials()
    robj = cls_obj.Evaulate_rmse_fitting()
    rmse_net_list.append(robj)
    print "finished poly order", poly_order

  print "rmse values", robj.rmse_list, "\n min ", np.min(robj.rmse_list)

  poly_orders = []
  rmse_vals = []
  for idx in range(len(rmse_net_list)):
    poly_orders = np.concatenate([poly_orders, rmse_net_list[idx].poly_degree])
    # rmse_vals = np.concatenate([rmse_vals, rmse_net_list[idx].rmse_list])
    rmse_vals.append(rmse_net_list[idx].rmse_list)

  #Now make a violin plot for these
  sns.set(style="whitegrid")
  # sns.violinplot(x=poly_orders, y=rmse_vals)
  boxprops = dict(linewidth=5)
  # boxprops = dict(linestyle='--', linewidth=3, color='darkgoldenrod')
  flierprops = dict(marker='o', markerfacecolor='green', markersize=3, linestyle='none')
  medianprops = dict(linestyle='-.', linewidth=2.5, color='firebrick')
  meanpointprops = dict(marker='D', markeredgecolor='black', markerfacecolor='firebrick', markersize=12)
  meanlineprops = dict(linestyle='--', linewidth=2.5, color='purple')
  fig_size_tuple = (13,9)
  plot_axis_text_size = 28
  fig = plt.figure(figsize=fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
  ax = fig.gca()
  ax.boxplot(rmse_vals,labels=poly_orders_list, whis='range',boxprops=boxprops, meanprops=meanpointprops, flierprops=flierprops, medianprops=medianprops, meanline=False, showmeans=True)
  ax.set_xlabel("Polynomial Degree", fontsize=plot_axis_text_size)
  ax.set_ylabel("RMSE (ml)", fontsize=plot_axis_text_size)
  ax.tick_params(labelsize=plot_axis_text_size)
  #plt.show()

  #Now plot the example data
  cls_obj.polynomial_degree = 9
  cls_obj.fit_legendre_polynomials()
  xtrue,ytrue,xpred,ypred = cls_obj.return_data_for_plotting_example()
  fig2 = plt.figure(figsize=fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
  ax2 = fig2.gca()
  ax2.scatter(xtrue,ytrue,linewidth=2.5, label="Raw Data")
  ax2.plot(xpred,ypred,linewidth=2.5, label="Fitted data")
  ax2.set_xlabel("$\\theta$ (rad)", fontsize=plot_axis_text_size)
  ax2.set_ylabel("$V$ (ml)", fontsize=plot_axis_text_size)
  ax2.tick_params(labelsize=plot_axis_text_size)
  plt.show()


if __name__ == '__main__':
  main()

