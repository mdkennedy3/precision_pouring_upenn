#!/usr/bin/env python2
#Created by Monroe Kennedy III and Tyler Larkworthy   All Rights Reserved
import rospy
import numpy as np
import rospkg
import os
import json
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D

#import matplotlib
#matplotlib.rcParams['ps.useafm'] = True
#matplotlib.rcParams['pdf.use14corefonts'] = True
#matplotlib.rcParams['text.usetex'] = True

from sklearn import mixture
from sklearn.decomposition import PCA
import sys #to get size of variable sys.getsizeof(s) in bits
import argparse

import cPickle as pickle


from pouring_unknown_geom.tyler_code import curve_fitting as tyler_curve_fit




class args_cls(object):
  """Neccessary for tyler code """
  def __init__(self, derivative=0, normalized=False, inverted=False, scaled=False):
    self.derivative = derivative
    self.normalized = False
    self.inverted = False
    self.scaled = False


class Container_classifier_class(object):
  """docstring for ClassName"""
  def __init__(self):
    # self.arg = arg
    self.polynomial_degree = 9#9 #7 is best for V_dec curve
    self.domain_end = 2.4#2.5#np.pi#2.5#2.3#1.65#2.0  #vs np.pi
    #5th/6th order allows us to keep "006", for 7 order it must be excluded (however the data is better fit by 7th order)
    self.rospack = rospkg.RosPack()
    self.data_path = self.rospack.get_path("pouring_unknown_geom") + '/data/'
    self.data_list = ["001","002","003","004","005","006",
                      "007","008","009","010","011","012",
                      "013","014","015","016","017","018",
                      "019","020","021","022","023"]#,'S000', 'S001', 'S002', 'S003', 'S004', 'S005', 'S006', 'S007', 'S008', 'S009', 'S010', 'S011', 'S012', 'S013', 'S014', 'S015', 'S016', 'S017', 'S018', 'S019', 'S020', 'S021', 'S022', 'S023', 'S024', 'S025', 'S026', 'S027', 'S028', 'S029', 'S030', 'S031', 'S032', 'S033', 'S034', 'S035', 'S036', 'S037', 'S038', 'S039', 'S040', 'S041', 'S042', 'S043', 'S044', 'S045', 'S046', 'S047', 'S048', 'S049', 'S050', 'S051', 'S052', 'S053', 'S054', 'S055', 'S056', 'S057', 'S058', 'S059', 'S060', 'S061', 'S062', 'S063', 'S064', 'S065', 'S066', 'S067', 'S068', 'S069', 'S070', 'S071', 'S072', 'S073', 'S074', 'S075', 'S076', 'S077', 'S078', 'S079', 'S080', 'S081', 'S082', 'S083', 'S084', 'S085', 'S086', 'S087', 'S088', 'S089', 'S090', 'S091', 'S092', 'S093', 'S094', 'S095', 'S096', 'S097', 'S098', 'S099', 'S100', 'S101', 'S102', 'S103', 'S104', 'S105', 'S106', 'S107', 'S108', 'S109', 'S110', 'S111', 'S112', 'S113', 'S114', 'S115', 'S116', 'S117', 'S118', 'S119', 'S120', 'S121', 'S122', 'S123', 'S124', 'S125', 'S126', 'S127', 'S128', 'S129', 'S130', 'S131', 'S132', 'S133', 'S134', 'S135', 'S136', 'S137', 'S138', 'S139', 'S140', 'S141', 'S142', 'S143', 'S144', 'S145', 'S146', 'S147', 'S148', 'S149', 'S150', 'S151', 'S152', 'S153', 'S154', 'S155', 'S156', 'S157', 'S158', 'S159', 'S160', 'S161', 'S162', 'S163', 'S164', 'S165', 'S166', 'S167', 'S168', 'S169', 'S170', 'S171', 'S172', 'S173', 'S174', 'S175', 'S176', 'S177', 'S178', 'S179', 'S180', 'S181', 'S182', 'S183', 'S184', 'S185', 'S186', 'S187', 'S188', 'S189', 'S190', 'S191', 'S192', 'S193', 'S194', 'S195', 'S196', 'S197', 'S198', 'S199', 'S200', 'S201', 'S202', 'S203', 'S204', 'S205', 'S206', 'S207', 'S208', 'S209', 'S210', 'S211', 'S212', 'S213', 'S214', 'S215', 'S216', 'S217', 'S218', 'S219', 'S220', 'S221', 'S222', 'S223', 'S224', 'S225', 'S226', 'S227', 'S228', 'S229', 'S230', 'S231', 'S232', 'S233', 'S234', 'S235', 'S236', 'S237', 'S238', 'S239', 'S240', 'S241', 'S242', 'S243', 'S244', 'S245', 'S246', 'S247', 'S248', 'S249', 'S250', 'S251', 'S252', 'S253', 'S254', 'S255', 'S256', 'S257', 'S258', 'S259', 'S260', 'S261', 'S262', 'S263', 'S264', 'S265', 'S266', 'S267', 'S268', 'S269', 'S270', 'S271', 'S272', 'S273', 'S274', 'S275', 'S276', 'S277', 'S278', 'S279', 'S280', 'S281', 'S282', 'S283']


    #hold these as their curves are incomplete
    self.outliers = ["000", "011", "013", "018", "019", "014", "S266","S002","S007","S038","S045","S048","S049","S051","S074","S076","S078","S080","S084","S098","S103", "S105",'S106', 'S107', 'S109', 'S110', 'S111', 'S112', 'S113', 'S126', 'S129', 'S130', 'S131', 'S133', 'S136', 'S138', 'S140', 'S141', 'S142', 'S143', 'S145', 'S146', 'S147', 'S148', 'S150', 'S158', 'S159', 'S161', 'S163', 'S165', 'S166', 'S167', 'S168', 'S169', 'S170', 'S171', 'S172', 'S173', 'S174', 'S175', 'S176', 'S177', 'S180', 'S186', 'S190', 'S191', 'S193', 'S194', 'S195', 'S197', 'S198', 'S200', 'S201', 'S202', 'S203', 'S204', 'S205', 'S206', 'S208', 'S209', 'S210', 'S211', 'S212', 'S236', 'S257', 'S259', 'S260', 'S271', 'S272', 'S275', 'S276']  #014, #006 didn't have enough near the beginning and this caused very weird angles
    # self.outliers = ["000", "011", "013", "018", "019", "014", "004","005","006",
    #                   "007","008","009","010","011","012",
    #                   "013","014","015","016","017","018",
    #                   "019","020","021","022","023"]

    self.fail_list = []
    self.legend_text_size = 20
    self.plot_axis_text_size = 28

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
              # angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc,  last_ang = self.filter_volume_data(curr_data, MAX_VOLUME, last_ang=0, angles_dec=[], volumes_dec=[])
              angles_dec, volumes_dec, angles_inc,  volumes_inc, angle_derv_inc, volume_derv_inc, angle_derv_dec, volume_derv_dec,  last_ang = self.filter_volume_data(curr_data, MAX_VOLUME, last_ang=0, angles_dec=[], volumes_dec=[])

              #tyler process file
              args = args_cls()
              nangles, nvolumes, nlast_ang = tyler_curve_fit.process_file(curr_dir, fn, MAX_VOLUME, last_ang=last_ang, angles=[], volumes=[])
              angles_dec, volumes_dec = nangles, nvolumes
              # print "\n\nangles: ", angles_dec
              # print "\nvolumes: ", volumes_dec
              args_inc = args_cls(inverted=True)
              angles_inc, volumes_inc = tyler_curve_fit.process_directory(curr_dir, args_inc)



              curr_data_dict = {'angs_dec':angles_dec, 'vols_dec':volumes_dec, 'vols_inc':volumes_inc, 'angs_inc':angles_inc, 'vols_dv_inc':volume_derv_inc, 'angs_dv_inc':angle_derv_inc, "max_vol":MAX_VOLUME, 'vols_dv_dec':volume_derv_dec, 'angs_dv_dec':angle_derv_dec}
              self.data_dict[file] = curr_data_dict

    # print "data: ", self.data_dict

  def fit_legendre_polynomials(self):
    #fit legendre polynomials to Vol data in domian [0,pi]
    #This fitting is for the derivative function as it is observable and this is required for online classfication (handling basis functions and observability of V)
    for file in self.data_list:
      if file not in self.outliers:
        try:
          #may need to do derivatives up front  (problem in comparing a fit to a zeroeth order vs other coefficients..., and V is not observable...)
          # x = self.data_dict[file]['angs_dv_dec']
          # y = self.data_dict[file]['vols_dv_dec']
          # ds = np.polynomial.legendre.Legendre.fit(x,y,self.polynomial_degree,domain=[0,self.domain_end], window=[0,self.domain_end])
          max_vol = self.data_dict[file]['max_vol']
          # x = self.data_dict[file]['angs_dec']
          # y = self.data_dict[file]['vols_dec']
          
          """DOUBLE CHECK WINDOW VS DOMAIN, IF WINDOW IS FOR INTEGRATION AND DOMAIN FOR MAPPING OR VICE VERSA """
          #Fitting with regular polynomial type: 
          # dv_coeff = np.polynomial.polynomial.polyfit(x,y,self.polynomial_degree)
          # power_series_dv = np.polynomial.polynomial.Polynomial(dv_coeff,domain=[0,self.domain_end], window=[0,self.domain_end])
          # power_series_v = power_series_dv.integ(m=1,k=[max_vol])
          # ds = power_series_dv.convert(domain=[0,self.domain_end], window=[0,self.domain_end], kind=np.polynomial.legendre.Legendre)
          # s = power_series_v.convert(domain=[0,self.domain_end], window=[0,self.domain_end], kind=np.polynomial.legendre.Legendre)


          """ Increasing fit"""
          #Tyler method to insure only increasing function
          x = self.data_dict[file]['angs_inc']
          y = self.data_dict[file]['vols_inc']
          # s = np.polynomial.legendre.Legendre.fit(x,y,self.polynomial_degree,domain=[0,self.domain_end], window=[0,self.domain_end])
          args_v_inc = args_cls();#normalized = True
          coef_v_inc = tyler_curve_fit.fit_curve(x,y,fn=file,domain=[0,self.domain_end], poly_order=self.polynomial_degree, args=args_v_inc)
          # args_dv_inc = args_cls(derivative=1); coef_dv_inc = tyler_curve_fit.fit_curve(x,y,fn=file,domain=[0,self.domain_end], poly_order=self.polynomial_degree, args=args_dv_inc)
          # args_ddv_inc = args_cls(derivative=2); coef_ddv_inc = tyler_curve_fit.fit_curve(x,y,fn=file,domain=[0,self.domain_end], poly_order=self.polynomial_degree, args=args_ddv_inc)

          s = np.polynomial.legendre.Legendre(coef_v_inc,domain=[0,self.domain_end], window=[0,self.domain_end])  #This is an increasing function

          # s = s_l.convert(kind=np.polynomial.polynomial.Polynomial, domain=[0,self.domain_end], window=[0,self.domain_end])
          ds = s.deriv(1) #first derivative series
          dds = s.deriv(2) #first derivative series
          # ds = np.polynomial.legendre.Legendre(coef_dv_inc,domain=[0,self.domain_end], window=[0,self.domain_end])  #This is an increasing function
          # dds = np.polynomial.legendre.Legendre(coef_ddv_inc,domain=[0,self.domain_end], window=[0,self.domain_end])  #This is an increasing function

          # print "\n\n power series coeff s: ", s.coef, " \n coeff ds: ", ds.coef
          # print "\n\n power series coeff s(1): ", np.sum(s.coef), " \n coeff ds(1): ", np.sum(ds.coef)


          # s = ds.integ(m=1,k=[max_vol]) #first derivative series
          # dds = ds.deriv(1) #second derivative series
          # print "\n\nproblem is here: ", s, " for file: ", file
          # ds = s.deriv(1) #first derivative series
          # dds = s.deriv(2) #second derivative series

          alpha = s.coef # coeff for V
          d_alpha= ds.coef # coeff for dV
          dd_alpha = dds.coef # coeff for ddV


          self.data_dict[file]['poly_v_inc'] = s  #store series object for further manipulation (may need to save this)
          self.data_dict[file]['poly_dv_inc'] = ds  #store series object for further manipulation (may need to save this)
          self.data_dict[file]['poly_ddv_inc'] = dds
          self.data_dict[file]['alpha_v_inc'] = alpha
          self.data_dict[file]['alpha_dv_inc'] = d_alpha
          self.data_dict[file]['alpha_ddv_inc'] = dd_alpha

          """Decreasing Fit"""
          # print "\n\n\nds", ds.coef
          ds_inc_pow = ds.convert(kind=np.polynomial.polynomial.Polynomial, domain=[0,self.domain_end], window=[0,self.domain_end])
          # print "ds_pow", ds_inc_pow.coef
          ds_dec_pow_coef = -ds_inc_pow.coef
          ds_dec_pow = np.polynomial.polynomial.Polynomial(ds_dec_pow_coef, domain=[0,self.domain_end], window=[0,self.domain_end])
          s_dec_pow = ds_dec_pow.integ(m=1,k=[max_vol]) #max_vol
          # print "\ncurrent max volume:", max_vol, "\n but power series coeff", s_dec_pow.coef

          s_dec = s_dec_pow.convert(kind=np.polynomial.legendre.Legendre, domain=[0,self.domain_end], window=[0,self.domain_end])
          ds_dec = ds_dec_pow.convert(kind=np.polynomial.legendre.Legendre, domain=[0,self.domain_end], window=[0,self.domain_end])

          # ds_dec_coef = -ds.coef
          # ds_dec = np.polynomial.legendre.Legendre(ds_dec_coef, domain=[0,self.domain_end], window=[0,self.domain_end])
          # s_dec = ds_dec.integ(m=1,k=[max_vol])
          dds_dec = ds_dec.deriv(1)
          alpha_dec = s_dec.coef
          d_alpha_dec = ds_dec.coef
          dd_alpha_dec = dds_dec.coef
          self.data_dict[file]['poly_v_dec'] = s_dec #s_dec_pow  #store series object for further manipulation (may need to save this)
          self.data_dict[file]['poly_dv_dec'] = ds_dec  #store series object for further manipulation (may need to save this)
          self.data_dict[file]['poly_ddv_dec'] = dds_dec
          self.data_dict[file]['alpha_v_dec'] = alpha_dec
          self.data_dict[file]['alpha_dv_dec'] = d_alpha_dec
          self.data_dict[file]['alpha_ddv_dec'] = dd_alpha_dec
        except:
          print "data failed:",file
          self.fail_list.append(file)
    print "list of all fails: ", self.fail_list




  def normalize_data_coeff(self):
    #this function, find the normalized coefficients for each data
    for file in self.data_list:
      if file not in self.outliers:
        #extract coefficients from decreasing function
        alpha_v = np.array(self.data_dict[file]["alpha_v_dec"]) 
        alpha_dv = np.array(self.data_dict[file]["alpha_dv_dec"]) 
        alpha_ddv = np.array(self.data_dict[file]["alpha_ddv_dec"])  #Stacking these didn't provide additional information

        #for increasing function
        # alpha_v = np.array(self.data_dict[file]["alpha_v_inc"]) 
        # alpha_dv = np.array(self.data_dict[file]["alpha_dv_inc"]) 
        # alpha_ddv = np.array(self.data_dict[file]["alpha_ddv_inc"])  #Stacking these didn't provide additional information

        #concantenate to make one large vector and normalize
        # alpha_vect = np.hstack([alpha_dv, alpha_ddv]) 
        # alpha_vect_normed= np.divide(alpha_vect, np.linalg.norm(alpha_vect))
        #store this vector and its normal
        # self.data_dict[file]["alpha_dv_ddv"] = alpha_vect
        # self.data_dict[file]["alpha_dv_ddv_normed"] = alpha_vect_normed
        alpha_v_normed = np.divide(alpha_v, np.linalg.norm(np.array(alpha_v)))*np.sign(alpha_v[0])
        alpha_dv_normed = np.divide(alpha_dv, np.linalg.norm(np.array(alpha_dv)))
        alpha_ddv_normed = np.divide(alpha_dv, np.linalg.norm(np.array(alpha_ddv)))
        self.data_dict[file]["alpha_v_normed"] = alpha_v_normed
        self.data_dict[file]["alpha_dv_normed"] = alpha_dv_normed
        self.data_dict[file]["alpha_ddv_normed"] = alpha_ddv_normed

  def classify_hmm(self, num_clusters):
    #obtain clusters using Hidden Markov Model (unsupervised clustering), requires the number of clusters and then provides cluster centroids, as well as covariances (and hence scores for test values)
    #Store two things: 1. matrix with each column being intercept, then normalized alpha_vector 2. trained hmm classifier object
    #This clustering produces coefficients for V_dec

    if False:
      pass
      # samples is 1000,2 np.array
      # gmix = mixture.GaussianMixture(n_components=2, covariance_type='full'); gmix.fit(samples); print gmix.means_; gmix.means_  the rows are the centers [x,y,...]; gmix.covariances_ are array of covariance (row matching);
      # # Then use gmix.predict;
      # colors = ['r' if i==0 else 'g' for i in gmix.predict(samples)]; ax = plt.gca(); ax.scatter(samples[:,0], samples[:,1], c=colors, alpha=0.8); plt.savefig("class.png");

    #input array of samples, with x axis being different data, and y axis being components in array
    alpha_vect_mat = []
    file_list = []
    for file in self.data_list:
      if file not in self.outliers:
        alpha_vect_normed = self.data_dict[file]["alpha_v_normed"]  #normalize the V_dec function
        # alpha_vect_normed = self.data_dict[file]["alpha_dv_normed"] #normalize the dV_dec function
        # alpha_vect_normed = self.data_dict[file]["alpha_ddv_normed"] #normalize the ddV_dec function
        # print "shape of individual apha", alpha_vect_normed.shape
        alpha_vect_mat.append(alpha_vect_normed)
        file_list.append(file)
    #stack the alpha's, now each column is the coefficient for the corresponding file in file_list list
    alpha_vect_mat = np.vstack(alpha_vect_mat)
    # print "alpha mat: ", alpha_vect_mat

    #Fit to Guassian Mixture Model
    gmix = mixture.GaussianMixture(n_components=num_clusters, covariance_type='full')
    gmix.fit(alpha_vect_mat)  #each data point is on x axis, y axis is components of each data point
    cluster_means_row_major = gmix.means_
    print "the cluster poly coeff: ", cluster_means_row_major, "\n shape: ", cluster_means_row_major.shape
    cluster_cov = gmix.covariances_
    cluster_data = {"cluster_obj":gmix, "cluster_centers_row_major":cluster_means_row_major, "cluster_cov":cluster_cov}
    self.data_dict["clusters"] = cluster_data

  def find_cls_center_roots(self):
    #Given the clusters, obtain the respective intercepts, as these intercepts define region of usefulness for each of these classification curves  #NOTE: INTERCEPTS ARE MEANINGLESS FOR DV function, instead intercepts must be obtained from V function  **WHAT DOES THIS MEAN FOR FITTING ETC?***
    

    def obtain_intercept_numerically(sk):
      #this function numerically finds the intercepts for each curve as the roots is not returning real values, so this is done by performing linspace then finding values closest to 0, and storing in list, of the candidates within the 
      #threshold, only the first whose th value is >0 (& less then domain limit) is used as the root and returned
      th,vol = sk.linspace(1000)
      vol_min = vol[0]
      th_min = th[0]
      min_idx = 0
      int_update_bool = False
      vol_thresh = 0.01 #ml
      for idx in range(len(vol)):
        vol_curr = vol[idx]
        th_curr = th[idx]
        if np.abs(vol_curr) < vol_min and th_curr >= 0 and th_curr <= self.domain_end:
          vol_min = vol_curr
          th_min = th_curr
          min_idx = idx
          int_update_bool = True

      intercept = [th_min, vol_min]
      return intercept, int_update_bool
      


    def obtain_v_inc_from_v_dec(sk):
      #this function takes a decreasing volume function polynomial class series, and converts it to increasing using the following steps: 
      #1. Differentiate alpha_v_dec_k, 
      sk_dv_dec = sk.deriv(1)
      #2. Then: alpha_dv_dec_k = -alpha_dv_inc_k
      sk_dv_inc_coeff = -sk_dv_dec.coef
      sk_dv_inc= np.polynomial.legendre.Legendre(sk_dv_inc_coeff,domain=[0,self.domain_end], window=[0,self.domain_end])
      #3. Then integrate with intercept c1 = 0:  alpha_v_inc_k  , return this series
      sk_v_inc = sk_dv_inc.integ(m=1,k=[0])  #integrate with intercept being th=0
      #issue may be that there must only be one intercept, but this should be the case unless functions are not well behaved even in decreasing case (use this to choose poly degree appropriately)
      return sk_v_inc



    #1. For set of coefficients, generate a series object in specified window and For each cluster center Vk_dec, find the roots,
    series = []
    for idx in range(self.data_dict['clusters']['cluster_centers_row_major'].shape[0]):
      #obtain the coefficient
      coeff_curr = self.data_dict['clusters']['cluster_centers_row_major'][idx,:]
      #Fit this to a series
      sk = np.polynomial.legendre.Legendre(coeff_curr, domain=[0,self.domain_end], window=[0,self.domain_end])
      # print "all the roots: ", sk.roots()
      intercept = sk.roots()[0]  #this should be singular value, but should fit in the range of data, so just take first intercept (plot these curves as idiot check)
      intercept_pt, int_update_bool = obtain_intercept_numerically(sk)
      th_intercept = intercept_pt[0]
      print "\n\nthe intercepts: ", th_intercept, ", for idx: ", idx
      sk_inc = obtain_v_inc_from_v_dec(sk)
      cluster_info = {"cluster_series_v_inc":sk_inc, 'cluster_series_intercept':th_intercept}
      series.append(cluster_info)

    self.data_dict['clusters']['clusters_inc'] = series

    print "cluster series: ", series


    #Plot the roots to determine which are appropriate (problem is that dV may intercept multiple times, instead, need to look at V functions and cluster with those and use these intercepts instead??)

  def plot_volume_curves(self, raw_decreasing=True, raw_increasing=True, raw_derivative_increasing=True):
    #Plot the following
    #a) plot raw data input
    #b) plot PCA of legendre coefficients
    #c) plot unnormalized and normalized legendre poly for all containers
    #d) plot cluster centers for legendre coeff

    #plot the volume curve for each of the data labeling the data, and perhaps color codeing for different types? (but needs to be distinguishable)
    def plot_raw_decreasing_volume():
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      # ax = fig.gca(projection='3d')
      ax = fig.gca()
      for file in self.data_list:
        if file not in self.outliers:
          angs = self.data_dict[file]['angs_dec']
          vols = self.data_dict[file]['vols_dec']
          ax.scatter(angs, vols, alpha=0.2)
          # ax.scatter(angs, vols, alpha=0.8, label=file) # print "\n\ngetting ready to plot this decreasing function: ", self.data_dict[file]['poly_v_dec'].coef # print "\ngetting ready to plot this derivative decreasing function: ", self.data_dict[file]['poly_dv_dec'].coef # print "\n comparre this to the  derivative of the increasing function", self.data_dict[file]['poly_dv_inc'].coef
          x,y = self.data_dict[file]['poly_v_dec'].linspace(100)#,domain=[0,self.domain_end])
          xtrunc = [x[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          ytrunc = [y[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          # print "x,y:",x,"\n",y
          # ax.plot(x,y, label=file)
          ax.plot(xtrunc,ytrunc, label=file)
          # plt.plot(angs, vols, label=(file+'raw'))
      ax.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
      ax.set_xlim(0,self.domain_end)#np.pi)
      ax.set_ylim(-10,1000)
      ax.set_ylabel('$V_{max}$ (ml)', fontsize=self.plot_axis_text_size)
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0. ,prop={'size': self.legend_text_size})
      # plt.show()
      plt.savefig("volume_curves_decreasing.png",bbox_inches='tight')
    def plot_raw_increasing_volume():
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      # ax = fig.gca(projection='3d')
      ax = fig.gca()
      for file in self.data_list:
        if file not in self.outliers:
          angs = self.data_dict[file]['angs_inc']
          vols = self.data_dict[file]['vols_inc']
          ax.scatter(angs, vols, alpha=0.2)
          # plt.plot(angs, vols, label=file)
          x,y = self.data_dict[file]['poly_v_inc'].linspace(100)#,domain=[0,self.domain_end])
          xtrunc = [x[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          ytrunc = [y[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          # print "x,y:",x,"\n",y
          # ax.plot(x,y, label=file)
          ax.plot(xtrunc,ytrunc, label=file)

      ax.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
      ax.set_xlim(0,self.domain_end)#np.pi)
      ax.set_ylim(0,1100)
      ax.set_ylabel('$V_{max}$ (ml)', fontsize=self.plot_axis_text_size)
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
      # plt.show()
      plt.savefig("volume_curves_increasing_fit.png",bbox_inches='tight')
    def plot_raw_derivative_volume():
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      # ax = fig.gca(projection='3d')
      ax = fig.gca()
      for file in self.data_list:
        if file not in self.outliers:
          angs = self.data_dict[file]['angs_dv_inc']
          vols = self.data_dict[file]['vols_dv_inc']
          # angs = self.data_dict[file]['angs_dv_dec']
          # vols = self.data_dict[file]['vols_dv_dec']
          ax.scatter(angs, vols, alpha=0.2)
          # plt.plot(angs, vols, label=file)
          # x,y = self.data_dict[file]['poly_dv_dec'].linspace(100,domain=[0,self.domain_end])
          x,y = self.data_dict[file]['poly_dv_inc'].linspace(100,domain=[0,self.domain_end])
          xtrunc = [x[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          ytrunc = [y[mdx] for mdx in range(len(x)) if x[mdx] <= angs[-1]]
          # print "x,y:",x,"\n",y
          # ax.plot(x,y, label=file)
          ax.plot(xtrunc,ytrunc, label=file)

      ax.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
      ax.set_xlim(0,self.domain_end)#np.pi)
      ax.set_ylim(-500,2000)
      ax.set_ylabel('$dV/d\\theta$ (ml)', fontsize=self.plot_axis_text_size)
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
      # plt.show()
      plt.savefig("volume_derivative_curves_increasing.png",bbox_inches='tight')
    def plot_normalized_coefficients():
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      ax = fig.gca()
      alpha_vect_mat = []
      file_list = []

      for file in self.data_list:
        if file not in self.outliers:
          # alpha_vect_normed = self.data_dict[file]["alpha_dv_ddv_normed"]
          # alpha_vect_normed = self.data_dict[file]["alpha_dv_normed"]
          alpha_vect_normed = self.data_dict[file]["alpha_v_normed"]
          # print "shape of individual apha", alpha_vect_normed.shape
          alpha_vect_mat.append(alpha_vect_normed)
          file_list.append(file)
      #stack the alpha's, now each column is the coefficient for the corresponding file in file_list list
      alpha_vect_mat = np.vstack(alpha_vect_mat)

      x_count = range(len(file_list)) #generate list, so each data has its own x value

      # print "number of files", len(file_list)
      # print "poly degree: ", self.polynomial_degree
      # print "checking shape of alpha mat: ", alpha_vect_mat.shape
      # print "commanded length: ", 2*(self.polynomial_degree+1) - 1
      print "file list: ", file_list
      # for idx in range(2*(self.polynomial_degree+1) - 1):
      for idx in range(self.polynomial_degree+1):
        ax.plot(x_count, alpha_vect_mat[:,idx], label=("a"+str(idx)))
        ax.scatter(x_count, alpha_vect_mat[:,idx], alpha=0.2, c='k')
      ax.set_xlabel("data")
      ax.set_ylabel("coefficient value")
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
      plt.savefig("normed_coefficients_for_analysis_for_clustering.png",bbox_inches='tight')

      #plot combinations for 3D
      fig2 = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      ax2 = fig2.gca(projection='3d')
      ax2.scatter(alpha_vect_mat[:,0],alpha_vect_mat[:,1],alpha_vect_mat[:,2],c='k')
      ax2.scatter(alpha_vect_mat[:,3],alpha_vect_mat[:,4],alpha_vect_mat[:,5],c='b')
      ax2.scatter(alpha_vect_mat[:,0],alpha_vect_mat[:,1],alpha_vect_mat[:,6],c='g')
      ax2.set_xlabel('$a_{0,k}$,$a_{3,b}$,$a_{0,g}$')
      ax2.set_ylabel('$a_{1,k}$,$a_{4,b}$,$a_{1,g}$')
      ax2.set_zlabel('$a_{2,k}$,$a_{5,b}$,$a_{6,g}$')
      ax.tick_params(labelsize=self.plot_axis_text_size)
      # plt.show()
      plt.savefig("normed_coefficients_for_analysis_for_clustering_3D.png",bbox_inches='tight')

    def plot_gmm_centroid_functions_and_intercepts():
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      ax = fig.gca()

      series = self.data_dict['clusters']['clusters_inc']
      # cluster_info = {"cluster_series_v_inc":sk_inc, 'cluster_series_intercept':intercept}
      #iterate through each cluster, plot points and intercepts as a seperate color/marker type
      for idx in range(len(series)):
        curr_cluster = series[idx]['cluster_series_v_inc']  #cluster obj
        curr_intercept = series[idx]['cluster_series_intercept'] #cluster intercept



        #from cluster center obtain points
        x,y = curr_cluster.linspace(100)#, domain=[0,self.domain_end])
        th_int_x, th_int_y = curr_intercept, 0.0  

        xtrunc = [x[mdx] for mdx in range(len(x)) if x[mdx] <= curr_intercept]
        ytrunc = [y[mdx] for mdx in range(len(x)) if x[mdx] <= curr_intercept]
        # xtrunc = [x[mdx] for mdx in range(len(x)) if x[mdx] <= self.domain_end]
        # ytrunc = [y[mdx] for mdx in range(len(x)) if x[mdx] <= self.domain_end]
        

        ax.plot(xtrunc,ytrunc,label=('$c\\_$'+str(idx)))
        # ax.plot(x,y,label=('c_'+str(idx)))

        ax.scatter(th_int_x,th_int_y, label=("$int\\_$"+str(idx)))
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
      # plt.show()
      #need to choose intercept that is not just first, but also is >0
      ax.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
      ax.set_ylabel('$V\\_{max}$ (ml)', fontsize=self.plot_axis_text_size)
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.savefig("gmm_centroid_fncts_and_intercept.png",bbox_inches='tight')


    def plot_gmm_centroid_functions_decreasing():
      '''Also plot the decreasing functino for the centroids to visualize where the intercepts are '''
      fig = plt.figure(num=None, figsize=(28,10), dpi=80, facecolor='w', edgecolor='k')
      # ax = fig.gca(projection='3d')
      ax = fig.gca()
      print "entered plot gmm cnt dec"
      for idx in range(self.data_dict['clusters']['cluster_centers_row_major'].shape[0]):
        coeff_curr = self.data_dict['clusters']['cluster_centers_row_major'][idx,:]
        #Fit this to a series
        sk = np.polynomial.legendre.Legendre(coeff_curr, domain=[0,self.domain_end], window=[0,self.domain_end])
        x,y = sk.linspace(100)

        ax.plot(x,y,label=("$c\\_$"+str(idx)))
        #Plot the intercepts here
        th_int,vol_int = self.data_dict['clusters']['clusters_inc'][idx]['cluster_series_intercept'], 0
        ax.scatter(th_int,vol_int)
      ax.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
      ax.set_xlim(0,self.domain_end)#np.pi)
      ax.set_ylim(-2,2)
      ax.set_ylabel('$V_{max}$ (ml)', fontsize=self.plot_axis_text_size)
      ax.tick_params(labelsize=self.plot_axis_text_size)
      plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
      # plt.show()
      plt.savefig("cluster_curves_decreasing.png",bbox_inches='tight')




    #plot the raw decreasing function
    plot_raw_decreasing_volume()
    #Plot the raw increasing function
    plot_raw_increasing_volume()
    #Plot the raw increasing derivative
    plot_raw_derivative_volume()
    #Plot Coefficients (make subplot that shows each coefficient for every dataset on a single plot, with x axis being data, y being coefficient value, scatter...), this is neccessary for determining how many clusters, essentially how many unique graphs exist here
    plot_normalized_coefficients()
    #Plot GMM centroid curves while high lighting the theta (root) intercepts
    plot_gmm_centroid_functions_and_intercepts()
    #Plot GMM centroid curves decreasing, with theta (root) intercepts
    plot_gmm_centroid_functions_decreasing()
    
    plt.show()



  def store_classification_data(self):
    #This function should store data/struct neccessary to get GMM and perform assignment on data (this should also be developed in streamline manner, (with intelligent naming) to obtain new GMM if training data is updated)

    #path to file
    file_name = 'classifier.p'
    path_to_classification = self.rospack.get_path("pouring_unknown_geom") + '/classification/'+ file_name
    with open(path_to_classification, 'wb') as fp:
      pickle.dump(self.data_dict,fp)


    pass




def main():
  parser = argparse.ArgumentParser(description="Analyze and plot clusters in pouring data")
  parser.add_argument("-p", "--plot_data", action="store_true")
  parser.add_argument("-s", "--store_data", action="store_true")
  # subparsers = parser.add_subparsers(); parser_clusters = subparsers.add_parser("cluster"); parser_clusters.set_defaults(func=cluster)

  args = parser.parse_args()


  cls_obj = Container_classifier_class()
  #1.Read in data
  cls_obj.read_in_data()

  #2a.Fit Legendre polynomials to V  (derivative coefficients found immediately from numpy.polynomial.legendre.legder: https://docs.scipy.org/doc/numpy/reference/generated/numpy.polynomial.legendre.legder.html#numpy.polynomial.legendre.legder)
  # Find coefficients for both dV and ddV as well 
  cls_obj.fit_legendre_polynomials()

  
  #2b. For each polynomial coefficient base, store the normalized coefficients as well  (make subplot that shows each coefficient for every dataset on a single plot, with x axis being data, y being coefficient value, scatter...), this is neccessary for determining how many clusters, essentially how many unique graphs exist here
  cls_obj.normalize_data_coeff()


  #3a.Classify dV, ddV functions using HMM, saving the cluster centers, as well as the HMM object for later use (save centers and covariance models, as well as cls object)
  num_clusters = 2 # 10 for sim, 2 for real arguably 3, but definitely 2 :(
  cls_obj.classify_hmm(num_clusters)

  
  #3b. For each class polynomial, find the intercept/roots in domain [0,pi] and store this info as well (plot for clarity)
  cls_obj.find_cls_center_roots()
  
  if args.plot_data:
    #4.Make neccessary plots of data
    cls_obj.plot_volume_curves()

  if args.store_data:
    #5. Store loadable data for online classification
    cls_obj.store_classification_data()





if __name__ == '__main__':
    main()
