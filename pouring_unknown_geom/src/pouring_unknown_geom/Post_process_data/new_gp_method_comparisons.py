#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse
import rosbag, rospkg
import os
# from os import listdir
# from os.path import isfile, join
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

from pouring_msgs.msg import *
from pouring_unknown_geom import *


class PlotCovariance(object):
  """docstring for PlotCovariance"""
  def plot_point_cov(self,points, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma ellipse based on the mean and covariance of a point
    "cloud" (points, an Nx2 array).

    Parameters
    ----------
        points : An Nx2 array of the data points.
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    pos = points.mean(axis=0)
    cov = np.cov(points, rowvar=False)
    return self.plot_cov_ellipse(cov, pos, nstd, ax, **kwargs)

  def plot_cov_ellipse(self,cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip


class MethodDataStruct(object):
  """docstring for MethodDataStruct"""
  def __init__(self):
    self.trial = '001'
    self.final_vol_err = None
    self.final_vol_err_time = None
    self.pour_time = None #time for complete pour





class NewGPPlotting(object):
  """docstring for NewGPPlotting"""
  def __init__(self, dir_path=None,fig_size_tuple=(13, 8), show_dist_pts=True, show_labeled_bag_trials=False, plot_text_size=20, tick_plot_axis_text_size=20):
    rospack = rospkg.RosPack()
    if dir_path:
      self.directory_path = dir_path
    else:
      self.directory_path = rospack.get_path("pouring_unknown_geom") + '/bagfiles/' + 'cont6/150ml'
    self.directory_path = self.directory_path + '/' #add this for file indexing
    self.cls_obj_plt_cov = PlotCovariance() #for plotting covariance
    self.show_dist_pts = show_dist_pts
    self.fig_size_tuple = fig_size_tuple
    self.show_labeled_bag_trials = show_labeled_bag_trials
    self.plot_text_size = plot_text_size
    self.tick_plot_axis_text_size = tick_plot_axis_text_size

  def load_data_to_struct(self,method_struct=None, bag=None, onset_pouring_thresh_vol=10):
    #onset pouring vol is in ml
    mh_err_list = []
    t_err_list = []
    mh_list = []
    t_mh_list = []
    #read in the bag
    for topic,msg,t in bag.read_messages(topics=['/pouring_control/Hybrid_controller_data','/pouring_control/mh_mh_dot']):
      if topic in '/pouring_control/Hybrid_controller_data':
        mh_err_list.append(msg.ml_error)
        t_err_list.append(msg.header.stamp.to_sec())
      elif topic in '/pouring_control/mh_mh_dot':
        mh_list.append(msg.mh)
        t_mh_list.append(msg.header.stamp.to_sec())
    #find mh in last second
    tf_pour_end_close = t_err_list[-1]-1.0
    tpour_end_idx = np.argmin([np.abs(t - tf_pour_end_close) for t in t_err_list])
    #get median for last second
    mh_err_final_median = np.abs(np.median(mh_err_list[tpour_end_idx:])) #absolute err, overshoot will be negative value naturally

    mh_err_final_median_without_abs=np.median(mh_err_list[tpour_end_idx:])

    median_mh_err_idx = np.argsort(mh_err_list[tpour_end_idx:])[int(len(mh_err_list[tpour_end_idx:])/2)]+tpour_end_idx #wrt whole data
    t_mh_err_median = t_err_list[median_mh_err_idx]
    mh_list_final_median_idx = np.argmin([np.abs(t - t_mh_err_median) for t in t_mh_list])

    #now find the volume at the median time
    mh_final_median = mh_list[mh_list_final_median_idx]
    t_mh_list_final_median = t_mh_list[mh_list_final_median_idx]
    #now find the first idx at which this volume occurs
    min_mh_err_list = [mh - mh_final_median for mh in mh_list]

    idx_mh_greater_than_vol = np.argwhere([0 if err < 0 else err for err in min_mh_err_list]).transpose().tolist()[0]
    t_mh_first_occur = t_mh_list[idx_mh_greater_than_vol[0]]
    mh_first_occur = mh_list[idx_mh_greater_than_vol[0]]
    #now with this time, find the corresponding time and mh_err (this time can be passed directly)
    mh_err_list_adjusted_median_first_occur_idx = np.argmin([np.abs(t - t_mh_first_occur) for t in t_err_list])
    t_mh_err_median_first_occur = t_err_list[mh_err_list_adjusted_median_first_occur_idx]
    mh_err_median_first_occur  = mh_err_list[mh_err_list_adjusted_median_first_occur_idx]

    if False:
      #plot the last points
      plt.figure(1)
      plt.plot(t_mh_list,mh_list)
      plt.plot(t_mh_first_occur,mh_first_occur,'ro')
      plt.plot(t_mh_list_final_median,mh_final_median,'r*')

      #for err plot
      plt.figure(2)
      plt.plot(t_err_list,mh_err_list)
      plt.plot(t_mh_err_median_first_occur,mh_err_median_first_occur,'ro')
      plt.plot(t_mh_err_median,mh_err_final_median_without_abs,'r*')
      plt.show()


    #Now to find pour time:
    #first find idx where vol was above thresh vol (default is 10ml)
    mh_onset_pouring_idx = np.argmin([np.abs(mh - onset_pouring_thresh_vol) for mh in mh_list])
    time_onset_pouring = t_mh_list[mh_onset_pouring_idx]
    #populate struct
    method_struct.final_vol_err = mh_err_final_median
    
    method_struct.final_vol_err_time = t_mh_err_median_first_occur
    method_struct.pour_time = method_struct.final_vol_err_time - time_onset_pouring
    
    # method_struct.final_vol_err_time = t_err_list[tpour_end_idx]
    # method_struct.pour_time = method_struct.final_vol_err_time - time_onset_pouring
    
    return method_struct


  def struct_to_array(self,method_list=[]):
    points = []
    for method_struct in method_list:
      points.append([method_struct.pour_time, method_struct.final_vol_err]) #[x,y]
    return np.array(points)

  def label_data(self,label='',x=0,y=0, side='right', text_size=10):
    if side in 'right':
      plt.annotate(
          label,
          xy=(x, y), xytext=(-20, 20),
          textcoords='offset points', ha='right', va='bottom',
          bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
          arrowprops=dict(arrowstyle = '->', connectionstyle='arc3,rad=0'), fontsize=text_size)
    else:
      plt.annotate(
          label,
          xy=(x, y), xytext=(20, 20),
          textcoords='offset points', ha='left', va='bottom',
          bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
          arrowprops=dict(arrowstyle = '->', connectionstyle='arc3,rad=0'), fontsize=text_size)


  def GP_method_comparison_read_in_data_from_directory(self,plot_type=None, seperate_plots=None):
    '''
    This assumes that the directory contains all methods (param/semiparam/nonparam)
    1. For each method, a list of [file_idx, final_vol_err, final_pour_time] is collected with the stipulation that
      A. final_vol_err is the median volume taken over the last second of pouring  (stores the vol/time at which this occurs)
      B. final time is the time between
        i) the onset of pouring: measured by when the volume first exceeded 10ml
        ii) when the median volume during the last second was taken (obtained in step A)
      Output: [e_{v,f}, t_{f}]
    2. two plots, first shows true data (90pts) with labels on top of statistics, second just shows the statistics (mean,var)
    '''
    all_files_in_dir = os.listdir(self.directory_path) #contains all files
    param_method_data = []
    semiparam_method_data = []
    nonparam_method_data = []
    #1. iterate through all files extracting 1.A,B
    for curr_file in all_files_in_dir:
      if '.bag' in curr_file:
        #load data and retrieve data
        curr_bag = rosbag.Bag(self.directory_path+curr_file)
        curr_bag_data_struct = MethodDataStruct()
        curr_bag_data_struct.trial = curr_file[curr_file.find('ml')-7:curr_file.find('ml')-4]
        curr_bag_data_struct = self.load_data_to_struct(method_struct=curr_bag_data_struct, bag=curr_bag)
        curr_bag.close()

        #append this to appropriate list
        if 'semiparam' in curr_file:
          semiparam_method_data.append(curr_bag_data_struct)
        elif 'nonparam' in curr_file:
          nonparam_method_data.append(curr_bag_data_struct)
        else:
          param_method_data.append(curr_bag_data_struct)

    #2. Plot full data
    param_points = self.struct_to_array(method_list=param_method_data)
    semiparam_points  = self.struct_to_array(method_list=semiparam_method_data)
    nonparam_points = self.struct_to_array(method_list=nonparam_method_data)

    if plot_type in 'final_err_time_coupled':
      self.plot_final_err_time_coupled(param_points=param_points, semiparam_points=semiparam_points,nonparam_points=nonparam_points, 
                                       param_method_data=param_method_data, semiparam_method_data=semiparam_method_data, nonparam_method_data=nonparam_method_data)
    if plot_type in 'final_err_time_box_plots':
      self.plot_final_err_time_vs_method_boxplot(param_points=param_points, semiparam_points=semiparam_points,nonparam_points=nonparam_points, seperate_plots=seperate_plots)


  def plot_final_err_time_coupled(self,param_points=None,semiparam_points=None,nonparam_points=None, param_method_data=None, semiparam_method_data=None,nonparam_method_data=None ):
    xp,yp = param_points.T
    xp_mean, yp_mean = np.mean(xp), np.mean(yp)
    xsp,ysp = semiparam_points.T
    xsp_mean, ysp_mean = np.mean(xsp), np.mean(ysp)
    xnp,ynp = nonparam_points.T
    xnp_mean, ynp_mean = np.mean(xnp), np.mean(ynp)
    #setup plot
    fig = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
    ax = fig.gca()
    #plot the means
    plt.plot(xp_mean,yp_mean,'rx',linewidth=20.0)
    self.label_data(label="param",x=xp_mean,y=yp_mean, text_size=self.plot_text_size)
    plt.plot(xsp_mean,ysp_mean,'rx',linewidth=20.0)
    self.label_data(label="semiparam",x=xsp_mean,y=ysp_mean, text_size=self.plot_text_size)
    plt.plot(xnp_mean,ynp_mean,'rx',linewidth=20.0)
    self.label_data(label="nonparam",x=xnp_mean,y=ynp_mean,side='left', text_size=self.plot_text_size)
    #label bags if desired
    if self.show_labeled_bag_trials:
      for idx in range(len(xp)):
        self.label_data(label='p'+param_method_data[idx].trial, x=xp[idx], y=yp[idx])
      for idx in range(len(xsp)):
        self.label_data(label='sp'+semiparam_method_data[idx].trial, x=xsp[idx], y=ysp[idx])
      for idx in range(len(xnp)):
        self.label_data(label='np'+nonparam_method_data[idx].trial, x=xnp[idx], y=ynp[idx])

    #Method plots
    self.cls_obj_plt_cov.plot_point_cov(param_points, nstd=2, alpha=0.5, color='green') #nstd is the number of the stdev
    self.cls_obj_plt_cov.plot_point_cov(semiparam_points, nstd=2, alpha=0.5, color='blue')
    self.cls_obj_plt_cov.plot_point_cov(nonparam_points, nstd=2, alpha=0.5, color='red')

    # plt.xlim([np.min([np.min(xp),np.min(xsp),np.min(xnp)])-3,np.max([np.max(xp),np.max(xsp),np.max(xnp)]) +3])
    # plt.ylim([np.min([np.min(yp),np.min(ysp),np.min(ynp)])-3,np.max([np.max(yp),np.max(ysp),np.max(ynp)])+3 ])
    plt.xlim([0,np.max([np.max(xp),np.max(xsp),np.max(xnp)]) +3])
    plt.ylim([0,np.max([np.max(yp),np.max(ysp),np.max(ynp)])+3 ])

    if self.show_dist_pts:
      plt.plot(xp,yp,'go')
      plt.plot(xsp,ysp,'bo')
      plt.plot(xnp,ynp,'ro')

    ax.set_aspect(1.0)
    ax.set_xlabel('pour time (s)', fontsize=self.plot_text_size)
    ax.set_ylabel('$e_{V}(t_f)$ (ml)', fontsize=self.plot_text_size)
    ax.tick_params(labelsize=self.tick_plot_axis_text_size)


  def plot_final_err_time_vs_method_boxplot(self,param_points=None,semiparam_points=None,nonparam_points=None, seperate_plots=False):
    xp,yp,xsp,ysp,xnp,ynp=[],[],[],[],[],[]
    try:
      xp,yp = param_points.T
      xp_mean, yp_mean = np.mean(xp), np.mean(yp)
    except:
      pass
    try:
      xsp,ysp = semiparam_points.T
      xsp_mean, ysp_mean = np.mean(xsp), np.mean(ysp)
    except:
      pass
    try:
      xnp,ynp = nonparam_points.T
      xnp_mean, ynp_mean = np.mean(xnp), np.mean(ynp)
    except:
      pass
    #plot box plot of error
    labels = ['param','semiparam','nonparam']
    err_data =[yp,ysp,ynp]
    time_data = [xp,xsp,xnp]
    #set properties
    boxprops = dict(linewidth=5)
    flierprops = dict(marker='o', markerfacecolor='green', markersize=3, linestyle='none')
    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')
    meanpointprops = dict(marker='D', markeredgecolor='black', markerfacecolor='blue', markersize=12)
    meanlineprops = dict(linestyle='--', linewidth=2.5, color='purple')
    #box plot the error
    if seperate_plots:
      fig1 = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
      ax1 = fig1.gca()
      fig2 = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
      ax2 = fig2.gca()
    else:
      fig = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
      ax1 = fig.add_subplot(1,2,1)
      ax2 = fig.add_subplot(1,2,2)

    ax1.boxplot(err_data, labels=labels, whis='range',boxprops=boxprops, meanprops=meanpointprops, flierprops=flierprops, medianprops=medianprops, meanline=False, showmeans=True )
    ax1.set_xlabel('Method', fontsize=self.plot_text_size)
    ax1.set_ylabel('$e_{V}(t_f)$ (ml)', fontsize=self.plot_text_size)
    ax1.set_ylim(bottom=0)
    ax1.tick_params(labelsize=self.tick_plot_axis_text_size)
    #box plot the final time
    ax2.boxplot(time_data, labels=labels, whis='range',boxprops=boxprops, meanprops=meanpointprops, flierprops=flierprops, medianprops=medianprops, meanline=False, showmeans=True )
    ax2.set_xlabel('Method', fontsize=self.plot_text_size)
    ax2.set_ylabel('$t_f$ (ml)', fontsize=self.plot_text_size)
    ax2.set_ylim(bottom=0)
    ax2.tick_params(labelsize=self.tick_plot_axis_text_size)



def plot_cov_example():
  points = np.random.multivariate_normal(
            mean=(1,1), cov=[[0.4, 9],[9, 10]], size=1000
            )
  points2 = np.random.multivariate_normal(
          mean=(4,4), cov=[[0.3, 9],[9, 7]], size=1000
          )
    # Plot the raw points...
  x, y = points.T
  x2, y2 = points2.T
  x_mean = np.mean(x)
  y_mean = np.mean(y)
  x2_mean = np.mean(x2)
  y2_mean = np.mean(y2)
  plt.plot(x, y, 'ro')
  plt.plot(x2, y2, 'bo')
  plt.plot(x_mean, y_mean, 'ro')
  plt.plot(x2_mean, y2_mean, 'ro')

  # Plot a transparent 3 standard deviation covariance ellipse
  cls_obj_plt_cov = PlotCovariance()
  cls_obj_plt_cov.plot_point_cov(points, nstd=2, alpha=0.5, color='green')
  cls_obj_plt_cov.plot_point_cov(points2, nstd=2, alpha=0.5, color='blue')
  plt.xlim([np.min([np.min(x),np.min(x2)])-3,np.max([np.max(x),np.max(x2)]) +3])
  plt.ylim([np.min([np.min(y),np.min(y2)])-3,np.max([np.max(y),np.max(y2)])+3 ])
  plt.show()


def main():
  rospy.init_node('method_comp_post_process_node')
  rospack = rospkg.RosPack()
  dir_path = rospack.get_path("pouring_unknown_geom") + '/bagfiles/' + 'cont6/150ml/'
  bag_path = rospy.get_param("~post_process_bags_path", dir_path)
  
  plot_type = rospy.get_param("~plot_type", 'final_err_time_coupled')  #'final_err_time_coupled' or 'final_err_time_box_plots'
  seperate_plots = rospy.get_param("~seperate_plots", False)  #'final_err_time_coupled'

  show_data_pts = rospy.get_param("~show_dist_pts", True)
  show_labeled_bag_trials = rospy.get_param("~show_labeled_bag_trials", True)
  print "bag path: ", bag_path
  plot_axis_text_size = 30
  tick_plot_axis_text_size = 20#14

  cls_obj = NewGPPlotting(dir_path=bag_path, show_dist_pts=show_data_pts, show_labeled_bag_trials=show_labeled_bag_trials, plot_text_size=plot_axis_text_size, tick_plot_axis_text_size=tick_plot_axis_text_size)
  # plot_type = 'final_err_time_coupled'
  # plot_type = 'final_err_time_box_plots'
  cls_obj.GP_method_comparison_read_in_data_from_directory(plot_type=plot_type, seperate_plots=seperate_plots)
  plt.show()

if __name__ == '__main__':
  main()
# https://stackoverflow.com/questions/5147112/how-to-put-individual-tags-for-a-scatter-plot
