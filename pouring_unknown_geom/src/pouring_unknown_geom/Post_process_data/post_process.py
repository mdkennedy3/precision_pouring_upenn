#!/usr/bin/env python2

from __future__ import division

import rospy
import numpy as np
import matplotlib.pyplot as plt
import copy
import cPickle as pickle
import rospkg
import json
import os
import sys
from pouring_unknown_geom.container_classification import generate_classification
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D


import rosbag
from pouring_unknown_geom.msg import PourModelPred, PouringMsg
from pouring_control_pkg.msg import MeasMsg

from pouring_unknown_geom.hybrid_control_pour import hybrid_cntrl_pouring_gp_compatible
from pouring_unknown_geom.gaussian_process import gp_process_calc
from pouring_msgs.msg import PourModelPred, GPVolumeModel, ContainerEdgeProfile, GPEdgeProfileSelectedTrain


#from usb_scale.msg import Scale

'''
V = np.polyval(model.coef_opt,theta)
dV = np.polyval(np.polyder(model.coef_opt),theta)
'''
class filestruct():
  def __init__(self):
    self.dir=None; self.subdir=None; self.files=None



class ReadBags(object):
  """docstring for ReadBags"""
  def __init__(self, filename,msgs, num_iter_plots, dir_path=None, fig_size_tuple=(13, 8), msg_prefix=""):
    rospack = rospkg.RosPack()
    self.filename=filename
    if dir_path:
      self.directory_path = dir_path
    else:
      #self.directory_path = rospack.get_path("pouring_unknown_geom") + '/bagfiles/' + filename  #data path
      directory_path = rospack.get_path("pouring_unknown_geom") + '/bagfiles/'
      all_files = []
      for dirname, subdirlist, file_list in os.walk(directory_path): fs = filestruct(); fs.dir=dirname; fs.subdir=subdirlist; fs.files=file_list; all_files.append(fs)
      for f in all_files:
        if filename in f.files:
          self.directory_path = f.dir+'/'+filename


    self.figure_directory_path = rospack.get_path("pouring_unknown_geom") + '/bagfiles/analysis_figs/'  #path to store figures
    self.msgs = msgs
    #load bag:
    self.bag = rosbag.Bag(self.directory_path)

    #number of curves to show in addition to the final curve for system identification
    self.num_iter_plots = num_iter_plots
    print num_iter_plots, self.num_iter_plots

    self.legend_text_size = 20
    self.plot_axis_text_size = 28

    self.fig_size_tuple = fig_size_tuple

    self.msg_prefix = msg_prefix# '/pouring_control'

    #For GP operations:
    self.hyb_control_cls = hybrid_cntrl_pouring_gp_compatible.SawyerHybridControlPour() #for functions

  def load_data(self):
    self.bag_data = { self.msgs[idx]:{'msg':[],'t':[]} for idx in range(len(self.msgs))}
    for topic, msg, t in self.bag.read_messages(topics=self.msgs):
      self.bag_data[topic]['msg'].append(msg)
      self.bag_data[topic]['t'].append(t.to_sec()) #store time seconds
    self.bag.close()
    #time shift options
    # t0 = np.array([cls_obj.bag_data['/pouring_msg']['t'][0] for idx in range(len( cls_obj.bag_data['/pouring_msg']['t'] ))])
    # t =  np.array(cls_obj.bag_data['/pouring_msg']['t'])
    # tshift=t-t0


  def GP_model_evaluator(self,coef=None,theta_list=None,pour_model_obj=None):
    theta_output = []
    pour_model_obj.coef_opt = coef
    self.hyb_control_cls.pour_model_obj = pour_model_obj
    for theta_curr in theta_list:
      theta_output.append(self.hyb_control_cls.volume_model_evaluation(theta_bar=theta_curr))
    return theta_output

  def plot_model_identification(self, show_fig_in_script_bool):
    #all the data is in the object self.bag_data
    #Extract data
    th = []
    Volume = []
    time_vect = []
    for idx in range(len(self.bag_data[self.msg_prefix+'/pouring_msg']['msg'])):
      msg_curr = self.bag_data[self.msg_prefix+'/pouring_msg']['msg'][idx]
      time_curr = self.bag_data[self.msg_prefix+'/pouring_msg']['t'][idx] - self.bag_data[self.msg_prefix+'/pouring_msg']['t'][0]
      if len(th) > 0:
        '''This occurs if back rotation is allowed '''
        if msg_curr.angle > th[-1]:
          th.append(msg_curr.angle)
          # Volume.append(msg_curr.dvolume)
          Volume.append(msg_curr.volume)
          time_vect.append(time_curr)
      else:
        th.append(msg_curr.angle)
        # Volume.append(msg_curr.dvolume)
        Volume.append(msg_curr.volume)
        time_vect.append(time_curr)


    coef_list = []
    t_coef = []
    residual_list = []
    residual_list_stdev = []

    for idx in range(len(self.bag_data[self.msg_prefix+'/Pour_Model_opt']['msg'])):
      msg_curr = self.bag_data[self.msg_prefix+'/Pour_Model_opt']['msg'][idx]
      time_curr = self.bag_data[self.msg_prefix+'/Pour_Model_opt']['t'][idx] - self.bag_data[self.msg_prefix+'/Pour_Model_opt']['t'][0]
      coef_curr = msg_curr.coef_opt
      #msg_curr.volume_solution_mode = "nonparam" #test this before semiparam
      volume_solution_mode = msg_curr.volume_solution_mode
      data_domain = msg_curr.domain
      coef_type = msg_curr.type
      resid_avg_curr = msg_curr.residual_avg
      resid_std_curr = msg_curr.residual_std

      coef_list.append(coef_curr)
      t_coef.append(time_curr)
      residual_list.append(resid_avg_curr)
      residual_list_stdev.append(resid_std_curr)
    pour_model_curr = msg_curr #save this for further use

    '''Collect Edge profiles '''
    # container_edge_profile_model = ContainerEdgeProfile()
    for idx in range(len(self.bag_data[self.msg_prefix+'/container_training_edge_profile_pub_msg']['msg'])):
      msg_curr = self.bag_data[self.msg_prefix+'/container_training_edge_profile_pub_msg']['msg'][idx]
      container_edge_profile_model = msg_curr

    # training_container_edge_profiles_models = GPEdgeProfileSelectedTrain()
    for idx in range(len(self.bag_data[self.msg_prefix+'/gp_selected_models_edge_profile']['msg'])):
      msg_curr = self.bag_data[self.msg_prefix+'/gp_selected_models_edge_profile']['msg'][idx]
      training_container_edge_profiles_models = msg_curr



    '''Plotting '''
    #1. Plot (th,Volume)
    #Plot f(th,model) (or graph of model), (for each or subset of curves) (on same graph as th,dV) using only 5 evenly spaced  (time label each curve!)
    fig1 = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
    ax1 = fig1.gca()
    # ax1.scatter(th,Volume,label='Volume',linewidth=10.0)
    ax1.scatter(th,Volume,label='V',linewidth=10.0)
    ax1.grid()

    # # example
    # x = np.linspace(1,31,31); e = len(x)/5.0; f = [idx*e for idx in range(5)]; xh = [x[int(f[idx])] for idx in range(len(f))]; xh.append(x[-1])
    print "raw number of coefficients from trial: ", len(coef_list), self.num_iter_plots
    coef_sys_ident_indexes = [int(idx*(len(coef_list)/self.num_iter_plots)) for idx in range(self.num_iter_plots)]
    print "lengths to see", len(coef_list), " and ", len(coef_sys_ident_indexes)
    coef_plots_to_show = [coef_list[int(coef_sys_ident_indexes[idx])] for idx in range(len(coef_sys_ident_indexes))]
    coef_plots_to_show.append(coef_list[-1])
    coef_times_to_show = [t_coef[int(coef_sys_ident_indexes[idx])] for idx in range(len(coef_sys_ident_indexes))]
    coef_times_to_show.append(t_coef[-1])

    #Train GP models if desired
    print "check volume solution mode", volume_solution_mode
    if volume_solution_mode in ["semiparam","nonparam"]:
      #train GP's:
      print "getting ready to train GP's"#.\nCurrent pour msg", pour_model_curr
      self.hyb_control_cls.train_GP_models(pour_model_obj=pour_model_curr) #this only needs to be done once per container
      #self.hyb_control_cls.train_GP_models(pour_model_obj=pour_model_curr, N_gp_pts = 10) #change this to see effect with different N for GP
      print "trained GP's"

    #PLOT THE SYSTEM IDENTIFICATION CURVES
    print "len of coef plots", len(coef_sys_ident_indexes)
    for idx in range(len(coef_plots_to_show)-1):
      #find the points from the current coefficients
      curr_coef = coef_plots_to_show[idx]
      if coef_type in "legendre":
        coef_points = np.polynomial.legendre.Legendre(curr_coef, domain=data_domain, window=data_domain).linspace(len(th), domain =data_domain )
      if coef_type in "power":
        theta_list = np.linspace(data_domain[0],data_domain[1],len(th))
        volume_evaluated = self.GP_model_evaluator(coef=curr_coef,theta_list=theta_list,pour_model_obj=pour_model_curr)
        # coef_points = (theta_list,np.polyval(curr_coef,theta_list)) #for same form as above legendre operation
        coef_points = (theta_list,volume_evaluated) #for same form as above legendre operation

      ax1.plot(coef_points[0],coef_points[1],'-.',label=('$a_{t'+str(round(coef_times_to_show[idx],2))+"}$" ), linewidth=4.0)  #label these with time
    #Now plot final curve
    curr_coef = coef_plots_to_show[-1]
    if coef_type in "legendre":
      coef_points = np.polynomial.legendre.Legendre(curr_coef, domain=data_domain, window=data_domain).linspace(len(th), domain =data_domain )
    if coef_type in "power":
      theta_list = np.linspace(data_domain[0],data_domain[1],len(th))
      volume_evaluated = self.GP_model_evaluator(coef=curr_coef,theta_list=theta_list,pour_model_obj=pour_model_curr)
      # coef_points = (theta_list,np.polyval(curr_coef,theta_list)) #for same form as above legendre operation
      coef_points = (theta_list,volume_evaluated) #for same form as above legendre operation
    ax1.plot(coef_points[0],coef_points[1],'r--',label=('$a_{t'+str(round(coef_times_to_show[-1],2))+"}$" ), linewidth=10.0)
    ax1.grid()
    plt.legend(loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
    ax1.set_xlabel('$\\theta$ (rad)', fontsize=self.plot_axis_text_size)
    # ax1.set_xlim(0,th[-1])#np.pi)
    ax1.set_xlim(0,2.3)#1.5,2.4,np.pi)
    ax1.set_ylim(0,np.max(Volume)+30)
    ax1.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax1.tick_params(labelsize=self.plot_axis_text_size)
    #save figure:
    fig_name= self.figure_directory_path + self.filename[:-4]+'_system_identification.png'
    plt.grid()
    plt.savefig(fig_name)
    #2. Plot (time,residual)
    fig2 = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
    ax2 = fig2.gca()
    ax2.plot(t_coef,residual_list,label='residual', linewidth=5.0)
    # plt.grid()
    #Show Variance:
    up_deviation = np.array(residual_list) + np.array(residual_list_stdev); up_deviation = up_deviation.tolist()
    dwn_deviation = np.array(residual_list) - np.array(residual_list_stdev); dwn_deviation = dwn_deviation.tolist()
    ax2.fill_between(t_coef, dwn_deviation, up_deviation, alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')


    plt.legend(loc=1, borderaxespad=0.,prop={'size': self.legend_text_size})
    # ax2.set_xlabel('$\\theta$ (rad)', fontsize=18)
    ax2.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax2.set_xlim(0,t_coef[-1])
    ax2.set_ylim(0,50)
    ax2.set_ylabel('$\sum_{i=1}^{N} \\frac{|y_i - f_i|}{N} $ (ml)', fontsize=self.plot_axis_text_size)
    ax2.tick_params(labelsize=self.plot_axis_text_size)
    ax2.grid()


    #Save figure
    fig_name= self.figure_directory_path + self.filename[:-4]+'_residual_vs_time.png'
    plt.savefig(fig_name)


    #plot the edge profiles
    container_edge_profile_model
    training_container_edge_profiles_models

    rmax = 0.0
    hmax = 0.0
    new_figsize_tuple = (20,10)
    # fig3 = plt.figure(figsize=self.fig_size_tuple, dpi=80, facecolor='w', edgecolor='k')
    fig3 = plt.figure(figsize=new_figsize_tuple, dpi=80, facecolor='w', edgecolor='k')
    ax3= fig3.gca()
    ax3.plot(container_edge_profile_model.radius, container_edge_profile_model.height,label="current container", linewidth=20.0,c='k')
    r_rev = [-r for r in container_edge_profile_model.radius]
    ax3.plot(r_rev, container_edge_profile_model.height, linewidth=20.0,c='k')
    ax3.scatter(0.0,container_edge_profile_model.container_height_m,label="max height", linewidth=20.0,c='k')

    hmax = np.max([hmax, np.max(container_edge_profile_model.height)])
    hmax = np.max([hmax, container_edge_profile_model.container_height_m])
    rmax = np.max([rmax, np.max(container_edge_profile_model.radius)])


    for idx in range(len(training_container_edge_profiles_models.training_containers)):
      curr_train_container = training_container_edge_profiles_models.training_containers[idx]
      ax3.plot(curr_train_container.radius,curr_train_container.height,label=("train cont "+str(idx)), linewidth=10.0, c=('C'+str(idx)) )
      r_rev = [-r for r in curr_train_container.radius]
      ax3.plot(r_rev,curr_train_container.height, linewidth=10.0, c=('C'+str(idx)) )
      #set new maxes if required
      hmax = np.max([hmax, np.max(curr_train_container.height)])
      hmax = np.max([hmax, curr_train_container.container_height_m])
      rmax = np.max([rmax, np.max(curr_train_container.radius)])


    plt.legend(loc=1, borderaxespad=0.,prop={'size': self.legend_text_size})
    ax3.set_xlabel('radius (m)', fontsize=self.plot_axis_text_size)
    hmax+=0.01
    rmax+=0.01
    ax3.set_xlim(-rmax,rmax) #query max rad/height
    ax3.set_ylim(0,hmax)
    ax3.set_ylabel('height (m)', fontsize=self.plot_axis_text_size)
    ax3.tick_params(labelsize=self.plot_axis_text_size)
    ax3.grid()
    #Save figure
    fig_name= self.figure_directory_path + self.filename[:-4]+'_GP_container_edge_profiles.png'
    plt.savefig(fig_name)



    #Plot the GP container volume-angle profiles (along with current container profile)
    fig4 = plt.figure(figsize=new_figsize_tuple, dpi=80, facecolor='w', edgecolor='k')
    ax4= fig4.gca()
    ax4.scatter(th,Volume,label='current container',linewidth=10.0)
    Vgp = volume_evaluated; thgp = theta_list
    ax4.scatter(thgp,Vgp,label='GP approx',linewidth=10.0)
    print "final angles: ",thgp, "\nfinal volumes",Vgp 
    Vmax = 0.0
    thmax = 0.0
    Vmax = np.max([Vmax,np.max(Volume)])
    thmax = np.max([thmax,np.max(th)])
    perc_list = []
    print "thmin, thmax", np.min(th) ,thmax
    for idx in range(len(pour_model_curr.gp_volume_models)):
      th_list_curr = pour_model_curr.gp_volume_models[idx].th
      vol_list_curr = pour_model_curr.gp_volume_models[idx].V
      ax4.scatter(th_list_curr,vol_list_curr,label=("train cont "+str(idx)),c=('C'+str(idx)))
      Vmax = np.max([Vmax,np.max(vol_list_curr)])
      thmax = np.max([thmax,np.max(th_list_curr)])
      perc_list.append(pour_model_curr.gp_volume_models[idx].perc_mix_model)
    #print "perc list: ", perc_list  #this was checked
    ax4.set_xlim(0,thmax) #query max rad/height
    ax4.set_ylim(0,Vmax)
    ax4.set_xlabel('th (rad)', fontsize=self.plot_axis_text_size)
    ax4.set_ylabel('volume (ml)', fontsize=self.plot_axis_text_size)
    ax4.tick_params(labelsize=self.plot_axis_text_size)
    ax4.grid()

    fig_name= self.figure_directory_path + self.filename[:-4]+'_GP_container_volume_profiles.png'
    plt.savefig(fig_name)

    if show_fig_in_script_bool:
      plt.show()  #can also save figures here






# def main():
#   filename = rospy.get_param("bagfile_name")
#   # msgs = ['/Pour_Model_opt','/pouring_msg','/h_filtered','/scale_node/scale','/mh_mh_dot']
#   msgs = ['/Pour_Model_opt','/pouring_msg']
#   num_iter_plots = 20 #number of curves to show in additional to final curve for system identification
#   cls_obj = ReadBags(filename,msgs, num_iter_plots)
#   cls_obj.load_data()

#   #now plot sys identification
#   show_fig_in_script_bool = True
#   cls_obj.plot_model_identification(show_fig_in_script_bool)




# if __name__ == '__main__':
#   main()
