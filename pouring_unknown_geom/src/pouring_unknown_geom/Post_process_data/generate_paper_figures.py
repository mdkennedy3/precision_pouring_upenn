#!/usr/bin/env python2
import rospy, rospkg, rosbag, sys, getopt, os, glob, numpy as np
from pouring_unknown_geom.Post_process_data import post_process
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import re 
import yaml

class Figure_Plots(object):
  def __init__(self):
    self.bag_file_location = '/home/manip_control/Desktop/bagfile_new_pouring/'
    self.container_repeatability =self.bag_file_location+ 'container_repeatability/'
    self.container_type_range_mult_vol = self.bag_file_location+'container_type_range_mult_vol/'
    self.opt_method_time_accuracy_over_steady_pour = self.bag_file_location+'opt_method_time_accuracy_over_steady_pour/'
    self.plot_directory = "/home/manip_control/Desktop/bagfile_new_pouring/paper_plots/"
    self.legend_text_size = 20
    self.plot_axis_text_size = 28

  def Demonstrate_system_identification(self, start_time=21.0):
    filename = '/home/manip_control/Desktop/bagfile_new_pouring/container_repeatability/cont26/cont26_100ml_repeat_trial_015_amazing_new.bag'
    msgs = ['/Pour_Model_opt','/pouring_msg']
    num_iter_plots = 5
    #Get the Volume model identification and residual graphs
    cls_obj = post_process.ReadBags('cont26_100ml_repeat_trial_015_amazing_new.bag',msgs, num_iter_plots, dir_path=filename)
    cls_obj.directory_path = '/home/manip_control/Desktop/bagfile_new_pouring/container_repeatability/cont26/'
    cls_obj.figure_directory_path = self.plot_directory
    cls_obj.load_data()
    show_fig_in_script_bool = False
    cls_obj.plot_model_identification(show_fig_in_script_bool)
    #Get the h_meas vs h_pred versus time
    bag = rosbag.Bag(filename) 
    t_first = None
    msgs = ['/mh_mh_dot','/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command','/scale_node/scale']
    data = { msgs[idx]:{'msg':[],'t':[]} for idx in range(len(msgs))}
    for topic, msg, t in bag.read_messages(topics=msgs):
      if not t_first: t_first = t.to_sec()
      if (t.to_sec()-t_first) > start_time:
        data[topic]['msg'].append(msg)
        data[topic]['t'].append(t.to_sec()-t_first)
    bag.close()

    self.h_data = {'h_meas':{'val':[],'t':[]},'h_traj':{'val':[],'t':[]},'h_model':{'val':[],'t':[]},'h_meas_pred':{'val':[],'t':[]}, 'h_traj_pred':{'val':[],'t':[]}}
    self.robot_data = {'cmd_vel':{'val':[],'t':[]},'vel_act':{'val':[],'t':[]}, 'dv_dth_alpha_pred':{'val':[],'t':[]}}
    self.scale_data = {'weight':{'val':[],'t':[]}}



    t_first = None
    for msg in data['/mh_mh_dot']['msg']:
      if msg.mh > 0.5:
        if not t_first: t_first = msg.header.stamp.to_sec()
        self.h_data['h_meas']['val'].append(msg.mh)
        self.h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)


    for msg in data['/robot/joint_states']['msg']:
      self.robot_data['vel_act']['val'].append(msg.velocity[7])
      self.robot_data['vel_act']['t'].append(msg.header.stamp.to_sec()-t_first)#data['/robot/joint_states']['msg'][0].header.stamp.to_sec())
    for msg in data['/robot/limb/right/joint_command']['msg']:
      self.robot_data['cmd_vel']['val'].append(msg.velocity)
      self.robot_data['cmd_vel']['t'].append(msg.header.stamp.to_sec()-t_first)#data['/robot/limb/right/joint_command']['msg'][0].header.stamp.to_sec())



    for msg in data['/model_volume_output']['msg']:
      self.h_data['h_model']['val'].append(msg.volume)
      self.h_data['h_model']['t'].append(msg.header.stamp.to_sec() -t_first)

    for msg in data['/Hybrid_controller_data']['msg']:
      self.h_data['h_traj']['val'].append(msg.traj_height)
      self.h_data['h_traj']['t'].append(msg.header.stamp.to_sec() -t_first)
      self.h_data['h_meas_pred']['val'].append(msg.predicted_height.pred_height)
      self.h_data['h_meas_pred']['t'].append(msg.header.stamp.to_sec() -t_first)
      self.h_data['h_traj_pred']['val'].append(msg.predicted_height.pred_traj_height)
      self.h_data['h_traj_pred']['t'].append(msg.header.stamp.to_sec() -t_first)
      self.robot_data['dv_dth_alpha_pred']['val'].append(msg.predicted_height.dv_dth_alpha)
      self.robot_data['dv_dth_alpha_pred']['t'].append(msg.header.stamp.to_sec() -t_first)

    for msg in data['/scale_node/scale']['msg']:
        self.scale_data['weight']['val'].append(msg.weight)
        self.scale_data['weight']['t'].append(msg.header.stamp.to_sec() -t_first)

    new_scale_data = {'weight':[],'t':[],'h_meas':[],'error_w-h':[]}
    for idx in range(len(self.scale_data['weight']['t'])):
      t = self.scale_data['weight']['t'][idx]
      if t > 0.0:
        #now we have caught up with the stamp on the camera data
        tdiff = [np.abs(t_meas-t) for t_meas in self.h_data['h_meas']['t']]
        closest_idx = np.argmin(tdiff)
        new_scale_data['weight'].append(self.scale_data['weight']['val'][idx])
        new_scale_data['t'].append(self.scale_data['weight']['val'][idx])
        new_scale_data['h_meas'].append(self.h_data['h_meas']['val'][closest_idx])
        new_scale_data['error_w-h'].append(self.scale_data['weight']['val'][idx] -self.h_data['h_meas']['val'][closest_idx] )
        
    



    #Now make plots: 
    fig1 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax1 = fig1.gca()
    ax1.plot(self.h_data['h_meas']['t'],self.h_data['h_meas']['val'],'g',label='$h_{meas}$', linewidth=5.0)
    ax1.plot(self.h_data['h_model']['t'], self.h_data['h_model']['val'],'r',label='$h_{model}$', linewidth=5.0)
    ax1.plot(self.h_data['h_traj']['t'], self.h_data['h_traj']['val'],'b',label='$h_{traj}$', linewidth=5.0)
    plt.grid()
    ax1.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    # ax1.set_xlim(0,th[-1])#np.pi)
    ax1.set_ylim(0,120)
    ax1.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax1.tick_params(labelsize=self.plot_axis_text_size)
    plt.legend(loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_1 =self.plot_directory+ "cont26_100mlrep_15_height_meas_traj_model"
    plt.savefig(fig_name_1,bbox_inches='tight')

    fig2 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax2 = fig2.gca()
    ax2.plot(self.h_data['h_meas_pred']['t'],self.h_data['h_meas_pred']['val'],'g',label='$h_{meas-predict}$', linewidth=5.0)
    ax2.plot(self.h_data['h_traj_pred']['t'],self.h_data['h_traj_pred']['val'],'b',label='$h_{traj-predict}$', linewidth=5.0)
    plt.grid()
    ax2.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax2.set_ylim(0,120)
    ax2.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax2.tick_params(labelsize=self.plot_axis_text_size)
    plt.legend(loc=2, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_2=self.plot_directory+"cont26_100mlrep_15_pred_height_meas_traj"
    plt.savefig(fig_name_2,bbox_inches='tight')

    fig3 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax3 = fig3.gca()
    ax3.plot(self.robot_data['vel_act']['t'],self.robot_data['vel_act']['val'],'g',label='$\\omega_{act}$', linewidth=5.0)
    ax3.plot(self.robot_data['cmd_vel']['t'],self.robot_data['cmd_vel']['val'],'b',label='$\\omega_{cmd}$', linewidth=5.0)
    plt.grid()
    ax3.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax3.set_ylabel('$\\omega$ (rad/sec)', fontsize=self.plot_axis_text_size)
    ax3.tick_params(labelsize=self.plot_axis_text_size)
    plt.legend(loc=1, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_3=self.plot_directory+"cont26_100mlrep_15_commanded_vs_actual_joint_vel"
    plt.savefig(fig_name_3,bbox_inches='tight')

    fig4 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax4 = fig4.gca()
    ax4.plot(self.robot_data['dv_dth_alpha_pred']['t'],self.robot_data['dv_dth_alpha_pred']['val'],'r',label='$dV/d\\theta$', linewidth=5.0)
    plt.grid()
    ax4.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax4.set_ylabel('$dV/d\\theta$ (ml/rad)', fontsize=self.plot_axis_text_size)
    ax4.tick_params(labelsize=self.plot_axis_text_size)
    plt.legend(loc=1, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_4=self.plot_directory+"cont26_100mlrep_15_dv_dth_alpha_est"
    plt.savefig(fig_name_4,bbox_inches='tight')
    # plt.show()



    fig5 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax5 = fig5.gca()
    ax5.plot(new_scale_data['t'],new_scale_data['error_w-h'],'r',label='$dV/d\\theta$', linewidth=5.0)
    ax5.grid()
    ax5.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax5.set_ylabel('$(V_{scale}-V_{cam})(ml)$ (ml/rad)', fontsize=self.plot_axis_text_size)
    ax5.tick_params(labelsize=self.plot_axis_text_size)
    plt.legend(loc=1, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_5=self.plot_directory+"cont26_100mlrep_15_Vol_error"
    plt.savefig(fig_name_5,bbox_inches='tight')
    # plt.show()




  def Variety_plots(self):
    directory = self.container_type_range_mult_vol
    file_list = []
    for root, dirs, files in os.walk(directory):
      for file in files:
          if file.endswith(".bag"):
            file_list.append(os.path.join(root,file))
    fig1 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax1 = fig1.gca()
    fig2 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax2 = fig2.gca()
    fig3 = plt.figure(figsize=(23,8.5), dpi=80, facecolor='w',edgecolor='k')
    ax3 = fig3.gca()


    table_data = []

    for bag_name in file_list:
      #load in the bag
      bag = rosbag.Bag(bag_name) 
      t_first = None
      msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
      self.h_data = {'h_meas':{'val':[],'t':[]}}
      for topic, msg, t in bag.read_messages(topics=msgs):
        if msg.mh > 0.5:
          if not t_first: t_first = msg.header.stamp.to_sec()
          self.h_data['h_meas']['val'].append(msg.mh)
          self.h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
      bag.close()

      #generate Label string for container: 
      back_slash_occur = [x.start() for x in re.finditer('/',bag_name)]
      cont_number = bag_name[back_slash_occur[-1]+5:back_slash_occur[-1]+7]
      label_str = '$h_{meas, ' + cont_number + '}$' 

      stop_time = self.h_data['h_meas']['t'][-1]
      for idx in range(len(self.h_data['h_meas']['val'])-1):
        jdx = -(idx+2)
        if np.abs(self.h_data['h_meas']['val'][jdx] - self.h_data['h_meas']['val'][-1]) >=2.0:
          stop_time = self.h_data['h_meas']['t'][jdx]
          break
        else:
          stop_time = self.h_data['h_meas']['t'][jdx]

      table_input = {'cont':cont_number,'h_final':self.h_data['h_meas']['val'][-1],'Time_to_complete':stop_time}
      table_data.append(table_input)

      #depending if its the 100, 150 or 50ml add it to the respective graph
      if bag_name[bag_name.find('ml')-3:bag_name.find('ml')] in '_50':
        ax1.plot(self.h_data['h_meas']['t'],self.h_data['h_meas']['val'],label=label_str, linewidth=5.0)
      if bag_name[bag_name.find('ml')-3:bag_name.find('ml')] in '100':
        ax2.plot(self.h_data['h_meas']['t'],self.h_data['h_meas']['val'],label=label_str, linewidth=5.0)
      if bag_name[bag_name.find('ml')-3:bag_name.find('ml')] in '150':
        ax3.plot(self.h_data['h_meas']['t'],self.h_data['h_meas']['val'],label=label_str, linewidth=5.0)

    # ax1.grid()
    major_ticks = np.arange(0, 80, 10)
    ax1.set_yticks(major_ticks)
    ax1.grid()
    ax1.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax1.set_ylim(0,70)
    ax1.set_xlim(0,35)
    ax1.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax1.tick_params(labelsize=self.plot_axis_text_size)
    ax1.legend(loc=4,  borderaxespad=0., prop={'size': self.legend_text_size})
    fig_name_1 =self.plot_directory+ "Variety_plots_50ml"
    fig1.savefig(fig_name_1,bbox_inches='tight')


    major_ticks = np.arange(0, 130, 10)
    ax2.set_yticks(major_ticks)
    ax2.grid()
    ax2.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax2.set_ylim(0,120)
    ax2.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax2.legend(loc=4, borderaxespad=0.,prop={'size': self.legend_text_size})
    ax2.tick_params(labelsize=self.plot_axis_text_size)
    fig_name_2 =self.plot_directory+ "Variety_plots_100ml"
    fig2.savefig(fig_name_2,bbox_inches='tight')


    major_ticks = np.arange(0, 170, 10)
    ax3.set_yticks(major_ticks)
    ax3.grid()
    ax3.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax3.set_ylim(0,160)
    ax3.set_xlim(0,60)
    ax3.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax3.legend(loc=4, borderaxespad=0.,prop={'size': self.legend_text_size})
    # ax3.tick_params(labelsize=self.plot_axis_text_size)
    fig_name_3 =self.plot_directory+ "Variety_plots_150ml"
    fig3.savefig(fig_name_3,bbox_inches='tight')

    #Save data for table output
    with open(self.plot_directory+'variety_data.yml', 'w') as outfile:
      yaml.dump(table_data, outfile, default_flow_style=False)

    # plt.show()



  def plot_repeat(self,fig=None,ax=None,cont_data=None, data_str=None, figname=None):
    #Now all the bag data are in the respective lists with similar start times, so iterate for each and find the mean and standard deviation
    #1. Cont15: find the one with the maximum time, then for every element of that, find the closest time in other lists (within the average time step) (store the mean and stdev and time directly)
    times = [cont['h_meas']['t'][-1]-cont['h_meas']['t'][0] for cont in cont_data]
    data_length = [len(cont['h_meas']['t']) for cont in cont_data]
    max_time_idx = np.argmax(data_length)
    cont_major = cont_data[max_time_idx]
    h_mean = []
    h_std_dev = []
    t_plot = []
    dt = [cont_major['h_meas']['t'][idx+1]-cont_major['h_meas']['t'][idx] for idx in range(len(cont_major['h_meas']['t'])-1)]
    dt_avg = np.mean(dt)
    for idx in range(len(cont_major['h_meas']['t'])):
      t_curr = cont_major['h_meas']['t'][idx]
      if data_str in 'cont15' and t_curr > 40.0: break
      if data_str in 'cont26' and t_curr > 38.0: break
      h_curr = cont_major['h_meas']['val'][idx]
      h_list_curr = []
      h_list_curr.append(h_curr)
      for mdx in range(len(cont_data)):
        if mdx != max_time_idx:
          cont_test = cont_data[mdx]
          if idx < len(cont_test['h_meas']['val'])-1:
            h_list_curr.append(cont_test['h_meas']['val'][idx])
      if len(h_list_curr) > 1:
        h_mean.append(np.mean(h_list_curr))
        # print "hlist: ", h_list_curr
        h_std_dev.append(np.std(h_list_curr))
        t_plot.append(t_curr)
    ax.plot(t_plot,h_mean,label='$h_{avg}$', linewidth=5.0)
    
    '''
    for idx in range(len(cont_data)):
      t_list = cont_data[idx]['h_meas']['t']
      h_list = cont_data[idx]['h_meas']['val']
      legend_text = '$'+ cont_data[idx]['data_trial'][:5]+ cont_data[idx]['data_trial'][7:]+'$'
      print "adding another to the plot list"
      ax.plot(t_list,h_list,label=legend_text)
    '''


    #Show variance:
    up_deviation = np.array(h_mean) + np.array(h_std_dev); up_deviation = up_deviation.tolist()
    dwn_deviation = np.array(h_mean) - np.array(h_std_dev); dwn_deviation = dwn_deviation.tolist()
    ax.fill_between(t_plot, dwn_deviation, up_deviation, alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
    major_ticks = np.arange(0, 130, 10)
    ax.set_yticks(major_ticks)
    ax.grid()
    ax.set_xlabel('$t$ (sec)', fontsize=self.plot_axis_text_size)
    ax.tick_params(labelsize=self.plot_axis_text_size)
    # ax.tick_params(labelsize=self.plot_axis_text_size)
    # ax1.set_ylim(0,120)
    ax.set_ylabel('$V$ (ml)', fontsize=self.plot_axis_text_size)
    ax.legend(loc=4, borderaxespad=0.,prop={'size': self.legend_text_size})
    fig_name_1 =self.plot_directory+ figname
    print "fig name:", fig_name_1
    fig.savefig(fig_name_1,bbox_inches='tight')
    # plt.show()



  def repeatability_plots(self):
    directory = self.container_repeatability
    file_list = []
    for root, dirs, files in os.walk(directory):
      for file in files:
          if file.endswith(".bag"):
            file_list.append(os.path.join(root,file))

    fig1 = plt.figure(figsize=(15,6), dpi=80, facecolor='w',edgecolor='k')  #was 23,8.5
    ax1 = fig1.gca()
    fig2 = plt.figure(figsize=(15,6), dpi=80, facecolor='w',edgecolor='k')
    ax2 = fig2.gca()
    fig3 = plt.figure(figsize=(15,6), dpi=80, facecolor='w',edgecolor='k')
    ax3 = fig3.gca()

    cont15_data = []
    cont26_data = []
    cont30_data = []

    for bag_name in file_list:
      #load in the bag
      back_slash_occur = [x.start() for x in re.finditer('/',bag_name)]
      trial_number = [x.start() for x in re.finditer('trial',bag_name)]
      cont_number = bag_name[back_slash_occur[-1]+5:back_slash_occur[-1]+7]
      # if cont_number in '15' and len(cont15_data) < 5:
      print "reading bag: ", bag_name
      bag = rosbag.Bag(bag_name) 
      t_first = None
      msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
      h_data = {'h_meas':{'val':[],'t':[]},'data_trial':bag_name[trial_number[0]:trial_number[0]+9]}
      for topic, msg, t in bag.read_messages(topics=msgs):
        if cont_number in '30':
          ml_start = 7
        else:
          ml_start = 0.5
        if msg.mh > ml_start:
          if not t_first: t_first = msg.header.stamp.to_sec()
          h_data['h_meas']['val'].append(msg.mh)
          h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
      bag.close()

      #generate Label string for container: 
      back_slash_occur = [x.start() for x in re.finditer('/',bag_name)]
      cont_number = bag_name[back_slash_occur[-1]+5:back_slash_occur[-1]+7]
      if cont_number in '15':
        cont15_data.append(h_data)
      if cont_number in '26':
        cont26_data.append(h_data)
      if cont_number in '30':
        cont30_data.append(h_data)



    self.plot_repeat(fig=fig1,ax=ax1,cont_data=cont15_data, data_str='cont15', figname='cont15_100ml_repeatability')
    self.plot_repeat(fig=fig2,ax=ax2,cont_data=cont26_data, data_str='cont26', figname='cont26_100ml_repeatability')
    self.plot_repeat(fig=fig3,ax=ax3,cont_data=cont30_data, data_str='cont30', figname='cont30_100ml_repeatability')



  def benchmarking_plots(self):
    directory = self.opt_method_time_accuracy_over_steady_pour
    file_list = []
    for root, dirs, files in os.walk(directory):
      for file in files:
          if file.endswith(".bag"):
            file_list.append(os.path.join(root,file))

    bench_data = []
    for bag_name in file_list:
      print "reading ", bag_name
      back_slash_occur = [x.start() for x in re.finditer('/',bag_name)]
      cont_number = bag_name[back_slash_occur[-1]+5:back_slash_occur[-1]+7]
      filename = bag_name[back_slash_occur[-1]+1:]
      if 'openloop' in filename and 'w=0.01' in filename:
        bag = rosbag.Bag(bag_name) 
        t_first = None
        msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
        h_data = {'h_meas':{'val':[],'t':[]}}
        for topic, msg, t in bag.read_messages(topics=msgs):
          if msg.mh > 0.5:
            if not t_first: t_first = msg.header.stamp.to_sec()
            h_data['h_meas']['val'].append(msg.mh)
            h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
        bag.close()
        stop_time = h_data['h_meas']['t'][-1]
        for idx in range(len(h_data['h_meas']['val'])-1):
          jdx = -(idx+2)
          if np.abs(h_data['h_meas']['val'][jdx] - h_data['h_meas']['val'][-1]) >=2.0:
            stop_time = h_data['h_meas']['t'][jdx]
            break
          else:
            stop_time = h_data['h_meas']['t'][jdx]
        data_entry = {'dataname':cont_number+'_openloop_w0.01','h_final':h_data['h_meas']['val'][-1], "t_final":stop_time}
        bench_data.append(data_entry)
      if 'openloop' in filename and 'w=0.02' in filename:
        bag = rosbag.Bag(bag_name) 
        t_first = None
        msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
        h_data = {'h_meas':{'val':[],'t':[]}}
        for topic, msg, t in bag.read_messages(topics=msgs):
          if msg.mh > 0.5:
            if not t_first: t_first = msg.header.stamp.to_sec()
            h_data['h_meas']['val'].append(msg.mh)
            h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
        bag.close()
        stop_time = h_data['h_meas']['t'][-1]
        for idx in range(len(h_data['h_meas']['val'])-1):
          jdx = -(idx+2)
          if np.abs(h_data['h_meas']['val'][jdx] - h_data['h_meas']['val'][-1]) >=2.0:
            stop_time = h_data['h_meas']['t'][jdx]
            break
          else:
            stop_time = h_data['h_meas']['t'][jdx]
        data_entry = {'dataname':cont_number+'_openloop_w0.02','h_final':h_data['h_meas']['val'][-1], "t_final":stop_time}
        bench_data.append(data_entry)
      if 'simple_gain' in filename:
        bag = rosbag.Bag(bag_name) 
        t_first = None
        msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
        h_data = {'h_meas':{'val':[],'t':[]}}
        for topic, msg, t in bag.read_messages(topics=msgs):
          if msg.mh > 0.5:
            if not t_first: t_first = msg.header.stamp.to_sec()
            h_data['h_meas']['val'].append(msg.mh)
            h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
        bag.close()
        stop_time = h_data['h_meas']['t'][-1]
        for idx in range(len(h_data['h_meas']['val'])-1):
          jdx = -(idx+2)
          if np.abs(h_data['h_meas']['val'][jdx] - h_data['h_meas']['val'][-1]) >=2.0:
            stop_time = h_data['h_meas']['t'][jdx]
            break
          else:
            stop_time = h_data['h_meas']['t'][jdx]
        data_entry = {'dataname':cont_number+'simple_gain_Kp_2.0','h_final':h_data['h_meas']['val'][-1], "t_final":stop_time}
        bench_data.append(data_entry)
      else:
        #our method
        bag = rosbag.Bag(bag_name) 
        t_first = None
        msgs = ['/mh_mh_dot']#,'/model_volume_output','/Hybrid_controller_data', '/robot/joint_states', '/robot/limb/right/joint_command']
        h_data = {'h_meas':{'val':[],'t':[]}}
        for topic, msg, t in bag.read_messages(topics=msgs):
          if msg.mh > 0.5:
            if not t_first: t_first = msg.header.stamp.to_sec()
            h_data['h_meas']['val'].append(msg.mh)
            h_data['h_meas']['t'].append(msg.header.stamp.to_sec() -t_first)
        bag.close()
        stop_time = h_data['h_meas']['t'][-1]
        for idx in range(len(h_data['h_meas']['val'])-1):
          jdx = -(idx+2)
          if np.abs(h_data['h_meas']['val'][jdx] - h_data['h_meas']['val'][-1]) >=2.0:
            stop_time = h_data['h_meas']['t'][jdx]
            break
          else:
            stop_time = h_data['h_meas']['t'][jdx]
        data_entry = {'dataname':cont_number+'our_method','h_final':h_data['h_meas']['val'][-1], "t_final":stop_time}
        bench_data.append(data_entry)
    with open(self.plot_directory+'benchmark_data.yml', 'w') as outfile:
      yaml.dump(bench_data, outfile, default_flow_style=False)


  def benchmarking_plots_analyzed(self):
    bench_data = None
    with open(self.plot_directory+'benchmark_data.yml') as stream:
      try:
          bench_data = yaml.load(stream)
      except yaml.YAMLError as exc:
          print(exc)


    #1. 
    cont24_data = {"w=0.01":[],"w=0.02":[],"Kp=2.0":[],"method":[],"method_avg":[]}
    cont26_data = {"w=0.01":[],"w=0.02":[],"Kp=2.0":[],"method":[],"method_avg":[]}
    cont28_data = {"w=0.01":[],"w=0.02":[],"Kp=2.0":[],"method":[],"method_avg":[]}
    cont16_data = {"w=0.01":[],"w=0.02":[],"Kp=2.0":[],"method":[],"method_avg":[]}
    for idx in range(len(bench_data)):
      entry = bench_data[idx]
      name = entry['dataname']
      if '24' in name:
        if "w0.01" in name:
          cont24_data['w=0.01'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "w0.02" in name:
          cont24_data['w=0.02'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "Kp" in name:
          cont24_data['Kp=2.0'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "our_method" in name:
          cont24_data['method'].append([np.abs(100-entry['h_final']),entry['t_final']])
      if '26' in name:
        if "w0.01" in name:
          cont26_data['w=0.01'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "w0.02" in name:
          cont26_data['w=0.02'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "Kp" in name:
          cont26_data['Kp=2.0'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "our_method" in name:
          cont26_data['method'].append([np.abs(100-entry['h_final']),entry['t_final']])
      if '28' in name:
        if "w0.01" in name:
          cont28_data['w=0.01'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "w0.02" in name:
          cont28_data['w=0.02'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "Kp" in name:
          cont28_data['Kp=2.0'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "our_method" in name:
          cont28_data['method'].append([np.abs(100-entry['h_final']),entry['t_final']])
      if '16' in name:
        if "w0.01" in name:
          cont16_data['w=0.01'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "w0.02" in name:
          cont16_data['w=0.02'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "Kp" in name:
          cont16_data['Kp=2.0'].append([np.abs(100-entry['h_final']),entry['t_final']])
        if "our_method" in name:
          cont16_data['method'].append([np.abs(100-entry['h_final']),entry['t_final']])

    #Now find the average of our method
    cont24_data['method_avg'] = np.mean(cont24_data['method'],axis=0)
    cont26_data['method_avg'] = np.mean(cont26_data['method'],axis=0)
    cont28_data['method_avg'] = np.mean(cont28_data['method'],axis=0)
    cont16_data['method_avg'] = np.mean(cont16_data['method'],axis=0)
    #Now plot
    fig = plt.figure(figsize=(16,8.5), dpi=80, facecolor='w',edgecolor='k')
    # fig = plt.figure(dpi=80, facecolor='w',edgecolor='k')
    ax = fig.gca()

    keys_to_plot = ["w=0.01","w=0.02","Kp=2.0","method_avg"]

    font_dict = {'color':'black',
                 'size':self.plot_axis_text_size}
    # ax1.text(date[10], closep[1],'Text Example', fontdict=font_dict)

    marker_size = 300

    color_key = dict(zip(keys_to_plot,['b','g','k','r']))
    xy_data = []
    x_shift = -5
    for key in keys_to_plot:
      if key in 'method_avg':
        y,x =cont24_data[key][0],cont24_data[key][1]
        labelname = '$'+'cont24-'+"method"+ '$'
        ax.scatter(x,y,s=marker_size,label=labelname,c=color_key[key], marker='x')
      else:
        y,x =cont24_data[key][0][0],cont24_data[key][0][1]
        labelname = '$'+'cont24- '+key + '$'
        ax.scatter(x,y,s=marker_size,label=labelname, marker='x',c=color_key[key])
      if key != 'Kp=2.0':
        xy_data.append([y,(x-x_shift)**2,(x-x_shift),1])
      # ax.annotate(labelname, xy=(x, y))
      # ax.text(x,y+0.5, labelname, fontdict=font_dict)

    for key in keys_to_plot:
      if key in 'method_avg':
        y,x =cont26_data[key][0],cont26_data[key][1]
        labelname = '$'+'cont26-'+"method"+ '$'
        ax.scatter(x,y,s=marker_size,label=labelname,c=color_key[key],marker='*')
      else:
        y,x =cont26_data[key][0][0],cont26_data[key][0][1]
        labelname = '$'+'cont26- '+key + '$'
        ax.scatter(x,y,s=marker_size,label=labelname,marker='*',c=color_key[key])
      if key != 'Kp=2.0':
        xy_data.append([y,(x-x_shift)**2,(x-x_shift),1])


    for key in keys_to_plot:
      if key in 'method_avg':
        y,x =cont28_data[key][0],cont28_data[key][1]
        labelname = '$'+'cont28-'+"method"+ '$'
        ax.scatter(x,y,s=marker_size,label=labelname,c=color_key[key],marker='s')
      else:
        y,x =cont28_data[key][0][0],cont28_data[key][0][1]
        labelname = '$'+'cont28- '+key + '$'
        ax.scatter(x,y,s=marker_size,label=labelname,marker='s',c=color_key[key])
      if key != 'Kp=2.0':
        xy_data.append([y,(x-x_shift)**2,(x-x_shift),1])


    for key in keys_to_plot:
      if key in 'method_avg':
        y,x =cont16_data[key][0],cont16_data[key][1]
        labelname = '$'+'cont16-'+"method"+ '$'
        ax.scatter(x,y,s=marker_size,label=labelname,c=color_key[key],marker='o')
      else:
        y,x =cont16_data[key][0][0],cont16_data[key][0][1]
        labelname = '$'+'cont16- '+key + '$'
        ax.scatter(x,y,s=marker_size,label=labelname,marker='o',c=color_key[key])
      if key != 'Kp=2.0':
        xy_data.append([y,(x-x_shift)**2,(x-x_shift),1])

    print "xydata",xy_data
    xy_data = np.matrix(xy_data)
    weights = np.linalg.pinv(xy_data[:,1:])*xy_data[:,0]
    print "weights", weights

    x = np.linspace(15,50,100)
    mat = []
    for idx in range(len(x)):
      x_curr = x[idx]
      mat.append([x_curr**2,x_curr,1])

    y = mat*weights 
    y = y.T.tolist()[0] 
    y = [e - 3 for e in y]
    print "y",y
    ax.plot(x,y,'--',c='k')

    ax.tick_params(labelsize=self.plot_axis_text_size)
    ax.legend(loc='top left', bbox_to_anchor=(1.01, 1), borderaxespad=0.,prop={'size': self.legend_text_size})
    ax.set_xlabel('$T_f$ (sec)', fontsize=self.plot_axis_text_size)
    ax.set_ylabel('$V_{error}$ (ml)', fontsize=self.plot_axis_text_size)
    plt.savefig(self.plot_directory+"benchmarking_figure_test.png",bbox_inches='tight')
    plt.show()





def main():
  rospy.init_node('Paper_fig_generation')
  cls_obj = Figure_Plots()

  '''
  Model pour for system identification graphs
  A: Show a model versus measured height with time next to the optimized 
  '''
  cls_obj.Demonstrate_system_identification()

  '''
  1. Obtain pour variety plots: 
  3 volumes (A:50,B:100,C:150) each volume on seperate graph with each 
  graph containing pours from all the contianers (shifted times so they roughly start at the same place) (this start time can be set manually)
  '''
  # cls_obj.Variety_plots()

  '''
  2. Repeatability  (use time shifting to align the onset of pours)
  A:Plot mean and variance for the 3 containers (try on one plot with faint and colored variance shade) on one graph 
  B:On second graph plot Vol-ground_truth (add 2ml to ground truth for scale)
  '''
  # cls_obj.repeatability_plots()

  ''' 
  3. Benchmarking
  A: For all pours obtain their final settled height and settled time, then make a plot of time vs accuracy (where accuracy is abs difference from value to target 100ml, and time is from the onset of pouring**)
  '''
  # cls_obj.benchmarking_plots()

  # cls_obj.benchmarking_plots_analyzed()


if __name__ == '__main__':
  main()
