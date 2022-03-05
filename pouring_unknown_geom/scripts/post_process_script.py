#!/usr/bin/env python2
import rospy
from pouring_unknown_geom.Post_process_data import post_process

def main():
  rospy.init_node('post_process_node')
  filename = rospy.get_param("~bagfile_name")
  msg_prefix = rospy.get_param("~msg_namespace",'pouring_control')  #rospy.get_param("bagfile_name")
  msg_prefix = '/'+msg_prefix# '/pouring_control'
  msgs = [msg_prefix+'/Pour_Model_opt', msg_prefix+'/pouring_msg', msg_prefix + '/gp_selected_models_edge_profile', msg_prefix+'/container_training_edge_profile_pub_msg']
  num_iter_plots = 10 #number of curves to show in additional to final curve for system identification
  print "got here"
  cls_obj = post_process.ReadBags(filename,msgs, num_iter_plots, msg_prefix=msg_prefix)
  cls_obj.load_data()
  #now plot sys identification
  show_fig_in_script_bool = True
  cls_obj.plot_model_identification(show_fig_in_script_bool)


if __name__ == '__main__':
  main()
