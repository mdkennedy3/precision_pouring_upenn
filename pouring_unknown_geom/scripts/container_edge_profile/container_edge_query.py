#!/usr/bin/env python2
'''
This function queries the profile then plots after the message has been published 
'''
import rospy
import numpy as np
from std_srvs.srv import * #for Trigger
from pouring_unknown_geom.srv import *  #ContainerEdgeProfileSrv*
import matplotlib
import matplotlib.pyplot as plt
from pouring_unknown_geom.msg import ContainerEdgeProfile
import threading
from sensor_msgs.msg import PointCloud2

class PublishContainerEdgeProfile(object):
  """docstring for PublishContainerEdgeProfile"""
  def __init__(self):

    # self.cont_edge_srv(pt); #request the service

    self.msg_obtained = False
    self.train_msg_obtained = False
    self.thread_lock = threading.Lock() #.acquire() .release()

    self.callback_msg = None

    self.training_data = None
    # pouring_unknown_geom::ShowFinalEdgeProfileSrv
    self.show_final_profile_srv = rospy.ServiceProxy('show_final_profile',ShowFinalEdgeProfileSrv);


  def ProfileCallback(self,data):
    if type(self.callback_msg) == type(None):
      #callback hasn't been received yet
      self.thread_lock.acquire()
      self.callback_msg = data
      self.msg_obtained = True
      self.thread_lock.release()

  def TrainingProfileCallback(self,data):
    self.thread_lock.acquire()
    self.training_data = data
    self.train_msg_obtained = True
    self.thread_lock.release()


  def PlotContainerCurvature(self):
    self.thread_lock.acquire()
    callback_msg = self.callback_msg
    training_data = self.training_data
    self.thread_lock.release()


    #service call the function to generate the final view of these points
    req = ShowFinalEdgeProfileSrvRequest()
    req.profile.height = callback_msg.height
    req.profile.radius = callback_msg.radius
    req.profile.container_height_m = callback_msg.container_height_m
    resp = self.show_final_profile_srv(req)

    
    r_train = training_data.radius
    h_train = training_data.height

    r = callback_msg.radius
    h = callback_msg.height

    fig, ax = plt.subplots()

    ax.scatter(r_train,h_train,label="training_pts",linewidth=12)
    ax.scatter(r,h,label="test_points",linewidth=4)
    
    plt.ylim((0,0.15))
    plt.xlim((0,0.1)) #.10
    ax.set_aspect(1.0)
    # plt.axis('equal')
    plt.xlabel('Radius(m)', fontsize=27)
    plt.ylabel('Height(m)',fontsize=27)
    ax.legend(fontsize=22)
    matplotlib.rcParams.update({'font.size': 22})
    plt.xticks(fontsize=24)
    plt.yticks(fontsize=24)
    plt.show()



def main():
  rospy.init_node("obtain_container_profile_edge")
  cls_obj = PublishContainerEdgeProfile()
  rospy.wait_for_service('publish_container_profile_serv');
  t_req = TriggerRequest();
  cont_edge_srv = rospy.ServiceProxy('publish_container_profile_serv',Trigger);
  resp = cont_edge_srv(t_req)

  while not rospy.is_shutdown() and not cls_obj.msg_obtained:
    rospy.Subscriber("container_edge_profile_pub_msg", ContainerEdgeProfile, cls_obj.ProfileCallback)

  while not rospy.is_shutdown() and not cls_obj.train_msg_obtained:
    rospy.Subscriber("container_training_edge_profile_pub_msg", ContainerEdgeProfile, cls_obj.TrainingProfileCallback)


  cls_obj.PlotContainerCurvature()

  #reset
  cont_edge_srv_reset = rospy.ServiceProxy('reset_publish_container_profile_serv',Trigger);
  cont_edge_srv_reset()



if __name__ == '__main__':
  main()
