#!/usr/bin/env python2
import numpy as np, math
import rospy, rospkg
from pouring_unknown_geom.srv import *  #ContainerEdgeProfileSrv*
from pouring_unknown_geom.msg import ScanBoxMsg
from sensor_msgs.msg import PointCloud2

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use('GTKAgg')
import threading
import scipy as sp
from scipy import stats

from std_srvs.srv import * #for Trigger
from pouring_unknown_geom.msg import ContainerEdgeProfile


class ContainerEdgeClient(object):
  def __init__(self, num_pts_return=128, container_slice_resolution=0.01, camera_frame= "pico_camera_link",scan_box=None, plot_container_points=True, num_profiles_to_collect=10):
    rospy.wait_for_service('~container_edge_profile');
    self.cont_edge_srv = rospy.ServiceProxy('~container_edge_profile',ContainerEdgeProfileSrv);
    self.thread_lock = threading.Lock() #.acquire() .release()
    #extract the profile service to be triggered by other code
    self.retrieve_container_profile_bool = False  #should the resp be recovered now?
    #set service request params for profile extractor
    self.num_pts_return = num_pts_return;
    self.container_slice_resolution = container_slice_resolution;
    self.camera_frame = camera_frame;
    self.scan_box= scan_box;
    self.plot_container_points = plot_container_points
    self.container_edge_profile = [] #list of profilesself.retrieve_container_profile_bool = False  #should the resp be recovered now?
    self.container_edge_training_points = []
    self.profile_number_to_collect = num_profiles_to_collect #number of profile edges to collect
    if self.profile_number_to_collect >= num_pts_return: rospy.loginfo("Change the number of profile number to collect to be less than num_pts_return")

    self.container_edge_profile_publisher = rospy.Publisher("container_edge_profile_pub_msg",ContainerEdgeProfile, queue_size=1, latch=True)
    self.container_training_edge_profile_publisher = rospy.Publisher("container_training_edge_profile_pub_msg",ContainerEdgeProfile, queue_size=1, latch=True)
    

  def ResetTrig(self,req):
    #function resets for next operation
    self.retrieve_container_profile_bool = False  #should the resp be recovered now?
    self.container_edge_profile = [] #list of profilesself.retrieve_container_profile_bool = False  #should the resp be recovered now?
    self.container_edge_training_points = []
    if self.profile_number_to_collect >= self.num_pts_return: rospy.loginfo("Change the number of profile number to collect to be less than num_pts_return")
    trig_resp = TriggerResponse()
    trig_resp.success = True
    trig_resp.message = "reset"
    return trig_resp

  def PubContainerProfile(self,req):
    #1. Collect the latest responses and add to a running list
    self.retrieve_container_profile_bool = True
    container_edge_profile_length = 0
    while not rospy.is_shutdown() and container_edge_profile_length <= self.profile_number_to_collect:
      self.thread_lock.acquire()
      container_edge_profile_length = len(self.container_edge_profile)
      self.thread_lock.release()
    self.retrieve_container_profile_bool = False #collection complete (stop querying)
    self.thread_lock.acquire()
    container_edge_profile = self.container_edge_profile
    container_edge_training_points = self.container_edge_training_points
    self.thread_lock.release()
    #2. Given the M profiles, for radial dimension (ri) over all N samples, find the index of the median (this creats an Nx1 list)
    container_edge_profile_matrix = []
    for idx in range(len(container_edge_profile)):
      container_edge_profile_matrix.append(container_edge_profile[idx].radius)
    container_edge_profile_matrix = np.matrix(container_edge_profile_matrix) #radius are aligned on columns
    #3. Over the Nx1 list, if N>M then there must be repetition, find the mode index (most occuring)
    median_val_vect = np.median(container_edge_profile_matrix,axis=0) #median values
    median_idx_vect = [np.where(container_edge_profile_matrix[:,j]==median_val_vect.item(j))[0].item(0) for j in range(median_val_vect.shape[1])] #idx of median values
    mode_output_idx = sp.stats.mode(median_idx_vect).mode.item(0)
    #4. The reoccuring mode index of the profile is then selected as the highest probability profile for the container, publish this message with a latch
    median_edge_profile = container_edge_profile[mode_output_idx] #output the mode value here, type pouring_unknown_geom/ContainerEdgeProfile
    median_training_edge_profile = container_edge_training_points[mode_output_idx]
    self.container_edge_profile_publisher.publish(median_edge_profile)
    self.container_training_edge_profile_publisher.publish(median_training_edge_profile)

    rospy.loginfo("publishing profile")
    # print "publishing profile"#, median_edge_profile
    trig_resp = TriggerResponse()
    trig_resp.success = True
    trig_resp.message = "obtained profile and published"
    return trig_resp


  def PtCldCallback(self,data):
    self.thread_lock.acquire()
    retrieve_container_profile_bool = self.retrieve_container_profile_bool
    self.thread_lock.release()
    if retrieve_container_profile_bool:
      req = ContainerEdgeProfileSrvRequest();
      req.pt_cld = data;
      req.camera_frame = self.camera_frame;
      req.container_slice_resolution = self.container_slice_resolution;#meters (start with 0.05), (better resolution: 0.005), latest was 0.002
      req.num_pts_return = self.num_pts_return;
      req.container_scan_box.header.frame_id = self.camera_frame;
      req.container_scan_box.p1.x= self.scan_box.p1.x;
      req.container_scan_box.p1.y= self.scan_box.p1.y;
      req.container_scan_box.p1.z= self.scan_box.p1.z;
      req.container_scan_box.p2.x= self.scan_box.p2.x;
      req.container_scan_box.p2.y= self.scan_box.p2.y;
      req.container_scan_box.p2.z= self.scan_box.p2.z;
      req.plot_container_points = self.plot_container_points;
      #Call service
      try:
        resp = self.cont_edge_srv(req);
        if resp.success:
          # print "\n\nfirst ensure it returns something sensible"
          # print resp
          self.thread_lock.acquire()
          self.container_edge_profile.append(resp.profile)
          self.container_edge_training_points.append(resp.generating_cyl_points)
          print "gen points", resp
          self.thread_lock.release()
      except:
        pass



def main():
  rospy.init_node('container_profile_client');
  scan_box = ScanBoxMsg()
  #When sensor is mounted on sawyer
  scan_box.p1.x = 0.22; scan_box.p1.y = -0.1; scan_box.p1.z = -0.05;
  scan_box.p2.x = 0.50; scan_box.p2.y = 0.1; scan_box.p2.z = 0.3;

  #on testing bag file
  # scan_box.p1.x = 0.0; scan_box.p1.y = -0.1; scan_box.p1.z = -0.1;
  # scan_box.p2.x = 0.35; scan_box.p2.y = 0.1; scan_box.p2.z = 0.3;


  num_profile_edge_points= rospy.get_param("num_profile_edge_points",128);

  #cls_obj = ContainerEdgeClient(num_pts_return=num_profile_edge_points, container_slice_resolution=0.01, camera_frame= "pico_camera_link", scan_box=scan_box);
  # cls_obj = ContainerEdgeClient(num_pts_return=num_profile_edge_points, container_slice_resolution=0.016, camera_frame= "pico_camera_link", scan_box=scan_box, num_profiles_to_collect=22);
  cls_obj = ContainerEdgeClient(num_pts_return=num_profile_edge_points, 
                                container_slice_resolution=0.015, #0.025 // 0.015 
                                camera_frame= "pico_camera_link", 
                                scan_box=scan_box, 
                                num_profiles_to_collect=22);

  sub = rospy.Subscriber("/monstar/points",PointCloud2, cls_obj.PtCldCallback);
  serv_prof = rospy.Service('~publish_container_profile_serv',Trigger, cls_obj.PubContainerProfile);
  serv_reset = rospy.Service('~reset_publish_container_profile_serv',Trigger, cls_obj.ResetTrig);
  rospy.spin()


if __name__=='__main__':
    main()



'''
 import matplotlib.pyplot as plt
  r = 
  h = 
  plt.scatter(r,h)
  plt.axis('equal')
  plt.show()
'''
