#!/usr/bin/env python2

from __future__ import division
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from pouring_control_pkg.msg import MeasMsg
import numpy as np
# from pouring_unknown_geom.msg import PourModelPred, HybCntrlData
from pouring_msgs.msg import PourModelPred, HybCntrlData


class HeadImageDisplay(object):
  def __init__(self):
    self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=1)
    self.bridge = cv_bridge.CvBridge()
    self.meas = (0,0)
    self.font = cv2.FONT_HERSHEY_SIMPLEX
    self.font_scale = 0.6#1

  def image_subscriber(self,data):
    img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    scale_width = 1024./data.width
    scale_height = 600./data.height
    scale = min(scale_height,scale_width)
    dim = (int(scale*data.width),int(scale*data.height))
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    # resized = cv2.resize(img, dim, interpolation = cv2.INTER_LINEAR)

    outimg = np.zeros((600,1024,3), np.uint8)
    x_start = int((outimg.shape[0] - resized.shape[0])/2);
    y_start = int((outimg.shape[1] - resized.shape[1])/2);
    x_end = x_start + dim[1];
    y_end = y_start + dim[0];
    outimg[x_start:x_end, y_start:y_end,:] += resized


    font = self.font
    font_scale = self.font_scale
    text = "Volume: %.2f ml"%(self.meas[0])
    # text2 = "Volume Rate: %.2f ml"%(self.meas[1])
    cv2.putText(outimg,text,(650,100), font, font_scale,(255,255,255),2,cv2.LINE_AA)
    # cv2.putText(outimg,text2,(650,150), font, 1,(255,255,255),2,cv2.LINE_AA)

    try:
      hyb_text_state = 'Hybrid State: '+self.hyb_cntrl_data.control_state
      hyb_text_traj = 'Desired Traj: %.2f ml'%self.hyb_cntrl_data.traj_height
      cv2.putText(outimg,hyb_text_state,(650,150), font, font_scale,(255,255,255),2,cv2.LINE_AA)
      cv2.putText(outimg,hyb_text_traj,(650,200), font, font_scale,(255,255,255),2,cv2.LINE_AA)
    except:
      #controller possibly isnt running
      pass

    #image_message = self.bridge.cv2_to_imgmsg(outimg, encoding="passthrough")
    image_message = self.bridge.cv2_to_imgmsg(outimg, "bgr8")
    self._image_pub.publish(image_message)


  def meas_callback(self,data):
    self.meas = (data.mh, data.mh_dot)


  def hyb_cntrl_callback(self,data):
    self.hyb_cntrl_data = data

  def easy_pub(self,img):
    self._image_pub.publish(img);


def main():
  rospy.init_node('disp_live_image_on_head')
  cls_obj = HeadImageDisplay()
  #rospy.Subscriber('input_img', Image,cls_obj.image_subscriber, queue_size=1)
  #rospy.Subscriber('mh_mh_dot', MeasMsg,cls_obj.meas_callback, queue_size=1)
  #rospy.Subscriber('Hybrid_controller_data', HybCntrlData, cls_obj.hyb_cntrl_callback, queue_size=1)
  rospy.Subscriber('controller_display',Image,cls_obj.easy_pub, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  main()
