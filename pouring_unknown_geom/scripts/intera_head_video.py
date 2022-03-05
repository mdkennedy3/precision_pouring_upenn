#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image


class HeadImageDisplay(object):
  def __init__(self):
    self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=1)

  def easy_pub(self,img):
    self._image_pub.publish(img);


def main():
  rospy.init_node('disp_live_image_on_head')
  cls_obj = HeadImageDisplay()
  rospy.Subscriber('/pouring_control/controller_display',Image,cls_obj.easy_pub, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  main()
