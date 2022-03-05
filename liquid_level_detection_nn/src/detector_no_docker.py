#!/usr/bin/env python

import rospy
import os
import numpy as np
import cv2
import sys
import time
import socket
import subprocess
import docker

import rospkg

from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError

from apriltag_msgs.msg import ApriltagArrayStamped

import copy
import threading
from std_srvs.srv import Trigger, TriggerResponse


import torch
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader
from model import HED

def grayTrans(img):
    """ Converts from pytorch tensor to numpy image
    """
    img = img.numpy()[0][0]*255.0
    img = (img).astype(np.uint8)
    return img

class Detector(object):
    """ Provides a ros interface to the docker image that contains the neural net.
    """
    def __init__(self):
        rospy.init_node('liquid_level_detector_nn_node')
        rospack = rospkg.RosPack()
        self.thread_lock = threading.Lock()

        self.receive_tag_msg = False

        self.apriltag_id = rospy.get_param("~apriltag_id", 4)
        self.apriltag_size = rospy.get_param("~apriltag_size", 100)

        self.apriltag = None
        self.center_x = None
        self.center_y = None
        self.original_image_shape = None
        self.conn = None

        self.connected = False

        self.buff = []
        self.buff_length = 0
        self.has_apriltag_msg = False

        self.image_height = 412
        self.image_width = 390

        self.apriltag_sub = rospy.Subscriber("/pg_13344889/tags", ApriltagArrayStamped, self.get_apriltag_msg, queue_size=1)

        self.image_pub = rospy.Publisher('/hed_level_detector/output', Image, queue_size=1)

        self.bridgeA = CvBridge()
        self.bridgeB = CvBridge()

        self.net = HED(pretrained=False)
        self.net.cuda()
        self.docker_path = os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker")
        self.net.load_state_dict(torch.load(os.path.join(self.docker_path, 'HED.pth')))


        self.image_sub = rospy.Subscriber("/pg_13344889/image_raw", Image, self.read_image, queue_size=1)
        self.serv_reset_nn = rospy.Service('/liquid_level_detection_nn/docker_apriltag_reset', Trigger, self.reset_callback)

        rospy.loginfo('hed_detector initialized')
        self.connected = True
        rospy.spin()



    def reset_callback(self, req):
        rospy.logwarn("reset NN: Reset requested")
        self.thread_lock.acquire()
        self.receive_tag_msg = False
        self.apriltag = None
        self.center_x = None
        self.center_y = None
        self.has_apriltag_msg = False

        self.thread_lock.release()

        return TriggerResponse(True,"Reset complete")




    def get_apriltag_msg(self, msg):

        self.thread_lock.acquire()
        receive_tag_msg = copy.deepcopy(self.receive_tag_msg)
        self.thread_lock.release()
        if not receive_tag_msg:
            if len(msg.apriltags) > 0:
                for idx in range(len(msg.apriltags)):
                    april = copy.deepcopy(msg.apriltags[idx])
                    #rospy.loginfo("%d", april.id)
                    if april.id == int(self.apriltag_id):

                        #rospy.logwarn("Found id %d %d", april.id, self.apriltag_id)
                        self.thread_lock.acquire()
                        self.apriltag = copy.deepcopy(april)

                        self.center_x = copy.copy(int(self.apriltag.center.x))
                        self.center_y = copy.copy(int(self.apriltag.center.y))
                        self.has_apriltag_msg = True

                        #self.thread_lock.acquire()
                        self.receive_tag_msg = True
                        self.thread_lock.release()
                        #rospy.logwarn("recieved tag once")
                        #rospy.logwarn("cent x y %d %d", self.center_x, self.center_y)
                        return
        else:
            return


    def read_image(self, data):
        """ Gets the image over ros and sends it to the docker image
        through the input socket
        """

        try:
            image = self.bridgeA.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return


        bool_exist = True
        self.thread_lock.acquire()

        original_image_shape = image.shape

        connected_bool = copy.copy(self.connected)
        has_apriltag_msg = copy.copy(self.has_apriltag_msg)

        if type(self.apriltag) != type(None):
            center_x = int(self.apriltag.center.x)
            center_y = int(self.apriltag.center.y)
        else:
            bool_exist=False

        image_height = self.image_height
        image_width = self.image_width

        self.thread_lock.release()

        if not bool_exist:
            #rospy.logwarn("No bool_exist")
            return

        if not has_apriltag_msg:
            rospy.logwarn("No apriltag has been recieved")
            return

        # Makes sure patch is in bounds
        center_y = max(100, center_y)
        center_y = min(image.shape[0] - 2 *image_height + 100, center_y)
        center_x = max(image_width, center_x)
        center_x = min(image.shape[1] - image_width, center_x)

        patch = image[center_y-100:center_y+2*image_height-100, center_x-image_width:center_x+image_width]

        patch =  cv2.resize(patch, (0,0), fx=0.5, fy=0.5)

        image = patch.astype(np.float32)
        image = image/255.0

        image -= np.array((0.485, 0.456, 0.406))
        image /= np.array((0.229, 0.224, 0.225))
        image = image.transpose((2,0,1))

        image = image.reshape((1, 3, image_height, image_width))
        image = torch.from_numpy(image)

        image = Variable(image.cuda())
        s1,s2,s3,s4,s5,s6 = self.net.forward(image)
        output = grayTrans(s6.data.cpu())

        image = np.fromstring(output, dtype=np.uint8)
        # image = image.reshape((256*2, 320*2))
        image = image.reshape((image_height, image_width))

        # ret, output = cv2.threshold(image, 160, 255, cv2.THRESH_TOZERO)
        image_scaled = cv2.resize(image, (0, 0), fx=2, fy=2)
        output = np.zeros(original_image_shape[0:2], dtype=np.uint8)
        output[center_y-100:center_y+2*image_height-100, center_x-image_width:center_x+image_width] = image_scaled
        try:
            out_image = self.bridgeB.cv2_to_imgmsg(output, encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image_pub.publish(out_image)







if __name__ == '__main__':
    Detector()
