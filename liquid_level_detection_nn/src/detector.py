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

        self.apriltag_sub = rospy.Subscriber("/pg_13344889/tags", ApriltagArrayStamped, self.get_apriltag_msg)

        self.image_pub = rospy.Publisher('/hed_level_detector/output', Image, queue_size=1)

        self.bridgeA = CvBridge()
        self.bridgeB = CvBridge()

        self.docker_path = os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker")

        # Initialize sockets for communication with docker image
        self.init_docker_image()

        self.image_sub = rospy.Subscriber("/pg_13344889/image_raw", Image, self.read_image)
        self.serv_docker_reset = rospy.Service('/liquid_level_detection_nn/reset_docker', Empty, self.reset_docker_image)
        self.serv_reset_nn = rospy.Service('/liquid_level_detection_nn/docker_apriltag_reset', Trigger, self.reset_callback)

        rospy.loginfo('hed_detector initialized')
        self.connected = True
        self.output_image_listener()
        # rospy.spin()



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

        self.original_image_shape = image.shape

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

        if not connected_bool:
            rospy.logwarn("Not connected")
            return

        #self.start_time = time.time()

        patch = image[center_y-100:center_y+2*image_height-100, center_x-image_width:center_x+image_width]
        patch =  cv2.resize(patch, (0,0), fx=0.5, fy=0.5)


        # test = np.zeros(self.original_image_shape[0:2])
        # patch_scaled = cv2.resize(patch, (0, 0), fx=2, fy=2)
        # test[self.center_y-100:self.center_y+2*self.image_height-100, self.center_x-self.image_width:self.center_x+self.image_width] = patch_scaled[:, :, 0]
        # print patch.shape
        # print len(patch.tostring())
        # cv2.imshow("test", patch)
        # cv2.waitKey(0)
        # cv2.imwrite('test.png', test)
        # patch =  cv2.resize(image, (0,0), fx=0.25, fy=0.25)
        try:
            self.input_socket.sendall(patch.tostring())
        except socket.error as err:
            rospy.logerr("Send failed : %s", str(err))




    def reset_docker_image(self, req):
        print "Resetting"
        self.thread_lock.acquire()
        self.connected = False
        self.thread_lock.release()

        client = docker.from_env()

        containers = client.containers.list()


        for container in containers:
            if 'pytorch_liquid_level' in container.attrs['Config']['Image']:
                rospy.loginfo("Killing container: %s", container.name)
                container.stop()

        self.output_socket.close()
        self.input_socket.close()

        self.init_docker_image()


        rospy.loginfo("hed_detector reinitialized")
        self.thread_lock.acquire()
        self.connected = True
        self.thread_lock.release()

        return EmptyResponse()


    def init_docker_image(self):
        self.output_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        if os.path.exists(os.path.join(self.docker_path, "output_socket")):
            os.remove(os.path.join(self.docker_path, "output_socket"))
        try:
            self.output_socket.bind(os.path.join(self.docker_path, "output_socket"))
        except socket.error as msg:
            rospy.logerr('Output Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            sys.exit()

        self.output_socket.listen(1)

        rospy.loginfo('Starting docker image')
        command = 'nvidia-docker run -v ' + self.docker_path + ':/volume -u `id -u` pytorch_liquid_level:0.1 &'
        os.system(command)
        rospy.loginfo('Waiting for connection')

        self.conn, addr = self.output_socket.accept()

        rospy.sleep(3)

        self.input_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.input_socket.connect(os.path.join(self.docker_path, "input_socket"))
        except socket.error as msg:
            rospy.logerr('Input Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            sys.exit()

    def output_image_listener(self):
        """ Recieves the neural net's output through the output
        socket, then post processes the image and publishes it
        on the ros network
        """
        while not rospy.is_shutdown():
            # data = conn.recv(256*320)
            data = []
            bool_exist = True

            self.thread_lock.acquire()
            original_image_shape = self.original_image_shape
            connected_bool = copy.copy(self.connected)
            if type(self.apriltag) != type(None):
                center_x = int(self.apriltag.center.x)
                center_y = int(self.apriltag.center.y)
            else:
                bool_exist=False

            image_height = self.image_height
            image_width = self.image_width
            self.thread_lock.release()

            if type(original_image_shape) == type(None):
                continue

            if not bool_exist:
                rospy.logwarn("No bool_exist for tag")
                continue

            if not connected_bool:
                rospy.logwarn("Not connected in output image listener")
                continue

            try:
                data = self.conn.recv(412*390) # TODO: Turn off timeout
            except socket.error as err:
                rospy.logerr("Receive failed: %s", str(err))

            if len(data) == 0:
                rospy.logwarn("No data from socket")
                continue

            if len(data) < 412*390:
            # if len(data) < 256*320:
                self.buff.append(data)
                self.buff_length += len(data)

                if self.buff_length == 412*390:
                # if self.buff_length == 256*320*2*2:
                    data = ''.join(self.buff)
                else:
                    rospy.logwarn("buf length not correct")
                    continue
            image = np.fromstring(data, dtype=np.uint8)
            # image = image.reshape((256*2, 320*2))
            image = image.reshape((412, 390))

            # ret, output = cv2.threshold(image, 160, 255, cv2.THRESH_TOZERO)
            image_scaled = cv2.resize(image, (0, 0), fx=2, fy=2)
            output = np.zeros(original_image_shape[0:2], dtype=np.uint8)
            output[center_y-100:center_y+2*image_height-100, center_x-image_width:center_x+image_width] = image_scaled
            try:
                out_image = self.bridgeB.cv2_to_imgmsg(output, encoding="passthrough")
            except CvBridgeError as e:
                rospy.logerr(e)
                break

            self.image_pub.publish(out_image)

            self.buff = []
            self.buff_length = 0


if __name__ == '__main__':
    Detector()
