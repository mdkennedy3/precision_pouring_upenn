#!/usr/bin/env python
# coding=UTF-8
# Keeps track of, plots, and stores angle and volume data from a pour


import rospy
import json
from sensor_msgs.msg import JointState
from usb_scale.msg import Scale
from pouring_unknown_geom.msg import IsReady
import matplotlib.pyplot as plt
import sys
import os
import datetime
import platform
from config import data_location

import numpy as np
import matplotlib as mpl

mpl.use('TkAgg')
import multiprocessing as mp
import logging


fig, ax = plt.subplots()
win = fig.canvas.manager.window

logger = mp.log_to_stderr(logging.INFO)

INITIAL_MASS = 772.0
current_angle = 0.0
current_weight = INITIAL_MASS
angles = [0.0]
weights = [0]

def truncate_angle(ang):
    # This function truncates the angle to 0.05 increments to account for the change
    # as the manipulator rotates

    rnd_ang = round(ang, 2)
    return ((rnd_ang * 100) - (rnd_ang * 100) % 5 ) / 100.0

def angle_callback(data):
    # records angle of gripper

    global current_angle
    global weights
    global angles
    current_angle = truncate_angle(data.position[6])
    if (current_weight != weights[-1]  or  current_angle != angles[-1]):
        angles.append(current_angle)
        weights.append(current_weight)
        print "Volume: {} Angle: {}".format(current_weight, round(current_angle, 3))

    

def weight_callback(data):
    # subscribed to scale

    global current_weight
    global weights
    global angles
    current_weight = INITIAL_MASS - data.weight
    if (current_weight != weights[-1]  or  current_angle != angles[-1]):
        weights.append(current_weight)
        angles.append(current_angle)
        print "Volume: {} Angle: {}".format(current_weight, round(current_angle, 3))

    

def root_callback(data):
    # this callback sets up the other two subscribers once the pouring begins
    # it is triggered by a ROS message called IsReady

    global INITIAL_MASS
    global container_id
    global current_weight
    global weights
    if (data.isReady):
        INITIAL_MASS = data.initialVolume
        current_weight = INITIAL_MASS
        container_id = data.containerId
        weights[0] = INITIAL_MASS
        rospy.Subscriber('/iiwa/joint_states', JointState, angle_callback, queue_size=1)
        rospy.Subscriber('/usb_scale/scale', Scale, weight_callback, queue_size=1)
        print "Ready!"
    else:
        print "Not ready"
        rospy.signal_shutdown("End of data collection")


def write_files():
    # Write data to file
    try:
        if not os.path.exists(data_location + 'data/' + container_id):
            os.makedirs(data_location + 'data/' + container_id)

        f = open(data_location + 'data/' + container_id +'/data_' + container_id + '_' + datetime.datetime.now().strftime('%Y-%m-%dT%H-%M-%S') + '.json', 'w')
        json.dump([angles, weights], f)
        print "***************FILES WRITTEN*****************"
    except (Exception):
        print "Files not written"


def main():

    axes = plt.gca()
    axes.set_xlim([0, 3])
    axes.set_ylim([-300, INITIAL_MASS + 20])

    plt.plot(angles, weights, 'b-')
    plt.xlabel(u"\u03B8" + '(rad)')
    plt.ylabel('Volume (mL)')


    rospy.init_node('angle_tracker')

    rospy.Subscriber('/pouring_unknown_geom/status', IsReady, root_callback, queue_size=1)

    win.after(100, animate)
    plt.show()

    rospy.on_shutdown(write_files)
    logger.info("Done")
     

def animate():
     # redraw the canvas
    if (not rospy.is_shutdown()):
        plt.cla()
        plt.plot(angles, weights, 'b-')
        axes = plt.gca()
        plt.xlabel(r"$\theta$ (rad)")
        plt.ylabel('Volume (mL)')
        axes.set_xlim([0, 3])
        axes.set_ylim([-300, INITIAL_MASS + 10])
        fig.canvas.draw() 
        win.after(100, animate)
    else:
        print "Shutdown"

if __name__ == '__main__':
    main()

