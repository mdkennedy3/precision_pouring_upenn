#!/usr/bin/env python2
import numpy as np, math
import rospy, time
import sys, getopt
from pouring_control_pkg.sawyer_control import sawyer_cntrl_cls 

import intera_interface

from intera_interface import CHECK_VERSION

def main(argv):


    try: 
        opts, args = getopt.getopt(argv, "hm:",["mode="])
    except getopt.GetoptError:
        print "go_to_start_pose.py -m ['full', 'j6']"
        pass

    start_mode = "j6"

    for opt, arg in opts:
        if opt == '-h':
            print "go_to_start_pose.py -m ['full', 'j6']"
            # sys.exit()
        elif opt in ["-m","--mode"]:        
            start_mode = arg





    rospy.init_node('go_to_start_node')

    sawyer_cntrl_obj = sawyer_cntrl_cls.Sawyer_Control()

    # theta_0 = -1.237259765625  #subtract this from all


    #this is increasing, but the theta_0 corresponds to camera down, and rethink cuff pointing up
    # jnt_pos = -0.9984208984375# 0.0 #SET THIS START POSITION
    # joints = 'right_j6'

    head = intera_interface.Head()
    if rospy.has_param('start_j6_angle'):
        j6_start_angle = float(rospy.get_param("start_j6_angle"))
    else:
        j6_start_angle = 2.1085830078125 

    if start_mode in 'full':
        #true zero for j6: 1.57323046875
        #jnt_pos = [0.74856640625, -0.284794921875, -0.6511220703125, -2.8591181640625, -1.5836279296875, -0.171435546875, 0.9511826171875, 2.1085830078125, 0.0]  #this is for beaker and scale only
        # jnt_pos = [0.540396484375, -0.31137109375, -0.719228515625, -2.6975546875, -1.490712890625, -0.098126953125, 0.77388964, 2.1085830078125, 0.0]; #This is for scale, light box and beaker

	#This joint pose is for large light on sawyer
        jnt_pos = [2.0, -0.31137109375, -0.719228515625, -2.6975546875, -1.490712890625, -0.098126953125, 0.77388964, j6_start_angle, 0.0]; #This is for scale, light box and beaker

        #This joint pose is for sawyer with new lightbox setup (with distortion background). and plastic receiver
	jnt_pos = [0.812890625, -0.30025, -0.6244296875, -2.6945498046875, -1.4690751953125, -0.1069716796875, 0.799646484375, 2.1077568359375, 0.0];

        #This is for glass reciever on plastic lightbox setup
	jnt_pos = [0.8135048828125, -0.289740234375, -0.5756904296875, -2.6143310546875, -1.2758115234375, -0.009416015625, 0.658103515625, 2.10775, 0.0];
	

        jnt_pos = [0.8135048828125, -0.289740234375, -0.5756904296875, -2.6143310546875, -1.2758115234375, -0.009416015625, 0.658103515625, -0.7571679687, 0.0];


        joints = ['head_pan', 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6', 'torso_t0']

    else:
        # jnt_pos = 1.57323046875# 0.0 #SET THIS START POSITION
        jnt_pos = j6_start_angle # 0.0 #SET THIS START POSITION
        joints = 'right_j6'



    ts = rospy.Time.now().to_sec()
    tf = rospy.Time.now().to_sec()
    dt = tf - ts

    while not rospy.is_shutdown() and dt < 5.:
        tf = rospy.Time.now().to_sec()
        dt = tf - ts
        sawyer_cntrl_obj.cmd_pos(jnt_pos, joints)

    
    try:
      head.set_pan(2.0)
      rospy.sleep(5)
    except:
      print "unable to move head"
      pass


    #make sure it disappears
    # del sawyer_cntrl_obj




if __name__ == '__main__':
    main(sys.argv[1:])
