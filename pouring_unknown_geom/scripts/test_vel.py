#!/usr/bin/env python


import rospy
import intera_interface

rospy.init_node('testvel')
j6_start_angle = 2.1077
start_jnt_positions = [-0.31137109375, -0.719228515625, -2.6975546875, -1.490712890625, -0.098126953125, 0.77388964, j6_start_angle]; #This is for scale, light box and beaker
jnt_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
#Move to desired pose
start_pose_dict = dict(zip(jnt_names,start_jnt_positions))


rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
rs.enable()
sawyer_limb = intera_interface.Limb("right")
starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}

#sawyer_limb.move_to_joint_positions(starting_joint_angles)   
sawyer_limb.move_to_joint_positions(start_pose_dict)   


