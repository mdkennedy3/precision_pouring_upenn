#!/usr/bin/env python


import rospy

import intera_interface
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerRequest
from pouring_unknown_geom.srv import *  #ContainerEdgeProfileSrv*


def convert_to_dict(angles):
  joint_angles = dict()

  for i in range(len(angles)):
    joint_angles['right_j'+str(i)] = angles[i]
  print joint_angles
  return joint_angles

if __name__ == '__main__':
  rospy.init_node("return_to_start_motion_motions", anonymous=True)


  midpoint_pose = [0.11596875, -0.5148837890625, -0.35419921875, 1.055234375, 0.0595146484375, -0.538197265625, -0.5323232421875]
  print "Initing limb"
  limb = intera_interface.Limb('right')
  gripper = intera_interface.Gripper('right')
  print limb.joint_names()
  print "Limb inited"
  _rs = intera_interface.RobotEnable()
  _init_state = _rs.state().enabled
  _rs.enable()
  print("Enabling robot... ")
  angles = limb.joint_angles()

  limb.move_to_joint_positions(convert_to_dict(midpoint_pose), timeout=20.0)



