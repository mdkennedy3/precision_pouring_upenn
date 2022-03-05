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
  rospy.init_node("setup_motions", anonymous=True)
  finished_pub = rospy.Publisher('/setup_finished', Empty, queue_size=1)

  #Previous
  #watch_pose = [0.682783203125, -0.790439453125, -0.4495224609375, 1.8742724609375, 0.1734658203125, -0.477193359375, -1.132205078125]
  
  #New (closer to table)
  #watch_pose = [0.69121484375, -0.6140234375, -0.4454619140625, 1.96512890625, 0.18889453125, -0.8354970703125, -1.13096484375]
  watch_pose = [ 0.68473828125, -0.4473955078125, -0.4537275390625, 2.0091630859375, 0.219326171875, -1.222955078125, -1.1295322265625]


  # pickup_pose = [0.5408017578125, 0.2664365234375, -1.13071875, 0.7716376953125, 1.1324267578125, -0.5188955078125, -1.133650390625]
  # midpoint_pose = [0.435609375, -0.202775390625, -0.7443720703125, 0.91694140625, 0.690330078125, -0.452958984375, -1.13303125]
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



  limb.move_to_joint_positions(convert_to_dict(watch_pose), timeout=20.0)

  print "Waiting for container edge"
  rospy.wait_for_service('/pouring_control/publish_container_profile_serv');
  t_req = TriggerRequest();
  cont_edge_srv = rospy.ServiceProxy('/pouring_control/publish_container_profile_serv',Trigger);
  print "Sent service request"
  resp = cont_edge_srv(t_req)
  print resp

  cont_edge_srv_reset = rospy.ServiceProxy('/pouring_control/reset_publish_container_profile_serv',Trigger);
  cont_edge_srv_reset()


  classification_srv = rospy.ServiceProxy('/pouring_control/model_prior_classificaton_serv', Trigger)
  classification_srv(TriggerRequest())

  # gripper.open()
  # rospy.sleep(1)
  # limb.move_to_joint_positions(convert_to_dict(pickup_pose), timeout=20.0)
  # gripper.close()
  # rospy.sleep(1)
  limb.move_to_joint_positions(convert_to_dict(midpoint_pose), timeout=20.0)


  finished_pub.publish(Empty())

