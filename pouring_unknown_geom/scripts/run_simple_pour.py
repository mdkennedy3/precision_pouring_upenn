#!/usr/bin/env python


import rospy

from std_srvs.srv import Trigger, TriggerRequest
from pouring_unknown_geom.srv import *  #ContainerEdgeProfileSrv*
from pouring_unknown_geom.msg import PourControlAction, PourControlGoal, PourControlResult
# from pouring_msgs.srv import DetectorResetRequest, DetectorReset
import actionlib
from std_msgs.msg import Empty
import intera_interface


def convert_to_dict(angles):
  joint_angles = dict()

  for i in range(len(angles)):
    joint_angles['right_j'+str(i)] = angles[i]
  print joint_angles
  return joint_angles

def trigger_service(name):
  print "Waiting for service: ", name
  rospy.wait_for_service(name)
  service_proxy = rospy.ServiceProxy(name,Trigger)
  print "Calling service: ", name
  service_proxy(TriggerRequest())
  print "Finished service: ", name, "\n"

if __name__ == '__main__':
  rospy.init_node("run_simple_pour")

  #desired_volume = 100
  desired_volume = float(rospy.get_param("/pouring_control/main_node/ml_des_volume",100))
  theta_start = 20*3.14/180
  print "Desired volume is: ", desired_volume


  midpoint_pose = [0.11596875, -0.5148837890625, -0.35419921875, 1.055234375, 0.0595146484375, -0.538197265625, -0.5323232421875]
  limb = intera_interface.Limb('right')
  #gripper = intera_interface.Gripper('right')

  limb.move_to_joint_positions(convert_to_dict(midpoint_pose), timeout=20.0)
  # Reset arm
  # pub = rospy.Publisher('/reset_arm', Empty)
  # pub.publish(Empty())



  # Reset models
  '''
  trigger_service('/pouring_control/level_detector_node_nn/reset')
  trigger_service('/liquid_level_detection_nn/docker_apriltag_reset')
  trigger_service('/pouring_control/reset_pour_vol_ang')
  trigger_service('/pouring_control/reset_model_selection_node')
  trigger_service('/pouring_control/reset_main_pouring_node')
  '''
  trigger_service('/pouring_control/reset_theta_start')
  
  # req = DetectorResetRequest()
  # req.receiving_beaker_tag_id = 4
  # req.desired_volume_ml = desired_volume
  # rospy.wait_for_service('/pouring_control/level_detector_node_nn/reset')
  # level_service_proxy = rospy.ServiceProxy('/pouring_control/level_detector_node_nn/reset', DetectorReset)
  # level_service_proxy(req)

  # Call pouring action
  client = actionlib.SimpleActionClient('/pouring_control/pour_action', PourControlAction)
  print "Waiting for connection to /pouring_control/pour_action"
  client.wait_for_server()

  goal = PourControlGoal()
  goal.desired_volume_ml = desired_volume
  goal.theta_start = theta_start
  client.send_goal(goal)
  print "Goal sent"
  client.wait_for_result()
  print "Action server returned: ", client.get_result()


