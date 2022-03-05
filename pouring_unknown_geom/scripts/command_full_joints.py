#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

#for sawyer simulation
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import (
    UInt16,
)

from pouring_unknown_geom.msg import PourJoints
from pouring_unknown_geom.srv import *

from intera_core_msgs.msg import JointCommand

class RobotJointCommanderCls(object):
  """docstring for RobotJointCommanderCls"""
  def __init__(self):
    self.robot_setup_bool = False
    self.robot_name = None

    self.sawyer_control_pub = rospy.Publisher('/robot/limb/right/joint_command',JointCommand, queue_size=1,latch=True)

  def server_and_spin(self):
    # self.sub = rospy.Subscriber('/robot_pour_command',PourJoints,self.pour_cmd_callback)
    self.srv = rospy.Service('/robot_pour_commander',PourPlatformControl, self.pour_cmd_callback)
    rospy.spin()

  def pour_cmd_callback(self,req_raw):
    req = req_raw.pour_data
    # rospy.loginfo("in pour_cmd_callback")
    pour_control_success_bool = True
    if self.robot_setup_bool == False:
      #this is run only once to set the robot
      self.robot_setup(robot_type=req.robot_name)
      self.robot_setup_bool = True
      self.robot_name = req.robot_name
    if req.robot_name in "iiwa":
      pour_control_success_bool = self.command_iiwa(req)
    elif req.robot_name in "sawyer":
      self.command_sawyer(req)
    else:
      rospy.loginfo("valid robot name not given")
    #Return success bool
    resp = PourPlatformControlResponse()
    resp.success = pour_control_success_bool
    return resp



  def command_iiwa(self,req):
    jnt_names = req.joint_names
    if req.control_type in "velocity":
      pass
      return False
    elif req.control_type in "position":
      jnt_positions = req.positions
      self.move_group.moveToJointPosition(jnt_names, jnt_positions, wait=True) # Plans the joints in joint_names to angles in pose
      self.move_group.get_move_action().wait_for_result() # Since we passed in wait=False above we need to wait here
      result = self.move_group.get_move_action().get_result()
      move_completed_bool = False
      if result:
          # Checking the MoveItErrorCode
          if result.error_code.val == MoveItErrorCodes.SUCCESS:
              rospy.loginfo("Moved to pour position!")
              move_completed_bool = True
          else:
              # If you get to this point please search for:
              # moveit_msgs/MoveItErrorCodes.msg
              rospy.logerr("Arm goal in state: %s",
                           self.move_group.get_move_action().get_state())
      else:
          rospy.logerr("MoveIt! failure no result returned.")
      return move_completed_bool
    elif req.control_type in "torque":
      return False
    else:
      return False #no valid option was selected

  def command_sawyer(self,req):
    jnt_names = req.joint_names
    if req.control_type in "velocity":
      jnt_vel = req.velocities
      jnt_vel_cmd = dict(zip(jnt_names,jnt_vel)); print "cmd vel dict:", jnt_vel_cmd
      #self.sawyer_arm.set_joint_velocities(jnt_vel_cmd)
      jnt_cmd = JointCommand()
      jnt_cmd.header.stamp = rospy.Time()
      jnt_cmd.header.frame_id = "base"
      jnt_cmd.mode = 2
      jnt_cmd.names = jnt_names
      jnt_cmd.velocity = jnt_vel 
      print "joint command", jnt_cmd
      self.sawyer_control_pub.publish(jnt_cmd)
      return True
    elif req.control_type in "position":
      start_jnt_positions = req.positions
      start_pose_dict = dict(zip(jnt_names,start_jnt_positions))
      self.sawyer_limb.move_to_joint_positions(start_pose_dict)   
      return True
    elif req.control_type in "torque":
      jnt_trq = req.torques
      jnt_trq_dict = dict(zip(jnt_names,jnt_trq))
      # self.sawyer_arm.set_joint_torques(jnt_trq_dict)
      print "Commander: \njoints: ", jnt_names, "\n jnt torque", jnt_trq
      jnt_cmd = JointCommand()
      jnt_cmd.header.stamp = rospy.Time()
      jnt_cmd.header.frame_id = "base"
      jnt_cmd.mode = 3
      jnt_cmd.names = jnt_names
      jnt_cmd.effort = jnt_trq 
      self.sawyer_control_pub.publish(jnt_cmd)
      return True
    else:
      rospy.loginfo("Valid control type not specified, choose from: [position, velocity, torque]")
      return False


  def robot_setup(self, robot_type=None):
    if robot_type in "iiwa":
      self.move_group = MoveGroupInterface("manipulator","iiwa_link_0")
      self.move_group.setPlannerId("RRTConnectkConfigDefault")
      #For planning scene:
      base_link = "iiwa_link_0"
      self.planning_scene = PlanningSceneInterface(base_link)
      self.planning_scene.removeCollisionObject("the_ground")
      rospy.sleep(1.0)

    if robot_type in "sawyer":
      self.sawyer_rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
      self.sawyer_rs.enable()
      self.sawyer_arm = intera_interface.limb.Limb("right")
      self.sawyer_limb = intera_interface.Limb("right")



def main():
  rospy.init_node('sim_joint_commander_server')
  

  cls_obj = RobotJointCommanderCls()
  cls_obj.server_and_spin()

  if cls_obj.robot_name in "iiwa":
    #close out
    cls_obj.move_group.get_move_action().cancel_all_goals()





if __name__ == '__main__':
  main()
