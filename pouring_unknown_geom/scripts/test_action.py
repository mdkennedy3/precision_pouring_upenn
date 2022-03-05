#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_srvs.srv import Empty, Trigger, SetBool, SetBoolRequest
from std_srvs.srv import EmptyResponse

from geometry_msgs.msg import Pose
#from pouring_smach.states.MoveArmState import MoveArmState
#from pouring_smach.others.moveit_motions import Motion_Options

#from pouring_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupResult

from pouring_unknown_geom.msg import PourControlAction, PourControlGoal, PourControlResult

# state naming conventions
# UPPER_CASE - state machines or states which are state machines
# CammelCase - regular states
# lower_case - transitions between states

def main():

  rospy.init_node('test_state_machine')

  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
  sm.userdata.userdata_input = 'GoPick'

  # Run the state machine with the introspection server
  sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT/MAIN');
  sis.start();

  with sm:

    def movegroup_goal_cb(userdata, goal):
      pour_goal = PourControlGoal()
      pour_goal.desired_volume_ml = 100
      return pour_goal

    smach.StateMachine.add('RESET_DETECTOR',
        smach_ros.ServiceState('/pouring_control/level_detector_node_cpp/reset', Trigger),
        transitions={'succeeded':'RESET_MODEL_SELECTION', 'aborted': 'aborted', 'preempted':'preempted'})

    smach.StateMachine.add('RESET_MODEL_SELECTION',
        smach_ros.ServiceState('/pouring_control/reset_model_selection_node', Trigger),
        transitions={'succeeded':'RESET_POUR_VOL_ANG', 'aborted': 'aborted', 'preempted':'preempted'})

    smach.StateMachine.add('RESET_POUR_VOL_ANG',
        smach_ros.ServiceState('/pouring_control/reset_pour_vol_ang', Trigger),
        transitions={'succeeded':'POUR_ACTION', 'aborted': 'aborted', 'preempted':'preempted'})

    smach.StateMachine.add('POUR_ACTION',
      smach_ros.SimpleActionState('/pouring_control/pour_action', PourControlAction,
          goal_cb=movegroup_goal_cb,
          server_wait_timeout = rospy.Duration(2.0),
          exec_timeout=rospy.Duration(30.0), preempt_timeout = rospy.Duration(10.0),
          input_keys=['movegroup_input']),
      transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'},remapping={'movegroup_input':'userdata_input'})


  # Execute SMACH plan
  outcome = sm.execute()

  rospy.spin()

  sis.stop()

if __name__ == '__main__':
  main()
