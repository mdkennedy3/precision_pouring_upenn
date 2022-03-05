#!/usr/bin/env python
import rospy
import iiwa_msgs.msg as iiwa 
from sensor_msgs.msg  import JointState


import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as geom_msgs

#IN terminal that this is run in (or in the launch file)
# export ROS_NAMESPACE=/iiwa

class Motion_Options(object):
    def __init__(self,moveit_commander,display_trajectory_publisher):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator") 
        self.display_trajectory_publisher = display_trajectory_publisher


    def move_to_pose(self,pose_target):
        # rospy.sleep(10) #let rviz init

        #set the pose target
        self.group.set_pose_target(pose_target)
        self.group.set_planner_id("RRTConnectkConfigDefault")
        plan1 = self.group.plan(); rospy.sleep(2) #gives rviz a chance to display plan 1

        #Display the planned trajectory: 

        disp_traj = moveit_msgs.msg.DisplayTrajectory()
        disp_traj.trajectory_start = self.robot.get_current_state()
        disp_traj.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(disp_traj) #display the traj
        rospy.sleep(2) #wait 5 seconds
        self.group.go(wait=True)


        # self.group.clear_pose_targets() #clear targets



    def move_joint_angles(self):
        group_variable_values = self.group.get_current_joint_values()
        print "============ Joint values: ", group_variable_values
        # group_variable_values[0] = 1.0 #change this one joint
        group_variable_values[6] = 1.0
        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        print "============ Waiting while RVIZ displays plan2..."
        rospy.sleep(1) #not required
        self.group.go(wait=True)


    def move_through_wypts(self):
        #Go through waypoints
        """CAREFUL: THIS IS MESSY, NEED TO LEARN HOW TO CHECK FEASIBILITY FIRST (and exceptions to catch)"""
        waypoints = []
        waypoints.append(self.group.get_current_pose().pose)
        #first orient the gripper then move foward
        wpose = geom_msgs.Pose()
        wpose.orientation.w = 1.0
        wpose.position.x = waypoints[0].position.x + 0.1
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))

        # second move down
        wpose.position.z -= 0.10
        waypoints.append(copy.deepcopy(wpose))

        # third move to the side
        wpose.position.y += 0.05
        waypoints.append(copy.deepcopy(wpose))

        (plan3, fraction) = self.group.compute_cartesian_path(
                                     waypoints,   # waypoints to follow
                                     0.01,        # eef_step
                                     0.0)         # jump_threshold
        print "============ Waiting while RVIZ displays plan3..."
        rospy.sleep(5)




def main():

    def jointposCallback(jp):
        print "\n\n\n joint position: ",  jp.position
        print "\n joint velocity: ",  jp.velocity

    moveit_commander.roscpp_initialize(sys.argv)#this is always necessary

    rospy.init_node('python_iiwa_test')
    #to visualize trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

    motion_obj= Motion_Options(moveit_commander,display_trajectory_publisher)


    #should query state, but issue is: Robot semantic description not found. Did you forget to define or remap '/robot_description_semantic'?
    # robot = moveit_commander.RobotCommander()
    # print"current state from moveit_commander: ",robot.get_current_state()
    # scene = moveit_commander.PlanningSceneInterface()
    # group = moveit_commander.MoveGroupCommander("manipulator") 

    #plan with random position
    # rand_goal_pose= group.get_random_joint_values()
    # path_to_rand_goal = group.plan(rand_goal_pose)
    # print "path to random pose: ", path_to_rand_goal

    #Plan to a given pose: 
    pose_target = geom_msgs.Pose()
    pose_target.position.x = -0.066; pose_target.position.y = -0.159; pose_target.position.z = 1.169
    pose_target.orientation.w = 0.707068357078; pose_target.orientation.x = 6.4378231585e-05; pose_target.orientation.y = -0.707145199351; pose_target.orientation.z = 3.61860014377e-05;

    motion_obj.move_to_pose(pose_target)

    motion_obj.group.clear_pose_targets() #clear targets

    motion_obj.move_joint_angles()

    #waypoints: 
    # motion_obj.move_through_wypts()




    '''So move to a desired position in cartesian, then move wrist to desired orientation and iterate in joint space for pouring motion!!!! '''


    # msg_str = "/iiwa/joint_states"; #or : # msg_str=  "/iiwa/state/JointPosition"
    # print "msg string is: ", msg_str
    # sub = rospy.Subscriber(msg_str,JointState,jointposCallback)
    # print "Subscriber called"
    # rospy.spin()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()