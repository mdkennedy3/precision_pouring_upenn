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



    def pour_out(self,angle):
        #angle is angle of the wrist
        group_variable_values = self.group.get_current_joint_values()
        # print "============ Joint values: ", group_variable_values
        # group_variable_values[0] = 1.0 #change this one joint
        group_variable_values[6] = angle
        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        print "============ Pouring out Liquid at angle: ", angle
        # rospy.sleep(1) #not required
        self.group.go(wait=True)



    def go_home_pose(self):
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values = [9.315686020805458e-05, 3.071415349609197e-05, -6.5161927516044216e-06, -8.568950793996777e-05, 2.2710875017928345e-05, -9.358478682930382e-05, -3.135311386603945e-05]
        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        # rospy.sleep(1) #not required
        self.group.go(wait=True)





def main():
    moveit_commander.roscpp_initialize(sys.argv)#this is always necessary

    rospy.init_node('python_iiwa_test')
    #to visualize trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

    

    motion_obj= Motion_Options(moveit_commander,display_trajectory_publisher)
    motion_obj.go_home_pose() #"""THIS SCRIPT ASSUMES YOU START FROM HOME POSITION """
    '''NEED TO ADD SAFETY SURFACES '''


    #Position to perform pouring action: 
    pose_target = geom_msgs.Pose()
    pose_target.position.x = 0.256904991137; pose_target.position.y = 0.119652839454; pose_target.position.z = 0.874820386446
    pose_target.orientation.w = 0.997903283723; pose_target.orientation.x = 0.0646627990248; pose_target.orientation.y = -0.00278332528021; pose_target.orientation.z = 0.000108896731512
    motion_obj.move_to_pose(pose_target)



    motion_obj.group.clear_pose_targets() #clear targets


    #Now perform pouring action
    angle = [0.,0.2,0.4,0.6,0.8,1.0]
    for ang in angle:
        motion_obj.pour_out(ang)
        rospy.sleep(2.)

    #waypoints: 
    # motion_obj.move_through_wypts()




    '''So move to a desired position in cartesian, then move wrist to desired orientation and iterate in joint space for pouring motion!!!! '''



    # def jointposCallback(jp):
    #     print "\n\n\n joint position: ",  jp.position
    #     print "\n joint velocity: ",  jp.velocity

    # msg_str = "/iiwa/joint_states"; #or : # msg_str=  "/iiwa/state/JointPosition"
    # print "msg string is: ", msg_str
    # sub = rospy.Subscriber(msg_str,JointState,jointposCallback)
    # print "Subscriber called"
    # rospy.spin()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()