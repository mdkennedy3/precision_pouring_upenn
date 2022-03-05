#!/usr/bin/env python
import rospy
import iiwa_msgs.msg as iiwa 
from sensor_msgs.msg  import JointState


import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as geom_msgs
import tf
from geometry_msgs.msg import WrenchStamped
from pouring_unknown_geom.msg import IsReady

from usb_scale.msg import Scale


#whenever importing from your library, always make last import the function.py from which you can access its classes
#e.g. don't do 'from pouring_unknown_geom import ts_math_operations' as you won't be able to access ts_math_oper
#but what is allowed is import pouring_unknown_geom.ts_math_operations.ts_math_oper as fnct to get it directly


# from pouring_unknown_geom.ts_math_operations import ts_math_oper 


#ts_math_oper_fncts. *TF_btw_frames, *.tf_to_mat, *transform_from_pose (multiply by vector: [])


#IN terminal that this is run in (or in the launch file)
# export ROS_NAMESPACE=/iiwa

#For Joint trajectories
#http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

class Motion_Options(object):
    def __init__(self, moveit_commander, display_trajectory_publisher):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator") 
        self.display_trajectory_publisher = display_trajectory_publisher
        self.angle = 0.0


        # self.ts_math_oper_fncts = ts_math_oper() 
        # self.ts_math_oper_fncts = ts_math_oper.ts_math_oper_fncts()




    def move_to_pose(self, pose_target):
        # rospy.sleep(10) #let rviz init

        #set the pose target
        # self.group.set_pose_target(pose_target)

        self.group.set_joint_value_target(pose_target)


        self.group.set_planner_id("RRTConnectkConfigDefault")
        plan1 = self.group.plan() 
        rospy.sleep(2) #gives rviz a chance to display plan 1

        #Display the planned trajectory: 

        disp_traj = moveit_msgs.msg.DisplayTrajectory()
        disp_traj.trajectory_start = self.robot.get_current_state()
        disp_traj.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(disp_traj) #display the traj
        rospy.sleep(2) #wait 5 seconds
        self.group.go(wait=True)


        # self.group.clear_pose_targets() 
        #clear targets


    def pour_out(self, angle):
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


    def callback(self, data):
        print "Current weight: {}  Current Angle: {}\n".format(data.weight, self.angle)





def main():
    moveit_commander.roscpp_initialize(sys.argv) #this is always necessary

    rospy.init_node('python_iiwa_test')
    #to visualize trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

    

    motion_obj = Motion_Options(moveit_commander, display_trajectory_publisher)
    # motion_obj.go_home_pose() #"""THIS SCRIPT ASSUMES YOU START FROM HOME POSITION """
    # '''NEED TO ADD SAFETY SURFACES '''


    # #Position to perform pouring action: 
    # # pose_target = geom_msgs.Pose()
    # # pose_target.position.x = 0.108614; pose_target.position.y = 0.5184; pose_target.position.z = 0.577136
    # # pose_target.orientation.w = 0.68312; pose_target.orientation.x = -0.711488; pose_target.orientation.y = -0.0894878; pose_target.orientation.z = 0.13829

    # pose_target = [-0.036999496721498666, 1.0101452240131397, 0.04424366107840788, -1.51033231686797, -0.12624796104481628, -0.893906571483256, 0.05936385810447666]


    # motion_obj.move_to_pose(pose_target)



    # motion_obj.group.clear_pose_targets() #clear targets


    # rospy.Subscriber("/usb_scale/scale", Scale, motion_obj.callback)

    # print "Subscribed\n"

    # group_variable_values = motion_obj.group.get_current_joint_values()

    # group_variable_values[5] = 0.0
    # group_variable_values[3] = 0.2
    # group_variable_values[1] = 1.5
    # motion_obj.group.set_joint_value_target(group_variable_values)
    # plan2 = motion_obj.group.plan()
    # motion_obj.group.go(wait=True)

    #Now perform pouring action

    motion_obj.pour_out(0.0)

    ready_pub = rospy.Publisher('/pouring_unknown_geom/status', IsReady, queue_size=1)
    rospy.sleep(5)
    ready_pub.publish(True)

    rospy.sleep(5)
    for ang in range(1, 31): 
        motion_obj.pour_out(-1 * ang / 10.)
        # rospy.sleep(1.0)

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

    ready_pub.publish(False)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()


