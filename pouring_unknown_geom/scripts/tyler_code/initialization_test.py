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
from iiwa_pouring.msg import IsReady

from usb_scale.msg import Scale


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

    def stop(self):
        self.group.stop()





def main():
    moveit_commander.roscpp_initialize(sys.argv) #this is always necessary

    rospy.init_node('python_iiwa_init_test')
    #to visualize trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

    

    motion_obj = Motion_Options(moveit_commander, display_trajectory_publisher)
    motion_obj.go_home_pose() #"""THIS SCRIPT ASSUMES YOU START FROM HOME POSITION """
    '''NEED TO ADD SAFETY SURFACES '''


    #Position to perform pouring action: 
    # pose_target = geom_msgs.Pose()
    # pose_target.position.x = 0.108614; pose_target.position.y = 0.5184; pose_target.position.z = 0.577136
    # pose_target.orientation.w = 0.68312; pose_target.orientation.x = -0.711488; pose_target.orientation.y = -0.0894878; pose_target.orientation.z = 0.13829

    # pose_target = [-0.038576740621799666, 1.0263869047121084, 0.04592684548255723, -1.5048551666063466, -0.12668044769934103, -0.9693528998251715, 0.0024136827995930037]

    pose_target = [-0.037577196340585664, 0.8013832618250014, 0.05339888984408386, -1.5534791218509714, -0.12646118442887394, -0.7961050496456741, 2.9995571180903267]



    motion_obj.move_to_pose(pose_target)



    motion_obj.group.clear_pose_targets() #clear targets


    # Pause for user to adjust grip and insert pouring container
    motion_obj.pour_out(0.0)
    cid = raw_input("Enter container ID (default is 000) and hit enter: ")
    if not cid:
        cid = '000'
    vol = input("Enter initial volume and hit enter: ")
    raw_input("Adjust grip and press enter to continue...")




    #Now perform pouring action

    

    ready_pub = rospy.Publisher('/iiwa_pouring/status', IsReady, queue_size=1)
    rospy.sleep(5)
    ready_pub.publish(IsReady(True, cid, vol))

    rospy.sleep(5)
    
    for ang in range(1, 62):
        try:
            if (rospy.is_shutdown()):
                motion_obj.stop()
                break 
            motion_obj.pour_out(ang / 20.)
            rospy.sleep(5.0)
        except (KeyboardInterrupt):
            break
        except (moveit_commander.MoveItCommanderException):
            pass
            break
        except (rospy.ROSInterruptException):
            pass
            break

    

    motion_obj.stop()
    print "stopped"

    ready_pub.publish(IsReady(False, cid, vol))

    rospy.signal_shutdown("Done")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
