#!/usr/bin/env python2
import numpy as np, math
import rospy
from intera_core_msgs.msg import JointCommand  #to publish types of joint commands



class Sawyer_Control(object):
    """
    Velocity_Control_Test commands the velocity using the JointCommand message type for sawyer (intera_core_msgs)
    -During class initialization, an alternate vel topic may be specified
    -to command velocity of single or multiple joints, pass to cmd_vel as either value, string, or lists of values, strings to be commanded
    -------
    -joint choices of: ['head_pan','right_j0','right_j1','right_j2','right_j3','right_j4','right_j5','right_j6','torso_t0']
    -modes are (position: 1), (vel: 2), (torque: 3), (trajectory 4)
    """
    def __init__(self, *cntrl_topic):        
        if len(cntrl_topic) == 0:
            self.cntrl_topic = '/robot/limb/right/joint_command'
        else:
            self.cntrl_topic = cntrl_topic[0]
            
        self.command_pub = rospy.Publisher(self.cntrl_topic,JointCommand,queue_size=1, latch=True)



    def cmd_vel(self,omega, joints):
        #joint_list choices of: ['head_pan','right_j0','right_j1','right_j2','right_j3','right_j4','right_j5','right_j6','torso_t0']
        #omega_list should correspond
        pub_msg= JointCommand()
        pub_msg.mode = 2  #modes are (position: 1), (vel: 2), (torque: 3), (trajectory 4)
        pub_msg.header.stamp = rospy.Time.now()

        if type(omega) == list and type(joints) == list:
            #loop through joints
            pub_msg.names = [] 
            pub_msg.velocity = []
            for idx in range(len(joints)):
                pub_msg.names.append(joints[idx])
                pub_msg.velocity.append(omega[idx])
        else:        
            pub_msg.names = [joints] 
            pub_msg.velocity = [omega]

        self.command_pub.publish(pub_msg)


    def cmd_pos(self,pos,joints):
        pub_msg= JointCommand()
        pub_msg.mode = 1  #modes are (position: 1), (vel: 2), (torque: 3), (trajectory 4)

        if type(pos) == list and type(joints) == list:
            #loop through joints
            pub_msg.names = [] 
            pub_msg.position = []
            for idx in range(len(joints)):
                pub_msg.names.append(joints[idx])
                pub_msg.position.append(pos[idx])
        else:        
            pub_msg.names = [joints] 
            pub_msg.position = [pos]

        self.command_pub.publish(pub_msg)



    def cmd_torque(self,torque,joints):
        pub_msg= JointCommand()
        pub_msg.mode = 3  #modes are (position: 1), (vel: 2), (torque: 3), (trajectory 4)

        if type(torque) == list and type(joints) == list:
            #loop through joints
            pub_msg.names = [] 
            pub_msg.effort = []
            for idx in range(len(joints)):
                pub_msg.names.append(joints[idx])
                pub_msg.effort.append(torque[idx])
        else:        
            pub_msg.names = [joints] 
            pub_msg.effort = [torque]

        self.command_pub.publish(pub_msg)






def main():
    #This is used to validate the control scheme: 
    print "Velocity Control Test"
    rospy.init_node('vel_cntrl_test_node'); 
    #msg_name = '/robot/limb/right/joint_command'; pub = rospy.Publisher(msg_name,JointCommand,queue_size=1, latch=True)
    vel_cls = Sawyer_Control()
    while not rospy.is_shutdown():

        omega = 0.5  #max is 1.5, 2.
        joint = 'right_j6'
        vel_cls.cmd_vel(omega, joint)



        
if __name__ == '__main__':
    main()
