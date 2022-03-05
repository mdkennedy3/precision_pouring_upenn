#!/usr/bin/env python2
import numpy as np, math
import rospy
from intera_core_msgs.msg import JointCommand  #to publish types of joint commands
import threading
#my classes
from pouring_control_pkg.sawyer_control import sawyer_cntrl_cls 
from pouring_control_pkg.pouring_control import pouring_model_control_cls 
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot

from sensor_msgs.msg import JointState


class Pouring_Main(object):    
    def __init__(self, scale_or_vision, final_mh, final_mh_time, theta_start):
        self.r = threading.RLock()#pass this lock to all threading objects        
        self.sawyer_cntrl_cls = sawyer_cntrl_cls.Sawyer_Control() #velocity control, has a sawyer_cntrl_cls.cmd_vel(self,omega, joints)   
        self.final_mh = final_mh; self.final_mh_time = final_mh_time; self.scale_or_vision = scale_or_vision
        #Pouring class:        
        sand_density = {'Af_rho_inv':168./100.}
        self.pouring_calculations = pouring_model_control_cls.Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time) #optionally provide pouring model dictionary here
        # self.pouring_calculations = pouring_model_control_cls.Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time, sand_density) #optionally provide pouring model dictionary here
        self.pouring_calculations.calc_meas_pram_min_jerk_traj('init')  #find the optimal trajectory (function time 0:t)
        self.cntrl_var_mh = 0.
        self.cntrl_var_theta = 0.
        
        self.subs_var_mh = 0.  #either mass or height callback
        self.subs_var_mh_dot = 0.01  #small positive value for feedback linearization requirement
        self.subs_var_theta = 0.  #either mass or height callback

        #starting angle for the arm, this was positioncontrolled to this position before starting
        self.theta_start = theta_start


    def control_thread(self):
        print "control thread"

        self.t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            print "control loop"
            #Obtain States
            self.r.acquire(blocking=True)
            self.cntrl_var_mh = self.subs_var_mh  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
            self.cntrl_var_mh_dot = self.subs_var_mh_dot  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
            self.cntrl_var_theta = self.subs_var_theta  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
            self.r.release()

            #Calculate omega input
            """Set the correct input after w call """
            self.t_curr = rospy.Time.now().to_sec()
            self.dt_curr = self.t_curr - self.t_start
            
            #Calculate theta wrt the upwrite orientation
            
            theta = self.cntrl_var_theta - self.theta_start


            h = self.cntrl_var_mh
            dh = self.cntrl_var_mh_dot
            omega, t_start_new = self.pouring_calculations.calc_omega(self.dt_curr, theta, h, dh) #omega = -0.5; 
            if type(t_start_new) != list:
                self.t_start = t_start_new
            print "\n Desired omega", omega
            # rospy.loginfo("Desired omega %f", omega)
            joint = 'right_j6' 
            #Command omega
            
            if self.dt_curr > self.final_mh_time:
                #if outside of trajectory time, then just command zero omega
                print "\n Operation complete, w=0 now"
                omega = 0.0; 

            self.sawyer_cntrl_cls.cmd_vel(omega, joint)


    def mh_callback(self,req):
        #req is callback obj
        self.r.acquire(blocking=True)
        self.subs_var_mh = req.mh #locking is occuring on subs_var_mh (to avoid memory accessing issues)
        self.subs_var_mh_dot = req.mh_dot
        self.r.release()

    def theta_callback(self,req):
        self.r.acquire(blocking=True)
        
        self.subs_var_theta = req.position[req.name.index('right_j6')] #locking is occuring on subs_var_mh (to avoid memory accessing issues)        
        vel = req.velocity[req.name.index('right_j6')]
        torque = req.effort[req.name.index('right_j6')]


        self.r.release()


    def init_subscribers(self):
        print "subscriber node"
        """TODO: SET THESE AND GET CORRECT COMPONENTS"""
        self.theta_sub = rospy.Subscriber('/robot/joint_states',JointState, self.theta_callback) #joint angle callback
        self.mh_sub = rospy.Subscriber('/mh_mh_dot',MeasMsg, self.mh_callback) #mass (m), height (h) callback
        #set all subscribers (remembering to aquire and release when setting variables)

        rospy.spin()


def main():
    print "main script"
    rospy.init_node('pouring_control_main_node')
    #terms for pouring
    scale_or_vision = 'scale'
    final_mh = 100 #ml or grams
    # final_mh = 30 #FOR SAND ml or grams 
    final_mh_time = 10 #seconds
    theta_start = -1.2372  #radians (set this approrpriately (doesn't have to match the actual start position, this is just reference frame))
    pouring_main_obj = Pouring_Main(scale_or_vision, final_mh, final_mh_time, theta_start)
    #init_threads
    sub_scribe_thread = threading.Thread(target=pouring_main_obj.init_subscribers) #Set Subscriber Thread
    control_thread = threading.Thread(target=pouring_main_obj.control_thread) #Set Control Thread
    #start threads:
    sub_scribe_thread.start()

    #maybe start control thread on click
    control_thread.start()
    




if __name__ == '__main__':
    main()
