#!/usr/bin/env python2
import numpy as np, math
import rospy, rospkg
import sys, getopt
from intera_core_msgs.msg import JointCommand  #to publish types of joint commands
import threading
#my classes
from pouring_control_pkg.sawyer_control import sawyer_cntrl_cls 
from pouring_control_pkg.pouring_control import pouring_model_control_cls 
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot

from sensor_msgs.msg import JointState

#for plotting:

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



class Pouring_Main(object):    
    def __init__(self, scale_or_vision, final_mh, final_mh_time, theta_start):
        # self.r = threading.RLock()#pass this lock to all threading objects        
        self.sawyer_cntrl_cls = sawyer_cntrl_cls.Sawyer_Control() #velocity control, has a sawyer_cntrl_cls.cmd_vel(self,omega, joints)   
        self.final_mh = final_mh; self.final_mh_time = final_mh_time; self.scale_or_vision = scale_or_vision
        #Pouring class:        
        sand_density = {'Af_rho_inv':168./100.}

        if scale_or_vision in "vision":
            Af_rho_inv = {'Af_rho_inv':168./100.}
        elif scale_or_vision in "scale":
            Af_rho_inv = {'Af_rho_inv':41.66} #cm**2

        self.pouring_calculations = pouring_model_control_cls.Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time, Af_rho_inv) #optionally provide pouring model dictionary here
        # self.pouring_calculations = pouring_model_control_cls.Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time, sand_density) #optionally provide pouring model dictionary here
        self.pouring_calculations.calc_meas_pram_min_jerk_traj('init')  #find the optimal trajectory (function time 0:t)
        self.cntrl_var_mh = 0.
        self.cntrl_var_theta = theta_start
        self.subs_var_mh = 0.  #either mass or height callback
        self.subs_var_mh_dot = 0.01  #small positive value for feedback linearization requirement
        self.subs_var_theta = theta_start  #either mass or height callback
        #starting angle for the arm, this was positioncontrolled to this position before starting
        self.theta_start = theta_start
        #For plotting
        self.mh_plot = []
        self.mh_dot_plot = []
        self.theta_plot = []
        self.omega_plot = []
        self.time_plot_start = rospy.Time.now().to_sec()
        self.time_plot = []


    def control_thread(self):
        #Obtain States
        self.cntrl_var_mh = self.subs_var_mh  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
        self.cntrl_var_mh_dot = self.subs_var_mh_dot  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
        self.cntrl_var_theta = self.subs_var_theta  #locking is occuring on subs_var_mh (to avoid memory accessing issues)


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
            print "time reset, dt:", self.dt_curr 
        print "\n Desired omega", omega, " theta: ", theta
        # rospy.loginfo("Desired omega %f", omega)
        joint = 'right_j6' 
        #Command omega
        if self.dt_curr > self.final_mh_time:
            #if outside of trajectory time, then just command zero omega
            print "\n Operation complete, w=0 now"
            omega = 0.0; 
        self.sawyer_cntrl_cls.cmd_vel(omega, joint)

        self.mh_plot.append(h)
        self.mh_dot_plot.append(dh)
        self.theta_plot.append(theta)
        self.omega_plot.append(omega)
        self.time_plot.append(self.t_curr - self.time_plot_start)


    def plot_results(self):
        #correct type
        fig1 = plt.figure(); ax1 = fig1.add_subplot(111)
        ax1.set_ylabel('$m$ (grams)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
        ax1.set_xlabel('$t$ (sec)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
        ax1.plot(self.time_plot, self.mh_plot,'-b', label="$mh$")
        ax1.plot(self.time_plot, self.mh_dot_plot,'--r', label="$\\dot{mh}$")
        ax1.set_xticks(np.arange(0,self.time_plot[-1],.5))
        ax1.set_yticks(np.arange(0,120,5))
        plt.grid()
        ax1.legend(loc=2,fontsize=20)
        fig2 = plt.figure(); ax2 = fig2.add_subplot(111)
        ax1.set_ylabel('angle (rad)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
        ax1.set_xlabel('$t$ (sec)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
        ax2.plot(self.time_plot, self.theta_plot,'--g', label="$\\theta$")
        ax2.plot(self.time_plot, self.omega_plot,'-b', label="$\\omega$")
        ax2.set_xticks(np.arange(0,self.time_plot[-1],.5))
        ax2.set_yticks(np.arange(-math.pi,math.pi,1))   
        plt.grid()
        ax2.legend(loc=2,fontsize=20)
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        file_path = rospack.get_path('pouring_control_pkg')
        fname = file_path + '/scripts/figs/'
        fig1.set_size_inches(18.5, 10.5)
        f1name = fname + 'mh_mh_dot.png'
        fig1.savefig(f1name, dpi=100)

        fig2.set_size_inches(18.5, 10.5)
        f2name = fname + 'omega_theta.png'
        fig2.savefig(f2name, dpi=100)
        plt.show()

    def mh_callback(self,req):
        #req is callback obj
        self.subs_var_mh = req.mh #locking is occuring on subs_var_mh (to avoid memory accessing issues)
        self.subs_var_mh_dot = req.mh_dot
        self.control_thread()

    def theta_callback(self,req):
        self.subs_var_theta = req.position[req.name.index('right_j6')] #locking is occuring on subs_var_mh (to avoid memory accessing issues)        
        vel = req.velocity[req.name.index('right_j6')]
        torque = req.effort[req.name.index('right_j6')]

    def init_subscribers(self):
        print "subscriber node"
        self.t_start = rospy.Time.now().to_sec()
        self.theta_sub = rospy.Subscriber('/robot/joint_states',JointState, self.theta_callback) #joint angle callback
        self.mh_sub = rospy.Subscriber('/mh_mh_dot',MeasMsg, self.mh_callback) #mass (m), height (h) callback
        rospy.spin()


def main(argv):
    print "main script"
    try: 
        opts, args = getopt.getopt(argv, "hd:t:",["des_height=","des_time"])
    except getopt.GetoptError:
        print "pub_desired_height.py -d height -t time"

    ml_height = 100. #ml
    traj_time = 10. #sec

    for opt, arg in opts:
        if opt == '-h':
            print "pub_desired_height.py -d value"
            # sys.exit()
        elif opt in ["-d","--des_height"]:
            print "req arg: ", arg
            ml_height = float(arg) 
        elif opt in ["-t","--des_time"]:
            traj_time = float(arg)



    rospy.init_node('pouring_control_main_node')
    #terms for pouring
    scale_or_vision = 'vision'
    final_mh = ml_height# 100 #ml or grams  (50,100,150)
    final_mh_time = traj_time#10 #seconds  (8,10,12)
    theta_start = 1.57323046875 #-1.2372  #radians (set this approrpriately (doesn't have to match the actual start position, this is just reference frame))
    pouring_main_obj = Pouring_Main(scale_or_vision, final_mh, final_mh_time, theta_start)
    #init_threads
    pouring_main_obj.init_subscribers()
    #when everything completes, plot
    # pouring_main_obj.plot_results()




if __name__ == '__main__':
    main(sys.argv[1:])
