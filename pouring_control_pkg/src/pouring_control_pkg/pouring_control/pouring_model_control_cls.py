import numpy as np, math
import rospy
import time
from scipy.integrate import ode
from pouring_control_pkg.min_jerk import min_jerk_traj 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




class Pouring_Calculations_Class(object):
    """docstring for Pouring_Calculations_Class"""
    def __init__(self, scale_or_vision, final_mh, final_mh_time, *parameter_dict):        
        # scale_or_vision: either ['scale', 'vision']
        # self.Kp = 1.5  #Gain for mh error
        # self.Kd = 2.0  #Gain for mh error

        # self.Kp = 2.0  #Gain for mh error
        # self.Kd = 4.0  #Gain for mh error

        # self.Kp = 1.0  #Gain for mh error
        # self.Kd = 2.0  #Gain for mh error

        self.Kp = 1.0  #Gain for mh error
        self.Kd = 1.0  #Gain for mh error
        self.Ki = 0.0 # 0.05
        self.Pi_error = []

        self.final_mh = final_mh  #scalar value specifying target mass or height (mass in grams, height in cm)
        self.final_mh_time = final_mh_time #scalar value in seconds specifiying time to pour

        #Set Default Parameters
        units = {'m':1.0, 'cm':100.0}
        self.params_dict = {'Lf':0.05,'g':9.81,'W':0.05,'l1':0.05,'Af_rho_inv':(1./1.0028),'mh':0.0,'mh_dot':0.0,'theta':0.0}
        #l1 = 7cm = 0.07m
        #W = 5cm = 0.05m
        #Lf = 5cm = 0.05m
        #Af: 0.005134 m^2, as circ is 25.2cm (flask poured into)
        #rho: 1g/ml (1g/cm^3)   106/100
        #mh: either m or h
        '''' LET MASS BE grams, and Volume ml '''
        #mh_dot: derivative or m, h resp.
        #theta: (rad) rotation angle measured from upright pose (so command it to go to init position (for onset of pouring, and reset this angle (rad)))
        if len(parameter_dict) >0:
            #set the parameters from the dictionary
            try:
                for keys in parameter_dict[0]:                
                    self.params_dict[keys] = parameter_dict[0][keys] #should have all the same keys 
            except:
                print "Incorrect Keys were passed"
        length_keys = ['Lf','W','g','l1','mh','mh_dot']; area_keys ='Af_rho_inv'
        #Convert to whichever unit base desired
        for keys in length_keys: self.params_dict[keys] = self.params_dict[keys]*units['cm']
        '''change this based on method '''
        # Already converted to cm**2
        # if scale_or_vision in 'vision':
        #     '''DOUBLE CHECK THESE '''
        #     self.params_dict[area_keys] = self.params_dict[area_keys]*(units['cm']**2)  #if using area A [m^2]: hence multiply cm^2/m^2
        # elif scale_or_vision in 'scale':
        #     self.params_dict[area_keys] = self.params_dict[area_keys]#*(units['cm']**3) #if using density: 1/rho = 1/(kg/m^3)=m^3/kg hence multiply cm^3/m^3
        '''Unit change useful for control of qdot, as there are power terms that may be unstable when modeling flowrate m^3/s vs cm^3/s '''
        '''BE CONSISTANT IN UNITS (ALL CM OR ALL M for mh, mh_dot) '''
        #pouring model terms
        self.Q1 = []
        self.Q2 = []
        self.Q3 = []

    def calc_meas_pram_min_jerk_traj(self, *init):
        if len(init) >0 and init[0] in ['init','restart'] :
            print "find the trajectory of the fluid that minimizes jerk" #pouring trajectory
            bc = {"i":[], "f":[], 't':[]}
            bc["i"] = [0.,0.,0.] #initial mass/height, 1st, 2nd derivatives
            bc["f"] = [self.final_mh,0.,0.] #final mass or height, first derivative, second derivative (zero as we want steady constant at end)
            bc["t"] = [0.,self.final_mh_time] #init, final time to pour
            self.min_jerk_traj_calc = min_jerk_traj.MinJerk(bc)
            self.min_jerk_poly = self.min_jerk_traj_calc.min_jerk_poly() #coefficients for curve
            self.traj_init_bool = True
            #calculate first states
            sys_states= self.min_jerk_traj_calc.system_state(self.min_jerk_poly, 0.)
            self.mh_des = sys_states.item(0)
            self.mh_des_dot = sys_states.item(1)
            self.mh_ddot_des = sys_states.item(2)
        else:
            try:
                if not self.traj_init_bool: print "need to initialize pouring trajectory"    
            except:
                print "need to initialize pouring trajectory"
            if init[0] in ['update']:
                t = init[1] #pass time in the second field
                #calculate first states
                if t <= self.final_mh_time:
                    sys_states = self.min_jerk_traj_calc.system_state(self.min_jerk_poly, t)
                    self.mh_des = sys_states.item(0)
                    self.mh_des_dot = sys_states.item(1)
                    self.mh_ddot_des = sys_states.item(2)
                else:
                    self.mh_des = self.final_mh
                    self.mh_des_dot = 0.0
                    self.mh_ddot_des = 0.0

    def obtain_pouring_model_terms(self, *theta_input):
        """Requires that theta be updated to current value """
        if len(theta_input) > 0:
            theta = theta_input[0]
            self.params_dict['theta'] = theta_input[0]
        else:
            theta = self.params_dict['theta']  #either replace it in the param_dict or send it         
        Lf = self.params_dict['Lf']
        g = self.params_dict['g']
        W = self.params_dict['W']
        l1 = self.params_dict['l1']
        risidual_deg = 6.*math.pi/180. #at 6 degrees cos(6) = 0.10452, sec(90-6) = 9.56, at sec(90-3 deg): 19.1, at sec(90- 1deg): 57.29
        if (np.abs(theta - (math.pi/2.)) < risidual_deg) or (np.abs(theta + (math.pi/2.)) < risidual_deg):
            print "theta dangerously close to singularity"
            """Threshold the following values to handle instability near the singularity """
            if np.abs(theta) > 0.001:
                theta_sign = np.sign(np.cos(theta))
            else:
                theta_sign = 1. #(less than 1 deg)
            self.Q1 = -0.1*theta_sign #-(2./3.)**(1./3.) *(Lf*np.sqrt(2*g))**(2./3.) * (np.cos(theta)/(W*l1))                           
            self.Q2 = -9.56*theta_sign*np.sign(np.sin(theta)) #-(2./3.)**(-1./3.) * np.tan(theta)
            self.Q3 = 4.14*theta_sign # (1./12.)**(1./3.)*l1*(Lf*np.sqrt(2*g))**(2./3.) (1./np.cos(theta))
        else:
            self.Q1 = -(2./3.)**(1./3.) *(Lf*np.sqrt(2*g))**(2./3.) * (np.cos(theta)/(W*l1))                           
            self.Q2 = -(2./3.)**(-1./3.) * np.tan(theta)
            self.Q3 = (1./12.)**(1./3.) *l1 *(Lf*np.sqrt(2*g))**(2./3.) *(1./np.cos(theta))

        return [self.Q1, self.Q2, self.Q3]

    def calc_omega_feedback(self, mh, mh_dot, theta):
        #Desired trajectory terms
        mh_des = self.mh_des
        mh_des_dot = self.mh_des_dot
        mh_ddot_des = self.mh_ddot_des
        w_ff = mh_ddot_des #feedforward expression
        #Calculate Response (feedforward + feedback)

        #conditions for integral error (with sliding window)

        if mh >= self.final_mh:
            self.Pi_error = [] #remove integral error if youve hit or exceeded your goal

        if len(self.Pi_error) > 0 and sum(self.Pi_error) >= 0.0:
            mh_error = w_ff + self.Kp*(mh_des - mh) + self.Ki*sum(self.Pi_error)
        else:
            mh_error = w_ff + self.Kp*(mh_des - mh)

        self.Pi_error.append(mh_error)
        if len(self.Pi_error) >= 10:
            self.Pi_error = self.Pi_error[1:]


        if np.abs(mh_des_dot ) > 0.001:
            mh_error +=  self.Kd*(mh_des_dot - mh_dot)  #for the second order response
        # mh_error = np.
        Af_rho_inv = self.params_dict['Af_rho_inv']
        """HYBRID CONTROL STRATEGY """
        #Handle case if mhdot = 0
        min_omega = 0.1 #pouring speed when mh_dot stops
        if mh_dot < 0.01 and 0.9*self.mh_des >= mh :
            #Hybrid state 2
            #if the vel stops and you have more than 10% of desired mh (simply move forward)
            # if np.sign(mh) == 0:
            #     w_fb = min_omega #small positive value
            # else:
            #     w_fb =  np.sign(mh)*min_omega #small positive value
            '''only pour forward for now '''
            w_fb = min_omega #small positive value
            t_start_new = []
        elif mh_dot < 0.01 and 0.05*np.abs(self.mh_des) >= mh:
            #Hybrid state 2
            #if the vel stops and you have less than 5% of desired mh (move forward slowly and reset trajectory)
            # if np.sign(mh) == 0:
            #     w_fb = min_omega #small positive value
            # else:
            #     w_fb =  np.sign(mh)*min_omega #small positive value
            '''only pour forward for now '''
            w_fb = min_omega #small positive value
            t_start_new = rospy.Time.now().to_sec()
            dt = 0.0
            self.calc_meas_pram_min_jerk_traj('update', dt)
        elif mh_dot < 0.01 and mh > 0.99*self.mh_des:
            #Hybrid state 3
            #if the velocity stops and your within 1% of your goal stop
            w_fb = 0.0 #stop as you've approached it and the cup is empty now
            t_start_new = []
        elif mh_dot <= 0.01:
            #Hybrid state 2
            # if np.sign(mh) == 0:
            #     w_fb = min_omega #small positive value
            # else:
            #     w_fb =  np.sign(mh)*min_omega #small positive value
            '''only pour forward for now '''
            w_fb = min_omega #small positive value
            t_start_new = []
        else:
            if theta >= math.pi/2 or theta <= -math.pi/2:
                #Hybrid state 3
                w_fb = 0.0
            else:
                #Hybrid state 1
                w_fb = (self.Q2*Af_rho_inv*mh_dot + self.Q3*Af_rho_inv**(1./3.) *mh_dot**(1./3.) )**(-1.) * ( Af_rho_inv* mh_error - self.Q1*Af_rho_inv**(4./3.)*mh_dot**(4./3.))
            t_start_new = []
        return w_fb, t_start_new

    def calc_omega(self, dt, theta, mh, mh_dot):
        #take input params: dt(time since start), theta (measured angle from start vertical), h (height/mass), dh (derivative height/mass)
        #calculate updated terms
        Q_list= self.obtain_pouring_model_terms(theta)
        #Calculate the current desired mh, mhdot, mhddot
        self.calc_meas_pram_min_jerk_traj('update', dt)
        #Calculate the feedforward and feedback terms
        w_fb, t_start_new = self.calc_omega_feedback(mh, mh_dot, theta)
        """ get feedforward working first """
        w = w_fb #contains feedforward in second derivative
        #threshold value: 
        if np.abs(w) > 1.5:
            w = np.sign(w)*1.5
        return w, t_start_new

    def ode_test(self,t,x):
        #this function returns the derivative of the state space
        # [dx1,dx2,dx3] = f(x) + g(x)*u
        #let x1 = mh, x2 = mhdot, x3 = theta, u = omega
        mh = x[0]; mh_dot = x[1]; theta = x[2]
        #1. Calculate Dynamical terms Q1,Q2,Q3
        Q_list = self.obtain_pouring_model_terms(theta)
        Q1 = Q_list[0]
        Q2 = Q_list[1]
        Q3 = Q_list[2]
        #2. Calculate Omega
        w, t_start_new = self.calc_omega(t, theta, mh, mh_dot)
        try:
            self.w_plot.append(w)
            self.w_plot_t.append(t)
        except:
            self.w_plot = []
            self.w_plot_t = []
            self.w_plot.append(w)
            self.w_plot_t.append(t)
        #3. Build Dynamical equations
        x1 = x[0]; x2 = x[1]; x3 = x[2]; Af_rho_inv = self.params_dict['Af_rho_inv']
        dx1 = np.sqrt(x2**2)
        dx2 = Q1*Af_rho_inv**(1./3)*x2**(4./3) + (Q2*x2 + Q3*Af_rho_inv**(-2./3)* x2**(1./3))*w
        dx3 = w        
        return [dx1,dx2,dx3]

def main():
    print "test the script with toy example"
    rospy.init_node('pouring_testing')
    scale_or_vision = 'scale'
    final_mh = 100 #ml or grams
    final_mh_time = 10 #seconds
    #set the parameters
    pour_obj = Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time)
    #calculate the min jerk desired pouring curve
    pour_obj.calc_meas_pram_min_jerk_traj('init') #initialize and find the trajectory (with offset time: 0:tf)
    #Given a time (t), angle (th) and measured h, dh; find the input omega
    dt, theta, h, dh = 0., 0., 0.,0.
    w, t_start_new = pour_obj.calc_omega(dt, theta, h, dh)
    y0,t0 = [0,0.01, 0],0
    f = pour_obj.ode_test
    r = ode(f).set_integrator('zvode', method='bdf', with_jacobian=False)
    f_params = [1,9.8,0.3] #args being passed
    r.set_initial_value(y0, t0)
    tf = 10
    dt = 0.01
    t_list = []
    x1_list = []
    x2_list = []
    x3_list = []
    while r.successful() and r.t < tf:
      r.integrate(r.t+dt)
      t_list.append(r.t)
      x1_list.append(np.real(r.y[0]).item(0))
      x2_list.append(np.real(r.y[1]).item(0))
      x3_list.append(np.real(r.y[2]).item(0))
    fig1 = plt.figure(); ax1 = fig1.add_subplot(111)
    ax1.set_xlabel('$t$ (normed)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
    ax1.plot(t_list, x1_list,'-b', label="$m$")
    ax1.plot(t_list, x2_list,'--r', label="$\\dot{m}$")
    ax1.set_xticks(np.arange(0,tf,.5))
    plt.grid()
    ax1.legend(loc=2,fontsize=20)

    fig2 = plt.figure(); ax2 = fig2.add_subplot(111)
    ax1.set_xlabel('$t$ (normed)', fontsize=20); #ax1.set_ylabel('$\\theta (rad)$', fontsize=30)
    ax2.plot(t_list, x3_list,'--g', label="$\\theta$")
    print "len of tlist: ", len(t_list), " len of w", len(pour_obj.w_plot)
    ax2.plot( pour_obj.w_plot_t, pour_obj.w_plot,'-b', label="$\\omega$")
    ax2.set_xticks(np.arange(0,tf,.5))
    ax2.set_yticks(np.arange(-math.pi,math.pi,1))   
    plt.grid()
    ax2.legend(loc=2,fontsize=20)
    plt.show()

if __name__ == '__main__':
    main()



