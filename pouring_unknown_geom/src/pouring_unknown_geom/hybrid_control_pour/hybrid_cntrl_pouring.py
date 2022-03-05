#!/usr/bin/env python
import rospy, math
import numpy as np
import matplotlib.pyplot as plt


class SawyerHybridControlPour(object):
  """docstring for SawyerHybridControlPour"""
  def __init__(self, coef={'coef':[0,0,0,0,0,0,0,0], 'domain':[0 ,np.pi], 'residual':0}, Kp=1.0, delta_omega_speed = 0.1, omega_max = 15.0*np.pi/180, theta_rotate_max= 2*np.pi/180.):
    #SEED VALUE REQUIRED
    self.coef = self.obtain_coef_obj(coef)  #dictionary with keys coef and domain
    # self.Afunct = lambda x: 41.66 This was taken from previous measurement into flask  min omega from before as well
    self.Kp = Kp
    self.delta_speed = delta_omega_speed
    self.omega_max = omega_max #0.2rad/s (10deg/s)

    self.commanded_omega_previously = False

    self.hybrid_states_str = {0:'Initiate Pour', 1:'Pouring Trajectory', 2:'Full Stop', 3:'Full Stop*'}

    self.current_hybrid_state = self.hybrid_states_str[0]


    #Find all the terms for time delay control
    #1. Set constants
    self.T_control_period = 0.1 #sec

    self.pouring_time_delay = 2.0 #was 1.0 sec but observed 3sec in flask
    if rospy.has_param('pour_time_delay'):
      self.pouring_time_delay = float(rospy.get_param("pour_time_delay"))
      
    self.pouring_integrating_steps = np.floor(self.pouring_time_delay/self.T_control_period)
    self.T_control_remainder = self.pouring_time_delay - self.pouring_integrating_steps*self.T_control_period

    self.prediction_dict = {'pred_time':0.0,'pred_height':0.0, 'pred_traj_height':0.0, 'dv_dth_alpha':0.0}


    #2. Obtain parametesr
    #2a. dA/dh_beta is constant and known in Abeta_func
    #2b. dV_alpha/dth (this is the negative of the reported dV_dth_beta)
    #2c. running list of parameters(th_meas, omega_meas, h_meas) for controller history numerical integrator
    self.state_history_list = [] 

    #set terms for finding previous control times
    self.control_time_lookup_prev = None
    self.control_time_lookup_curr = None

    self.h_dot_condition_trigger = False

    self.control_time_list = []

    self.control_avg_list_length = self.pouring_integrating_steps #this initializes


    #For pouring backward, only allow backward motion for confined theta
    self.neg_omega_theta_start = None
    self.neg_omega_theta_curr = None
    self.theta_rotate_max = theta_rotate_max  #allow 2deg of backward rotation to stop a pour

  def control_time_params_eval(self):
    #this script looks at the time it takes to run the control loop and updates the parameters accordingly
    #this is a constant (for now)
    self.pouring_time_delay = 1.0 #sec
    def obtain_average_time(time_list):
      dt_list = []
      for idx in range(len(time_list)-1):
        dt_list.append(time_list[idx+1]-time_list[idx])
      delta_time_list_avg = np.mean(dt_list)
      return delta_time_list_avg

    if len(self.control_time_list) >= self.control_avg_list_length:
      self.T_control_period = obtain_average_time(self.control_time_list) #sec
      self.pouring_integrating_steps = np.floor(self.pouring_time_delay/self.T_control_period)
      self.T_control_remainder = self.pouring_time_delay - self.pouring_integrating_steps*self.T_control_period
      curr_time = rospy.Time.now().to_sec()
      self.control_time_list.append(curr_time)
      self.control_time_list = self.control_time_list[1:]
      self.control_avg_list_length = self.pouring_integrating_steps
    else:
      curr_time = rospy.Time.now().to_sec()
      self.control_time_list.append(curr_time)



  def calculate_h_beta_future_old(self, state_history_list=None):
    #Each component of list has {'t':current_control_time, 'h_meas':h_meas, 'theta':theta_bar, 'omega_meas':omega_meas, 'A_beta':A_beta, 'dV_dth_alpha':dV_dth_alpha}
    h_t = state_history_list[-1]['h_meas'] #the current height h(t)
    #make function for dh/dt
    def calc_dh_dt(dA_dh=None, dV_dth=None, omega_meas = None):
      dh_dt = -(dV_dth*omega_meas)/(dA_dh)
      return dh_dt
    #1. Obtain dh_dt(t+tau-tilde_Tc), first determine if tilde_Tc means picking the last or second from last element
    if self.T_control_remainder < 0.5*self.T_control_period: 
      curr_stat_hist = state_history_list[-1] #this means choose the last element
    else:
      curr_stat_hist = state_history_list[-2]  #this means choose second to last element
    dh_dt_t_plus_tau_minus_tilde_Tc = calc_dh_dt(dA_dh=curr_stat_hist['A_beta'],dV_dth=curr_stat_hist['dV_dth_alpha'],omega_meas=curr_stat_hist['omega_meas']) 
    #2. Obtain dh_dt(t+i*Tc) storing dh_dt for each i'th element
    dh_dt_future_from_past = []
    omega_list = []
    for idx in range(int(self.pouring_integrating_steps)+1):
      curr_stat_hist = state_history_list[-(idx+1)]  #for n=10 there are 11 elements in state_history_list with the last being current time t, hence n steps back in time is tau (plus tilde_Tc)
      dh_dt_past_elem = calc_dh_dt(dA_dh=curr_stat_hist['A_beta'],dV_dth=curr_stat_hist['dV_dth_alpha'],omega_meas=curr_stat_hist['omega_meas']) 
      dh_dt_future_from_past.append(dh_dt_past_elem) #make a list of dh_dt
      omega_list.append(curr_stat_hist['omega_meas'])
    #3. Now sum these together to get the expected h_beta(t + tau)
    # h_t = state_history_list[-1]['h_meas'] #this is the latest h(t)
    # h_t_plus_tau_minus_tilde_Tc = h_t + np.sum([self.T_control_period*dh_dt_future_from_past[idx] for idx in range(len(dh_dt_future_from_past))]) #this is h(t) + sum_i^n Tc*dh/dt(t+i*Tc)
    # h_beta_t_plus_tau =  h_t_plus_tau_minus_tilde_Tc + self.T_control_remainder*dh_dt_t_plus_tau_minus_tilde_Tc #this is h(t) +  sum_i^n Tc*dh/dt(t+i*Tc) + tilde{T}_c * dh/dt_{t+tau-tilde{Tc}}
    h_t_tau_Tc = np.sum([self.T_control_period*dh_dt for dh_dt in dh_dt_future_from_past])
    dh_tau_Tc = self.T_control_remainder*dh_dt_t_plus_tau_minus_tilde_Tc
    rospy.loginfo("h(t): %f, h(t+tau-Tc): %f, dh(Tc): %f, Ttilde: %f, dh/dt(t+tau+Tc_tilde): %f"%(h_t, h_t_tau_Tc, dh_tau_Tc, self.T_control_remainder, dh_dt_t_plus_tau_minus_tilde_Tc))
    # print "dh_dt list: ", dh_dt_future_from_past
    # print "average control period: ", self.T_control_period
    # print "omega list: ", omega_list
    h_beta_t_plus_tau = state_history_list[-1]['h_meas']
    return h_beta_t_plus_tau


  def calculate_h_beta_future(self, state_history_list=None, Abeta_func=None):


    #1. Find the number of elements tau time back

    t_curr = state_history_list[-1]['t_oper']
    # rospy.loginfo('current time: %f'%(t_curr))
    # print " t_past list:", [state_history_list[idx]['t_oper']-t_curr for idx in range(len(state_history_list))]
    dt = 0.0
    idx = 1
    while (dt < self.pouring_time_delay) and (idx+1 <= len(state_history_list)):
      dt +=  state_history_list[-idx]['t_oper'] - state_history_list[-(idx+1)]['t_oper']
      idx += 1; # print "idx:", idx
    state_history_list = state_history_list[-idx:] #truncate those before dt > tau

    #DEBUG 1: dt is consistently >= tau [check]
    # print " dt >= tau: ", dt>self.pouring_time_delay, " dt: ", dt, " tau: ", self.pouring_time_delay #debug
    


    #2. Find future times
    t_future_list = [state_history_list[idx]['t_oper']+self.pouring_time_delay for idx in range(len(state_history_list))] #[t1,t2,t3]+tau = [t1',t2',t3'] where t1,t2,t3 <= t_curr and ti'>t_curr

    #DEBUG 2: time shifting verified [check]
    # rospy.loginfo('current time: %f'%(state_history_list[-1]['t_oper']-t_curr))
    # print " t_past list:", [state_history_list[idx]['t_oper']-t_curr for idx in range(len(state_history_list))]
    # print " t_future list: ", [t-t_curr for t in t_future_list]


    #3. Perform integration to obtain the estimated height based on actions to this point (with good model approx this allows us to estimate the delayed observation)
    h_running = state_history_list[-1]['h_meas']  #this is h(t)
    
    #DEBUG 3: check output of history of heights [check]
    # print "current h(t): ", h_running
    # print "past h(t-iTc) list: ", [state_history_list[idx]['h_meas'] for idx in range(len(state_history_list))]
    # print "times: ", [state_history_list[idx]['t_oper'] - t_curr for idx in range(len(state_history_list))]


    for idx in range(len(state_history_list)-1):
      A_beta = Abeta_func(h_running) #this updates for each dt: h(t) -> h(t) + Tc_1*dh_dt(t) -> h(t) + Tc_1*dh_dt(t) + Tc_2*dh_dt(t+Tc_1)
      dV_dth_alpha = state_history_list[idx]['dV_dth_alpha']
      if dV_dth_alpha > 0.0:
        rospy.loginfo(" dV/dth_alpha is positive: %f, setting to 0.0"%(dV_dth_alpha))
        dV_dth_alpha = 0.0  #set to zero to not use this (worse case will make height the observed height and time delay will not be accounted for.)
      dh_dt = -(A_beta)**(-1) * dV_dth_alpha * np.max([state_history_list[idx]['omega_meas'], 0.0])
      
      omega = state_history_list[idx]['omega_meas']
      dt = state_history_list[idx+1]['t_oper'] -  state_history_list[idx]['t_oper'] 
      h_running += dt*dh_dt # h(t) + dT*dh_dt(t)
      t = state_history_list[idx]['t_oper'] - t_curr 
      #DEBUG 4: output incremental h_add up
      # rospy.loginfo('t: %f, dt: %f, dh_dt: %f, h_run: %f, w: %f, Ab: %f, dVdth: %f'%(t,dt,dh_dt,h_running, omega, A_beta, dV_dth_alpha))

    #now if there was a Tc_tilde
    dt = state_history_list[-1]['t_oper'] -  state_history_list[-2]['t_oper'] 
    if self.T_control_remainder < 0.5*dt:
      dh_dt += -(Abeta_func(h_running))**(-1)* state_history_list[-1]['dV_dth_alpha']*state_history_list[-1]['omega_meas']
    else:
      dh_dt += -(Abeta_func(h_running))**(-1)* state_history_list[-2]['dV_dth_alpha']*state_history_list[-2]['omega_meas']
    h_running += self.T_control_remainder*dh_dt

    h_beta_t_plus_tau = h_running
    return h_beta_t_plus_tau


  def obtain_coef_obj(self,coef_dict):
    a = coef_dict['coef']
    domain = coef_dict['domain']
    sk = np.polynomial.legendre.Legendre(a, domain=domain, window=domain).deriv(1) #because the model for V is passed, but need to use dV
    return sk

  def condition_theta(self,theta_bar):
    # if theta_bar <= np.pi and theta_bar >= 0:
    #print "inside cond theta: ", theta_bar, "limit lower: ",(self.theta_rotate_max+math.radians(3.0)) 
    if theta_bar <= np.pi/2. and theta_bar >= 0-(self.theta_rotate_max+math.radians(3.0)):
      return True
    else:
      return False

  def condition_dV_dth(self,dV_dth_beta):
    if dV_dth_beta > 0.0:
      return True
    else:
      return False

  def condition_dh_dt(self,h_dot):
    if h_dot > 0.0:
      return True
    else:
      return False

  def calc_dv_dth_alpha(self,theta_bar):
    dV_dth_beta = np.polynomial.legendre.legval(theta_bar, self.coef.coef)
    return -dV_dth_beta


  def Controller_output(self, H_final=100, h_des=0, h_des_dot=0, h_des_t_plus_tau = 0, h_meas=0, h_meas_dot=0, theta_bar=0.0, omega_meas=0.0, Abeta_func= lambda x: 1.0 , coef=None , pour_sign=1, current_control_time=None):
    if coef:
      self.coef = self.obtain_coef_obj(coef) #if new were passed in then update model parameters  #this is a dictionary

    '''since h is in ml dont need area function 41.66 '''


    if not current_control_time:
      current_control_time = rospy.Time.now().to_sec() #get current time if not passed
    operational_time = rospy.Time.now().to_sec()

    #This function re-evaulates the control loop time to ensure the correct T_c is used for look-ahead
    self.control_time_params_eval()

    # print "\n\nCurrent T_c: ", self.T_control_period, " and integrating steps N: ", self.pouring_integrating_steps

    #calculate_terms
    A_beta = Abeta_func(h_meas)
    dV_dth_beta = np.polynomial.legendre.legval(theta_bar, self.coef.coef)
    dV_dth_alpha = self.calc_dv_dth_alpha(theta_bar)
    # dV_dth_alpha_real = dV_dth_alpha 
    #debug: 
    # dV_dth_beta = 500.0 #500 imperically for foggy glass #200.0/(90.0*math.pi/180.) #63.66 (set constant rate to see if pouring is then consistent across containers)
    # dV_dth_alpha = - dV_dth_beta


    #for time delay control
    omega_meas_history = np.max([omega_meas,0.0])
    state_history = {'t':current_control_time,'t_oper':operational_time, 'h_meas':h_meas, 'theta':theta_bar, 'omega_meas':omega_meas_history, 'A_beta':A_beta, 'dV_dth_alpha':dV_dth_alpha}
    self.state_history_list.append(state_history)
    if len(self.state_history_list) > self.pouring_integrating_steps+2:
      #greater than means length is n+2 as required
      self.state_history_list = self.state_history_list[1:] #pop off the first



    #Cacluate desired input
    # tau = h_des_dot + self.Kp*(h_des - h_meas)
    tau = self.Kp*(h_des - h_meas)
    ml_error = (h_des - h_meas)

    #Check conditions
    cond_theta = self.condition_theta(theta_bar)
    cond_dV = self.condition_dV_dth(dV_dth_beta)
    cond_h = self.condition_dh_dt(h_meas_dot)


    #term to recalculate traj if neccessary: 
    new_traj_bool = False

    # print "conditions: th:",cond_theta, " dv:", cond_dV, " h:", cond_h, "error", float(h_des)-float(h_meas), "h_des:", h_des, " h_meas", h_meas
    # print "\n Theta is: ", theta_bar

    #Calculate inputs
    u = 0.0


    if len(self.state_history_list) > self.pouring_integrating_steps:
      h_beta_t_plus_tau = self.calculate_h_beta_future(self.state_history_list, Abeta_func=Abeta_func) #find h_{beta}(t+tau) (expected volume time delay in the future)
      #set prediction terms to publish
      self.prediction_dict['pred_time'] = current_control_time + self.pouring_time_delay
      self.prediction_dict['pred_height'] = h_beta_t_plus_tau
      self.prediction_dict['pred_traj_height'] = h_des_t_plus_tau
      self.prediction_dict['dv_dth_alpha'] = dV_dth_alpha

    if self.h_dot_condition_trigger:
      cond_h = True

    if cond_theta == True and cond_dV == True and cond_h == True:
      self.h_dot_condition_trigger = True  #this causes cond_h to only be false until the onset of pouring
      #Choose whether to use time delay control or not based on the amount of history available
      if len(self.state_history_list) > self.pouring_integrating_steps:
        # rospy.loginfo("Controlling the Future!!!!")
        #implement pouring control with history
        h_beta_t_plus_tau = self.calculate_h_beta_future(self.state_history_list, Abeta_func=Abeta_func) #find h_{beta}(t+tau) (expected volume time delay in the future)
        dA_dh_t_plust_tau = Abeta_func(h_beta_t_plus_tau) #evaluate the cross sectional area of the receiver at the future time
        dV_dth_alpha_curr = dV_dth_alpha
        tau = self.Kp*(h_des_t_plus_tau - h_beta_t_plus_tau)
        ml_error = (h_des_t_plus_tau - h_beta_t_plus_tau)
        u = - dA_dh_t_plust_tau * (dV_dth_alpha_curr)**(-1)*tau  #input w(t) for future objective
      else:
        u = (dV_dth_beta)**(-1) * tau

      self.current_hybrid_state = self.hybrid_states_str[1]
    elif ((cond_theta == True and cond_dV == False) or (cond_h == False and cond_theta == True)) and (float(h_des)-float(h_meas))>0.0 :
      #u = np.sign(theta_bar)*self.delta_speed
      u = self.delta_speed
      new_traj_bool = True
      self.current_hybrid_state = self.hybrid_states_str[0]
      #small increment in angle direction

    elif cond_theta == False:
      self.current_hybrid_state = self.hybrid_states_str[2]
      # print "out of range 0 pour"
      u = 0.0
      new_traj_bool = True

    else:
      # print "non-catch"
      self.current_hybrid_state = self.hybrid_states_str[3]
      u = 0.0
      new_traj_bool = False




    if not self.commanded_omega_previously:
      new_traj_bool = True
      #u = np.sign(theta_bar)*self.delta_speed
      u = self.delta_speed
      # print "commanding vel from no command before: ", u, " because omega prev is ", self.commanded_omega_previously

    if u > 0.0:
      self.commanded_omega_previously = True
      # print "command prev omega set to true", self.commanded_omega_previously
      self.neg_omega_theta_start = None
      self.neg_omega_theta_curr = None


    if u <= -(10.0*np.pi/180):
      if self.neg_omega_theta_start:
        self.neg_omega_theta_curr = theta_bar
        if np.abs(self.neg_omega_theta_start - self.neg_omega_theta_curr) <= self.theta_rotate_max:
          u = -(10.0*np.pi/180)
        else:
          u = 0.0
      else:
        self.neg_omega_theta_start = theta_bar
        self.neg_omega_theta_curr = theta_bar
        u = -(1.0*np.pi/180)





      '''NEED TO SET ANGLE LIMIT '''

    # if u <= 0.0:
    #   #strictly drive forward, wait for the trajectory to catch up
    #   u = 0.0

    if h_meas >= (H_final+10.0):
      #dont pour past goal
      # print "height is above final: h_meas:", h_meas, " Hfinal: ", H_final, " hence commanding omega", pour_sign*u
      u = 0.0
      # print "new u:", u
      new_traj_bool = False


    #Make input the power of the sign
    omega = pour_sign*u


    if np.abs(omega) > self.omega_max:
      omega = np.sign(omega)*self.omega_max


    # print "returning the following omega: ", omega

    #Return the input
    return omega, new_traj_bool, self.current_hybrid_state, ml_error, self.prediction_dict



def main():
  #make example
  num_pts = 100
  t = np.linspace(0,1,num_pts)
  h_des = 100*np.power(t,2)
  h_des_dot = 0*h_des
  theta = np.linspace(0,np.pi,num_pts)

  h_act = h_des - 5.0
  h_act_dot = h_des_dot

  s = np.polynomial.legendre.legfit(t,h_des,2)
  sk = np.polynomial.legendre.Legendre(s,domain=[0,1], window=[0,1])

  coef_obj = {'coef':sk.coef, 'domain':sk.domain}

  cls_obj = SawyerHybridControlPour(coef=coef_obj)

  w_list = []
  for idx in range(len(h_des)):
    w, boolval = cls_obj.Controller_output(h_des=h_des[idx], h_des_dot=h_des_dot[idx], h_meas=h_act[idx], h_meas_dot=h_act_dot[idx], theta_bar=theta[idx])
    w_list.append(w)


  fig = plt.figure()
  ax = fig.gca()
  print len(w_list), len(t)
  ax.plot(t,w_list)
  err = h_des - h_act
  ax.plot(t,err)
  plt.show()

  """SET MAX OMEGA """


if __name__ == '__main__':
  main()
