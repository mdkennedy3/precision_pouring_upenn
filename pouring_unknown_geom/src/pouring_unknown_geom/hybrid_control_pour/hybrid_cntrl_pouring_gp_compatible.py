#!/usr/bin/env python
import rospy, math
import numpy as np
import copy
import matplotlib.pyplot as plt
from pouring_msgs.msg import PourModelPred, GPVolumeModel
from pouring_unknown_geom.gaussian_process import gp_process_calc

class GPModelMaintainer(object):
  """docstring for GPModelMaintainer"""
  def __init__(self):
    self.GP_models =[]
    self.GP_models_probability = []

class SawyerHybridControlPour(object):
  """docstring for SawyerHybridControlPour"""
  def __init__(self, pour_model_obj=PourModelPred(), Kp=1.0, delta_omega_speed = 0.1, omega_max = 15.0*np.pi/180, theta_rotate_max= 2*np.pi/180., diff_ang_resolution=0.2*np.pi/180.):
    #SEED VALUE REQUIRED
    # self.pour_model_obj = self.obtain_coef_obj(coef)  #dictionary with keys coef and domain
    self.pour_model_obj = pour_model_obj
    self.pour_model_obj.volume_solution_mode = "param"
    self.pour_model_obj.type = "power"
    self.pour_model_obj.domain = [0,2.4]
    self.pour_model_obj.coef_opt = np.zeros([10]).tolist()
    self.pour_model_obj.volume_solution_mode = "param" #options are "param"; "semiparam"; "nonparam"
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
    #2c. running list of parameters(tvol_meas, omega_meas, vol_meas) for controller history numerical integrator
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
    #For numerically differentiating volume profile
    self.diff_ang_resolution =diff_ang_resolution
    #For using Gaussian Process:
    self.gp_process_model_list = GPModelMaintainer()# gp_process_calc.GuassianProcessClass() #instatiate for each GP to train
    #for cond_h mode
    self.cond_h_time_thresh = 1.5 #sec
    self.cond_h_time_t_start = None
    self.shift_gp_volumes_bool = True

  def train_GP_models(self,pour_model_obj=None, N_gp_pts=50, volume_solution_mode="nonparam"):
    lowest_model_rmse = pour_model_obj.lowest_model_rmse
    self.gp_process_model_list = GPModelMaintainer()
    self.shift_gp_volumes_bool = True
    for idx in range(len(pour_model_obj.gp_volume_models)):
      curr_model = pour_model_obj.gp_volume_models[idx]
      #gp_model = gp_process_calc.GuassianProcessClass(x_train=curr_model.th, y_train=curr_model.V, N_max=N_gp_pts, lowest_model_rmse=lowest_model_rmse)
      gp_model = gp_process_calc.GuassianProcessClass(x_train=curr_model.th, y_train=curr_model.V, N_max=N_gp_pts, lowest_model_rmse=curr_model.rmse_value, param_method=volume_solution_mode)
      self.gp_process_model_list.GP_models.append(copy.copy(gp_model))
      self.gp_process_model_list.GP_models_probability.append(curr_model.perc_mix_model)



  def obtain_coef_obj(self,coef_dict):
    a = coef_dict['coef']
    domain = coef_dict['domain']
    sk = np.polynomial.legendre.Legendre(a, domain=domain, window=domain).deriv(1) #because the model for V is passed, but need to use dV
    return sk

  def control_time_params_eval(self):
    #this script looks at the time it takes to run the control loop and updates the parameters accordingly
    #this is a constant (for now)
    # self.pouring_time_delay = 1.0 #sec
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

  def calculate_h_beta_future(self, state_history_list=None, Abeta_func=None):
    #1. Find the number of elements tau time back
    t_curr = state_history_list[-1]['t_oper']
    dt = 0.0
    idx = 1
    while (dt < self.pouring_time_delay) and (idx+1 <= len(state_history_list)):
      dt += copy.copy(state_history_list[-idx]['t_oper']) - copy.copy(state_history_list[-(idx+1)]['t_oper'])
      idx += 1; # print "idx:", idx
    if idx == 1: idx =2
    state_history_list = state_history_list[-idx:] #truncate those before dt > tau
    #2. Find future times
    t_future_list = [state_history_list[idx]['t_oper']+self.pouring_time_delay for idx in range(len(state_history_list))] #[t1,t2,t3]+tau = [t1',t2',t3'] where t1,t2,t3 <= t_curr and ti'>t_curr
    #3. Perform integration to obtain the estimated height based on actions to this point (with good model approx this allows us to estimate the delayed observation)
    vol_running = state_history_list[-1]['vol_meas']  #this is h(t)
    for idx in range(len(state_history_list)-1):
      # A_beta = Abeta_func(vol_running) #this updates for each dt: h(t) -> h(t) + Tc_1*dh_dt(t) -> h(t) + Tc_1*dh_dt(t) + Tc_2*dh_dt(t+Tc_1)
      dV_dth_alpha = state_history_list[idx]['dV_dth_alpha']
      if dV_dth_alpha > 0.0:
        rospy.loginfo(" dV/dth_alpha is positive: %f, setting to 0.0"%(dV_dth_alpha))
        dV_dth_alpha = 0.0  #set to zero to not use this (worse case will make height the observed height and time delay will not be accounted for.)
      dvol_dt = -dV_dth_alpha * np.max([state_history_list[idx]['omega_meas'], 0.0])
      omega = state_history_list[idx]['omega_meas']
      dt = state_history_list[idx+1]['t_oper'] -  state_history_list[idx]['t_oper']
      vol_running += dt*dvol_dt # h(t) + dT*dvol_dt(t)
      t = state_history_list[idx]['t_oper'] - t_curr
    #now if there was a Tc_tilde
    dt = state_history_list[-1]['t_oper'] -  state_history_list[-2]['t_oper']
    if self.T_control_remainder < 0.5*dt:
      dvol_dt += -state_history_list[-1]['dV_dth_alpha']*state_history_list[-1]['omega_meas']
    else:
      dvol_dt += -state_history_list[-2]['dV_dth_alpha']*state_history_list[-2]['omega_meas']
    vol_running += self.T_control_remainder*dvol_dt
    V_beta_t_plus_tau = vol_running
    return V_beta_t_plus_tau

  def condition_theta(self,theta_bar):
    if theta_bar <= 2.*np.pi/3. and theta_bar >= 0-(self.theta_rotate_max+math.radians(3.0)):
      return True
    else:
      return False

  def condition_dV_dth(self,dV_dth_beta):
    if dV_dth_beta > 0.0:
      return True
    else:
      return False

  def condition_dh_dt(self,h_dot, vol_curr = 10):
    if vol_curr < 10:
      if vol_curr < 3: return [False,False]
      if h_dot > 0.0:
        if type(self.cond_h_time_t_start) == type(None):
          self.cond_h_time_t_start = rospy.Time.now().to_sec()
          return [False,False]
        else:
          t_curr = rospy.Time.now().to_sec()
          if (t_curr - self.cond_h_time_t_start) >= self.cond_h_time_thresh:
            return [True,True]
          else:
            return [False,False]
      else:
        self.cond_h_time_t_start = None
        return [False,False]
    else:
      if h_dot > 0.0:
        return [True,True]
      else:
        return [False,True]


  def calc_dv_dth_alpha(self,theta_bar):
    if self.pour_model_obj.type in 'legendre':
      dV_dth_beta = np.polynomial.legendre.legval(theta_bar, self.pour_model_obj.coef_opt)
    elif self.pour_model_obj.type in 'power':
      dV_dth_beta = np.polyval(np.polyder(self.pour_model_obj.coef_opt),theta_bar)
    return -dV_dth_beta


  def truncate_list(self,x_list=None, N_max=50):
      x_list = [x_list[idx] for idx in range(len(x_list)) if np.remainder(idx,np.floor(len(x_list)/N_max)) == 0.0]
      return x_list

  def volume_model_evaluation(self,theta_bar=None):
    #This function evaluates with the current method, the V(th) output
    if self.pour_model_obj.volume_solution_mode == "param":
      if self.pour_model_obj.type in 'power':
        volume_eval = np.polyval(self.pour_model_obj.coef_opt,theta_bar)
      else:
        rospy.logwarn("implement the legendre version or switch to previous code")
        volume_eval = 0.0
    elif self.pour_model_obj.volume_solution_mode == "semiparam":
      #combine coef and GP
      volume_eval = 0.0
      v_poly = np.polyval(self.pour_model_obj.coef_opt,theta_bar)
      vol_gp = 0
      for idx in range(len(self.gp_process_model_list.GP_models)):
        if (len(self.pour_model_obj.gp_volume_models) == len(self.gp_process_model_list.GP_models)) and self.shift_gp_volumes_bool:
          self.shift_gp_volumes_bool = False
          #however after the onset of pouring, all of the volumes were shifted down to the same theta intersect, so y_train must be found again
          curr_model = self.pour_model_obj.gp_volume_models[idx]
          y_train = curr_model.V
          N_max = self.gp_process_model_list.GP_models[idx].N_max
          y_train = [y_train[mdx] for mdx in range(len(y_train)) if np.remainder(mdx,np.floor(len(y_train)/N_max)) == 0.0]
          self.gp_process_model_list.GP_models[idx].y_train = y_train
          self.gp_process_model_list.GP_models[idx].y_train_vect = np.matrix(y_train).T
        
        #th_trunc = self.truncate_list(x_list=curr_model.th, N_max=self.gp_process_model_list.GP_models[idx].N_max) #truncate first to increase speed 
        th_trunc = self.gp_process_model_list.GP_models[idx].x_train #subsampled points
        #fit these points with model
        est_volume_train = np.polyval(self.pour_model_obj.coef_opt, th_trunc)
        if type(est_volume_train) == np.ndarray: est_volume_train = est_volume_train.tolist()
        gp_semi_volume_eval = self.gp_process_model_list.GP_models[idx].prediction(x_test = [theta_bar], y_mean_train =est_volume_train)
        #volume_eval +=  self.gp_process_model_list.GP_models_probability[idx]*gp_semi_volume_eval[0]
        vol_gp +=  self.gp_process_model_list.GP_models_probability[idx]*gp_semi_volume_eval[0]
      volume_eval = v_poly + vol_gp
      #rospy.loginfo("SEMIPARAM: v_poly %f, v gp: %f, summed v: %f"%(v_poly,vol_gp, volume_eval))  
    elif self.pour_model_obj.volume_solution_mode == "nonparam":
      #just use GP
      volume_eval = 0.0
      for idx in range(len(self.gp_process_model_list.GP_models)):
        if (len(self.pour_model_obj.gp_volume_models) == len(self.gp_process_model_list.GP_models)) and self.shift_gp_volumes_bool:
          self.shift_gp_volumes_bool = False
          curr_model = self.pour_model_obj.gp_volume_models[idx]
          y_train = curr_model.V
          N_max = self.gp_process_model_list.GP_models[idx].N_max
          y_train = [y_train[mdx] for mdx in range(len(y_train)) if np.remainder(mdx,np.floor(len(y_train)/N_max)) == 0.0]
          self.gp_process_model_list.GP_models[idx].y_train = y_train
          self.gp_process_model_list.GP_models[idx].y_train_vect = np.matrix(y_train).T
        #shift the points
        gp_semi_volume_eval = self.gp_process_model_list.GP_models[idx].prediction(x_test = [theta_bar])
        volume_eval +=  self.gp_process_model_list.GP_models_probability[idx]*gp_semi_volume_eval[0]

    return volume_eval

  def numerical_differentiate_volume_model(self,theta_bar=None):
    #self.diff_ang_resolution #ang resolution for differentiation
    #self.pour_model_obj #pour model (contains GP's and coeffs)
    theta_lower = theta_bar - self.diff_ang_resolution
    theta_upper =theta_bar + self.diff_ang_resolution
    Vlower = self.volume_model_evaluation(theta_lower)
    Vcurr = self.volume_model_evaluation(theta_bar)
    Vupper = self.volume_model_evaluation(theta_upper)
    #Get average derivative
    dV_beta_lower = (Vcurr - Vlower)/self.diff_ang_resolution
    dV_beta_upper = (Vupper-Vcurr)/self.diff_ang_resolution
    dV_beta_avg = 0.5*( dV_beta_lower+ dV_beta_upper )
    #populate and return
    dV_dth_beta = dV_beta_avg
    dV_dth_alpha = -dV_beta_avg
    return dV_dth_beta, dV_dth_alpha


  def train_gp_models_callback(self,gp_selected_models=None, volume_solution_mode="semiparam"):
    #This function is the callback to train the models on initial run
    if type(gp_selected_models) != type(None):
      if len(gp_selected_models.gp_volume_models) > 0:
        self.train_GP_models(pour_model_obj=gp_selected_models, volume_solution_mode=volume_solution_mode) #these are available immediately
        rospy.loginfo("GP models trained in main hyb cntrl")


  def Controller_output(self, Vol_final=100, vol_des=0, vol_des_dot=0, vol_des_t_plus_tau = 0, vol_meas=0, vol_meas_dot=0, theta_bar=0.0, omega_meas=0.0, Abeta_func= lambda x: 1.0 , pour_model_obj=None , pour_sign=1, current_control_time=None,time_delay=2.0, train_gp_models_bool=False, gp_selected_models=None):

    #train gp models if desired
    #print "train gp models bool: ", train_gp_models_bool
    if train_gp_models_bool:
      if type(gp_selected_models) != type(None):
        if len(gp_selected_models.gp_volume_models) > 0:
          self.train_GP_models(pour_model_obj=gp_selected_models) #these are available immediately
          rospy.loginfo("GP models trained in main hyb cntrl")
    #update time delay params
    #self.pouring_time_delay = np.max([time_delay,1.0])+0.3 #set the time delay
    self.pouring_time_delay = np.max([time_delay,1.0]) + 0.18 #added part is falltime #set the time delay
    #update pouring models
    if pour_model_obj:
      # print "Updating: ", pour_model_obj
      self.pour_model_obj = pour_model_obj #if new were passed in then update model parameters  #this is a dictionary
    if not current_control_time:
      current_control_time = rospy.Time.now().to_sec() #get current time if not passed
    operational_time = rospy.Time.now().to_sec()
    #This function re-evaulates the control loop time to ensure the correct T_c is used for look-ahead
    self.control_time_params_eval()
    #calculate_terms
    '''
    if self.pour_model_obj.volume_solution_mode == "param":
      if self.pour_model_obj.type in 'legendre':
        dV_dth_beta = np.polynomial.legendre.legval(theta_bar, self.pour_model_obj.coef_opt) #leg fit
      elif self.pour_model_obj.type in 'power':
        dV_dth_beta = np.polyval(np.polyder(self.pour_model_obj.coef_opt),theta_bar)  #differentiate as I return V
      dV_dth_alpha = self.calc_dv_dth_alpha(theta_bar)
    elif self.pour_model_obj.volume_solution_mode in ["semiparam","nonparam"]:
      #Use numerical differentiation in this case
    '''
    if self.pour_model_obj.volume_solution_mode in ["semiparam","param","nonparam"]:
      dV_dth_beta,dV_dth_alpha = self.numerical_differentiate_volume_model(theta_bar=theta_bar)
      #print "resultant dV/dth", dV_dth_alpha
    else:
      #print "Current: ", self.pour_model_obj.volume_solution_mode, "."
      rospy.logwarn("incorrect solution mode specified in PourModelPred msg {}.".format(self.pour_model_obj.volume_solution_mode) )


    #for time delay control
    omega_meas_history = np.max([omega_meas,0.0])
    state_history = {'t':current_control_time,'t_oper':operational_time, 'vol_meas':vol_meas, 'theta':theta_bar, 'omega_meas':omega_meas_history, 'dV_dth_alpha':dV_dth_alpha}
    self.state_history_list.append(state_history)
    if len(self.state_history_list) > self.pouring_integrating_steps+2:
      #greater than means length is n+2 as required
      self.state_history_list = self.state_history_list[1:] #pop off the first
    #Cacluate desired input
    tau = self.Kp*(vol_des - vol_meas)
    ml_error = (vol_des - vol_meas)
    #Check conditions
    cond_theta = self.condition_theta(theta_bar)
    cond_dV = self.condition_dV_dth(dV_dth_beta)
    [cond_h, set_cond_h_trigger] = self.condition_dh_dt(vol_meas_dot, vol_curr=vol_meas)
    #term to recalculate traj if neccessary:
    new_traj_bool = False
    #Calculate inputs
    u = 0.0
    if len(self.state_history_list) > self.pouring_integrating_steps:
      V_beta_t_plus_tau = self.calculate_h_beta_future(self.state_history_list) #find h_{beta}(t+tau) (expected volume time delay in the future)
      #set prediction terms to publish
      self.prediction_dict['pred_time'] = current_control_time + self.pouring_time_delay
      self.prediction_dict['pred_height'] = V_beta_t_plus_tau
      self.prediction_dict['pred_traj_height'] = vol_des_t_plus_tau
      self.prediction_dict['dv_dth_alpha'] = dV_dth_alpha
    #Bring this back if its truly neccessary
    if self.h_dot_condition_trigger and set_cond_h_trigger:
      cond_h = True
    if cond_theta == True and cond_dV == True and cond_h == True:
      self.h_dot_condition_trigger = True  #this causes cond_h to only be false until the onset of pouring
      #Choose whether to use time delay control or not based on the amount of history available
      if len(self.state_history_list) > self.pouring_integrating_steps:
        #implement pouring control with history
        V_beta_t_plus_tau = self.calculate_h_beta_future(self.state_history_list) #find h_{beta}(t+tau) (expected volume time delay in the future)
        # dA_dh_t_plust_tau = Abeta_func(V_beta_t_plus_tau) #evaluate the cross sectional area of the receiver at the future time
        dV_dth_alpha_curr = dV_dth_alpha
        if (vol_des_t_plus_tau - V_beta_t_plus_tau) >= 0.0:
          tau = self.Kp*(vol_des_t_plus_tau - V_beta_t_plus_tau)
        else:
          tau = 5*self.Kp*(vol_des_t_plus_tau - V_beta_t_plus_tau)

        ml_error = (vol_des_t_plus_tau - V_beta_t_plus_tau)
        u = -(dV_dth_alpha_curr)**(-1)*tau  #input w(t) for future objective
        #if tau < 0.0: rospy.logwarn("tau is negative"); print tau        
      else:
        u = (dV_dth_beta)**(-1) * tau
      self.current_hybrid_state = self.hybrid_states_str[1]
    elif ((cond_theta == True and cond_dV == False) or (cond_h == False and cond_theta == True)) and (float(vol_des)-float(vol_meas))>0.0 :
      #u = np.sign(theta_bar)*self.delta_speed
      #rospy.logwarn("in negated state, cond V false or cond h false")
      u = self.delta_speed
      new_traj_bool = True
      self.current_hybrid_state = self.hybrid_states_str[0]
      #small increment in angle direction
    elif cond_theta == False:
      rospy.logwarn("cond th false")
      self.current_hybrid_state = self.hybrid_states_str[2]
      u = 0.0
      new_traj_bool = True
    else:
      self.current_hybrid_state = self.hybrid_states_str[3]
      u = 0.0
      new_traj_bool = False
    if not self.commanded_omega_previously:
      rospy.logwarn("cmd omega prev is false")
      new_traj_bool = True
      u = self.delta_speed
    if u > 0.0:
      self.commanded_omega_previously = True
      self.neg_omega_theta_start = None
      self.neg_omega_theta_curr = None
    #if u <= -(10.0*np.pi/180):
    if u < 0.0:
      if self.neg_omega_theta_start:
        self.neg_omega_theta_curr = theta_bar
        if np.abs(self.neg_omega_theta_start - self.neg_omega_theta_curr) <= self.theta_rotate_max:
          #u = -(10.0*np.pi/180)
          u = np.max([-(15.0*np.pi/180), u])
          if vol_meas >= Vol_final: u = -10.0*np.pi/180.
        else:
          u = 0.0
      else:
        self.neg_omega_theta_start = theta_bar
        self.neg_omega_theta_curr = theta_bar
        u = np.max([-(15.0*np.pi/180),u])
        if vol_meas >= Vol_final: u = -10.0*np.pi/180.
    if vol_meas >= (Vol_final+10.0):
      #dont pour past goal
      u = 0.0
      new_traj_bool = False
    #Make input the power of the sign
    omega = pour_sign*u
    if np.abs(omega) > self.omega_max:
      omega = np.sign(omega)*self.omega_max
    #Return the input
    return omega, new_traj_bool, self.current_hybrid_state, ml_error, self.prediction_dict



def main():
  #make example
  rospy.init_node('test_hyb')
  num_pts = 100
  t = np.linspace(0,1,num_pts)
  vol_final = 100
  vol_des = vol_final*np.power(t,2)
  vol_des_dot = 0*vol_des
  theta = np.linspace(0,np.pi,num_pts)
  #actual
  h_act = vol_des - 5.0
  h_act_dot = vol_des_dot
  #fit poly
  coef = np.polyfit(t,vol_des,9)

  cls_obj = SawyerHybridControlPour()

  cntrl_var_model_struct = PourModelPred()
  cntrl_var_model_struct.coef_opt = coef.tolist()
  cntrl_var_model_struct.domain = [0,2.4]
  cntrl_var_model_struct.type="power"
  gp_mod_1 = GPVolumeModel()
  gp_mod_2 = GPVolumeModel()
  gp_mod_1.th = t; gp_mod_1.V = vol_des; gp_mod_1.perc_mix_model = 0.5
  gp_mod_2.th = t; gp_mod_2.V = vol_des; gp_mod_2.perc_mix_model = 0.5
  cntrl_var_model_struct.gp_volume_models.append(gp_mod_1)
  cntrl_var_model_struct.gp_volume_models.append(gp_mod_2)
  cntrl_var_model_struct.volume_solution_mode = 'nonparam'


  #gen data
  w_list = []
  for idx in range(len(vol_des)):
    w, new_traj_bool, curr_hybrid_state_str, ml_error, prediction_dict = cls_obj.Controller_output(Vol_final=vol_final, vol_des_t_plus_tau = vol_des_dot[idx],
                                                                              vol_des=vol_des[idx], vol_des_dot=vol_des_dot[idx],vol_meas=h_act[idx], vol_meas_dot=h_act_dot[idx],
                                                                              theta_bar=theta[idx], pour_model_obj=cntrl_var_model_struct)

    w_list.append(w)
  #plot
  fig = plt.figure()
  ax = fig.gca()
  print len(w_list), len(t)
  ax.plot(t,w_list,'r')
  err = vol_des - h_act
  ax.plot(t,err,'b')
  plt.show()

if __name__ == '__main__':
  main()
