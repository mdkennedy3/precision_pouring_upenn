#!/usr/bin/env python2
import numpy as np, math
import rospy, rospkg
import sys, getopt
import threading
import actionlib
import copy

# from pouring_unknown_geom.sawyer_control import sawyer_cntrl_cls
# from pouring_control_pkg.sawyer_control import sawyer_cntrl_cls
# from pouring_unknown_geom.hybrid_control_pour import hybrid_cntrl_pouring  #old method
from pouring_unknown_geom.hybrid_control_pour import hybrid_cntrl_pouring_gp_compatible
# from pouring_unknown_geom.traj_gen import min_jerk_traj, trapezoidal_traj
from pouring_unknown_geom.traj_gen import trapezoidal_traj
from pouring_control_pkg.msg import MeasMsg  #for mh, mhdot  (this is right and produces h, dh/dt)
from pouring_control_pkg.msg import Hfilt #contains (header, hfilt(float64))
from sensor_msgs.msg import JointState
from pouring_msgs.msg import PourModelPred, HybCntrlData, PourAngStateMsg, PouringTimeDelay, SelectedGPModels

from pouring_unknown_geom.msg import PourControlAction, PourControlGoal, PourControlResult
# from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse


class Pouring_Main(object):
  def __init__(self, des_volume=100, theta_start=1.57323, omega_max= 10.0*np.pi/180, delta_omega_speed=0.1, pouring_Kp=1.0, vol_funct_cm_ml=lambda x:x, theta_rotate_max=2*np.pi/180):
    self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()
    self.des_volume = des_volume
    self.theta_start = theta_start

    #Set parameters for trapezoidal trajectory
    pouring_acc = 3.0; #was 3, linear for recieving container ml/s
    # Vmax_upper = 20.0;#10, 5; #ml/s
    # Vmax_lower = 5.0;#5, 3; #ml/s

    Vmax_upper = 5.0;#10, 5; #ml/s
    Vmax_lower = 3.0;#5, 3; #ml/s
    resid_max=15.0;
    resid_min=0.0;
    residual_update_radius=0.5;
    self.traj_gen = trapezoidal_traj.TrapTrajClass(h_des={'unit':'ml','val':des_volume}, vol_funct_cm_ml=vol_funct_cm_ml, acceleration=pouring_acc, Vmax_upper=Vmax_upper,Vmax_lower=Vmax_lower, resid_max=resid_max, resid_min=resid_min, residual_update_radius=residual_update_radius)
    self.header=None
    #object for control
    # self.hybrid_controller = hybrid_cntrl_pouring_gp_compatible.SawyerHybridControlPour(omega_max=omega_max, delta_omega_speed=delta_omega_speed, Kp=pouring_Kp, theta_rotate_max=theta_rotate_max) #currently seeding with 0's coefficients which will default make V/th  0, putting us in the constant rate phase
    self.hybrid_controller = hybrid_cntrl_pouring_gp_compatible.SawyerHybridControlPour(omega_max=omega_max, delta_omega_speed=delta_omega_speed, Kp=pouring_Kp, theta_rotate_max=theta_rotate_max)
    #Variables for messages:
    self.subs_var_mh = None# 0.0
    self.subs_var_mh_dot =  0.0
    self.subs_var_theta = 0.0
    self.subs_var_omega = 0.0
    self.subs_var_mh = 0.0
    #publish controller data:
    self.cntrl_data_pub = rospy.Publisher('Hybrid_controller_data',HybCntrlData,queue_size=1,latch=False)
    #for debugging:
    self.vol_des_history = None
    #Check if pour is completed - send action server instructions
    self.pour_finished_check_timeout = rospy.get_param('~pouring_goal_complete_time', 5.0) # 5.0 #seconds
    self.pour_finished_check_volume = rospy.get_param('~pouring_goal_complete_volume_error', 0.0) #2.0 #ml (ml of target volume)
    self.pour_finished_check_start_time = None
    #setup variables
    self.gp_selected_models = SelectedGPModels()  #this should not be reset as it is a latched topic
    self.train_gp_models_bool = True
    self.train_gp_models_bool_trigger = False
    self.setup_variables()
    #volume solution mode:
    self.volume_solution_mode = "nonparam"

  def setup_variables(self):
    #resets
    self.thread_lock.acquire()
    self.gen_traj_flag_reset = True  #flag to regenerate trajectory
    self.min_jerk_poly_reset = None
    self.time_traj_path_reset = 0.0 #keep time along a trajectory
    self.subs_var_model_struct = PourModelPred() #data type
    self.subs_var_model_struct.volume_solution_mode = "nonparam"
    self.subs_var_model_struct.coef_opt = np.zeros([10]).tolist()
    self.t_start_reset = rospy.Time.now().to_sec()
    self.global_reset = True
    self.pouring_time_delay_obj = PouringTimeDelay()
    self.pouring_time_delay_obj.time_delay = 2.0 #init
    self.hybrid_controller.shift_gp_volumes_bool = True
    self.thread_lock.release()
    #call init subscribers
    #self.init_subscribers() #dont want to train right before pouring,

  def reset_callback(self, req):
    rospy.logwarn("reset main node for pouring: Reset requested")
    self.setup_variables()
    # self.pouring_model_gp_selected = rospy.Subscriber('gp_selected_models',SelectedGPModels, self.gp_selected_models_callback, queue_size=1) #since this is a latched topic
    
    return TriggerResponse(True,"Reset complete")

  def obtain_trajectory(self,start_time = 0.0, current_height=0, final_height=100, current_velocity=0.0, residual=0.0, C_val=1.0):
    traj_dict = self.traj_gen.traj_gen(curr_time=start_time, h_init={'unit':'ml','val':current_height}, h_final={'unit':'ml','val':final_height}, vel_init={'unit':'ml/s','val':current_velocity}, residual=residual)
    return traj_dict

  def pub_zero_vel(self):
    #This case is triggered when no height is observed, safely stop the system
    #publish controller info
    hyb_cntrl_data = HybCntrlData()
    hyb_cntrl_data.header.stamp = rospy.Time.now()
    hyb_cntrl_data.control_state = "h unobserved"
    hyb_cntrl_data.traj_height = 0.0
    hyb_cntrl_data.traj_dheight = 0.0
    hyb_cntrl_data.ml_error = 0.0
    hyb_cntrl_data.predicted_height.pred_time = 0.0
    hyb_cntrl_data.predicted_height.pred_height = 0.0
    hyb_cntrl_data.predicted_height.pred_traj_height = 0.0
    hyb_cntrl_data.predicted_height.dv_dth_alpha = 0.0
    hyb_cntrl_data.cmd_omega = 0.0  #commanded omega
    self.cntrl_data_pub.publish(hyb_cntrl_data)


  def control_thread(self, event):
    #make sure that the action hasn't been canceled
    if not self._action_server.is_active():
      #rospy.logwarn("Waiting for action server to be active")
      return;
    #Get the action goal #TODO get it only once at begining?
    self.thread_lock.acquire()
    action_goal = self._action_goal
    self.thread_lock.release()
    self.des_volume = action_goal.desired_volume_ml
    #self.theta_start = action_goal.theta_start
    #Obtain States
    self.thread_lock.acquire()
    #Put this check here to ensure it reaches
    self.cntrl_header = self.header
    height_msg_bool = True
    if type(self.subs_var_mh) == type(None):
       height_msg_bool = False
    self.cntrl_var_mh = copy.deepcopy(self.subs_var_mh)  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.cntrl_pouring_time_delay_obj = self.pouring_time_delay_obj #for time delay
    self.cntrl_var_mh_dot = copy.deepcopy(self.subs_var_mh_dot)  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.subs_var_mh = None
    self.subs_var_mh_dot = None
    self.cntrl_var_theta = self.subs_var_theta  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.cntrl_var_omega_meas = self.subs_var_omega #this is the actual joint state velocity needed for control (history)
    self.cntrl_var_model_struct = self.subs_var_model_struct  #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.cntrl_gp_selected_models = self.gp_selected_models
    #print "the contents of gp selected models lengths is ", len(self.cntrl_gp_selected_models.gp_volume_models)
    train_gp_models_bool_trigger_local = self.train_gp_models_bool_trigger
    #For resetting
    if self.global_reset:
      self.gen_traj_flag = copy.deepcopy(self.gen_traj_flag_reset)
      self.min_jerk_poly = copy.deepcopy(self.min_jerk_poly_reset )
      self.time_traj_path = copy.deepcopy(self.time_traj_path_reset) #keep time along a trajectory
      self.t_start = copy.deepcopy(self.t_start_reset)
      self.global_reset = False
    self.thread_lock.release()
    if not height_msg_bool:
      self.pub_zero_vel()
      return
    #Find the current differential time
    self.t_curr = rospy.Time.now().to_sec()
    self.dt_curr = self.t_curr - self.t_start
    #If neccessary, calculate desired trajectory
    #determine if sufficent change in residual has occured
    traj_recalc_bool = self.traj_gen.re_evaluate_residual(curr_residual=self.cntrl_var_model_struct.residual_avg)
    if traj_recalc_bool:
      self.gen_traj_flag = True
      # self.t_start = rospy.Time.now().to_sec() - self.time_traj_path  #keep the same time, you should just be at a different spot in a new trajectory
      self.t_start = rospy.Time.now().to_sec() #new trajectory is found, then give it a new starting time (this is not a pause)
    if self.gen_traj_flag:
      #replan trajectory
      # traj_dict = self.obtain_trajectory(start_time=self.t_start, current_height=self.cntrl_var_mh, final_height=self.des_volume, current_velocity = np.max([self.cntrl_var_mh_dot,0.0]), residual=self.cntrl_var_model_struct['residual'])
      traj_dict = self.obtain_trajectory(start_time=self.t_start, current_height=self.cntrl_var_mh, final_height=self.des_volume, current_velocity = 0.0, residual=self.cntrl_var_model_struct.residual_avg)  #velocity too erratic to use in planner
      self.traj_dict = traj_dict
      self.gen_traj_flag = False
    #Given the traj coeff, find the current vol_des, h_dot_des
    traj_eval = self.traj_gen.calc_curr_traj_params(curr_t=self.dt_curr, traj_dict=self.traj_dict, use_differential_time_bool=True) #{'h_t':h_t, 'v_t':v_t}
    vol_des, vol_des_dot = traj_eval['h_t'], traj_eval['v_t']
    #Now evaluate trajectory at future time (t+tau)
    self.cntrl_pouring_time_delay_obj.time_delay
    traj_eval_t_plus_tau = self.traj_gen.calc_curr_traj_params(curr_t=self.dt_curr+self.cntrl_pouring_time_delay_obj.time_delay, traj_dict=self.traj_dict, use_differential_time_bool=True) #{'h_t':h_t, 'v_t':v_t}
    # traj_eval_t_plus_tau = self.traj_gen.calc_curr_traj_params(curr_t=self.dt_curr+self.hybrid_controller.pouring_time_delay, traj_dict=self.traj_dict, use_differential_time_bool=True) #{'h_t':h_t, 'v_t':v_t}
    vol_des_t_plus_tau,vol_des_t_plus_tau_dot = traj_eval_t_plus_tau['h_t'], traj_eval_t_plus_tau['v_t']
    if self.cntrl_var_mh < traj_eval['h_init']:
      #if the height is below the init height then there was sloshing or visual error which caused the intial condtion to be higher than reality (settling causes lower measurement)
      self.gen_traj_flag = True
    #debug:
    if self.vol_des_history:
      if  vol_des > self.des_volume:
        traj_eval = self.traj_gen.calc_curr_traj_params(curr_t=self.dt_curr, traj_dict=self.traj_dict, use_differential_time_bool=True, debug_flag=False) #{'h_t':h_t, 'v_t':v_t}
        #this has a keyboard flag
    self.vol_des_history = vol_des
    #Find the theta:
    theta = self.cntrl_var_theta
    #Query Hybrid Controller:
    # omega, new_traj_bool, curr_hybrid_state_str, ml_error, prediction_dict = self.hybrid_controller.Controller_output(H_final=self.des_volume, vol_des_t_plus_tau = vol_des_t_plus_tau, vol_des=vol_des, vol_des_dot=vol_des_dot,
    #                                                                           h_meas=self.cntrl_var_mh, h_meas_dot=self.cntrl_var_mh_dot, theta_bar=theta, omega_meas=self.cntrl_var_omega_meas,
    #                                                                           Abeta_func= lambda x: 1.0 , coef=self.cntrl_var_model_struct , pour_sign=1, current_control_time=self.dt_curr)


    # print "In main: ", self.cntrl_var_model_struct, "."
    if self.train_gp_models_bool:
      #train_gp_models_bool_local = True
      train_gp_models_bool_local = False #this should be handled in initialization
    else:
      train_gp_models_bool_local = False

    omega, new_traj_bool, curr_hybrid_state_str, ml_error, prediction_dict = self.hybrid_controller.Controller_output(Vol_final=self.des_volume, vol_des_t_plus_tau = vol_des_t_plus_tau, vol_des=vol_des, vol_des_dot=vol_des_dot,
                                                                              vol_meas=self.cntrl_var_mh, vol_meas_dot=self.cntrl_var_mh_dot, theta_bar=theta, omega_meas=self.cntrl_var_omega_meas,
                                                                              Abeta_func= lambda x: 1.0 , pour_model_obj=self.cntrl_var_model_struct , pour_sign=1, current_control_time=self.dt_curr,
                                                                              time_delay=self.cntrl_pouring_time_delay_obj.time_delay, train_gp_models_bool=train_gp_models_bool_local, gp_selected_models=self.cntrl_gp_selected_models)

    #now check the gp training trigger
    if train_gp_models_bool_trigger_local:
      self.train_gp_models_bool = False

    if new_traj_bool:
      #controller was force into alternate hybrid form hence replan to be safe
      self.gen_traj_flag = new_traj_bool
      #***With trap traj and dynamic pouring when the flag is thrown there must be an actual new trajectory whose start time is the current time (or else do not throw the flag)
      self.t_start = rospy.Time.now().to_sec()  #keep the same time, you should just be at a different spot in a new trajectory
    else:
      self.time_traj_path = self.dt_curr
    # print "current time_traj_path: ", self.time_traj_path, " and start time: ", self.t_start
    #command the velocity:
    #publish controller info
    hyb_cntrl_data = HybCntrlData()
    hyb_cntrl_data.header = self.cntrl_header
    hyb_cntrl_data.control_state = curr_hybrid_state_str
    hyb_cntrl_data.traj_height = float(vol_des)
    hyb_cntrl_data.traj_dheight = float(vol_des_dot)
    hyb_cntrl_data.ml_error = float(ml_error)
    hyb_cntrl_data.predicted_height.pred_time = prediction_dict['pred_time']
    hyb_cntrl_data.predicted_height.pred_height = prediction_dict['pred_height']
    hyb_cntrl_data.predicted_height.pred_traj_height = prediction_dict['pred_traj_height']
    hyb_cntrl_data.predicted_height.dv_dth_alpha = prediction_dict['dv_dth_alpha']
    hyb_cntrl_data.cmd_omega = omega  #commanded omega
    self.cntrl_data_pub.publish(hyb_cntrl_data)
    #First see if you are within volume of goal (2-3ml), this is not absolute as heights well above the des height should timeout
    if (self.des_volume - self.cntrl_var_mh) < self.pour_finished_check_volume:
      rospy.loginfo("Finished state: greater than final height")
      if type(self.pour_finished_check_start_time) == type(None):
        self.pour_finished_check_start_time = rospy.Time.now()
      pour_finished_duration = rospy.Time.now() - self.pour_finished_check_start_time #get duration
      rospy.loginfo("checking finish state duration %d"%pour_finished_duration.to_sec())
      if pour_finished_duration.to_sec() >= self.pour_finished_check_timeout:
        #if you have maintained this for longer than set duration then send completed flag in action-server
        self._action_server.set_succeeded(result=None,text="Finished Pouring")
        rospy.loginfo("should have sent finished pouring to action-server")
    else:
      #fluid was above the final height but fell again before the timeout, so keep going!
      self.pour_finished_check_start_time = None

  def mh_callback(self,req):
    #req is callback obj
    self.header = req.header
    self.subs_var_mh = req.mh #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.subs_var_mh_dot = req.mh_dot
    #rospy.loginfo("reached mh callback")
    #self.control_thread()

  def theta_callback(self,req):
    self.thread_lock.acquire()
    # self.subs_var_theta = req.position[req.name.index('right_j6')] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    # self.subs_var_omega = req.velocity[req.name.index('right_j6')] #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.subs_var_theta = req.theta #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.subs_var_omega = req.omega #locking is occuring on subs_var_mh (to avoid memory accessing issues)
    self.thread_lock.release()
    #vel = req.velocity[req.name.index('right_j6')]
    #torque = req.effort[req.name.index('right_j6')]

  def model_callback(self,req):
    # req.residual_std; req.header; req.type
    self.thread_lock.acquire()
    # self.subs_var_model_struct ={'coef':req.coef_opt, 'domain':req.domain, 'residual':req.residual_avg}
    self.subs_var_model_struct = req
    self.thread_lock.release()

  def gp_selected_models_callback(self,req):
    #print "Entered the gp selected models callback"
    self.thread_lock.acquire()
    self.gp_selected_models = req
    # print "gp selected model msg", self.gp_selected_models
    self.train_gp_models_bool_trigger = True #trigger this affirmative
    rospy.logwarn("model_callback recieved in main script")
    self.thread_lock.release()
    self.hybrid_controller.train_gp_models_callback(gp_selected_models=req, volume_solution_mode=self.volume_solution_mode)

  def action_goal_callback(self):
    rospy.loginfo("Starting Pouring")
    #accept the new action goal
    action_goal = self._action_server.accept_new_goal()
    self.thread_lock.acquire()
    self._action_goal = action_goal
    self.thread_lock.release()
    rospy.logwarn("desired_volume_ml %d", action_goal.desired_volume_ml)
    #rospy.logwarn("theta_start %g", action_goal.theta_start)

  def action_preempt_callback(self):
    rospy.loginfo("Pouring Preempted")
    #Set the action state to preempted
    self._action_server.set_preempted(result=None,text="Preempted externally")

  def time_delay_callback(self,data):
    self.thread_lock.acquire()
    self.pouring_time_delay_obj = data
    self.thread_lock.release()

  def gp_train_reset_callback(self,req):
    self.pouring_model_gp_selected = rospy.Subscriber('gp_selected_models',SelectedGPModels, self.gp_selected_models_callback, queue_size=1)
    return TriggerResponse(True,"GP train in main Reset complete")


  def init_subscribers(self):
    self.t_start = rospy.Time.now().to_sec()
    # self.theta_sub = rospy.Subscriber('joint_states',JointState, self.theta_callback, queue_size=1) #joint angle callback
    self.theta_sub = rospy.Subscriber('pour_angle_state',PourAngStateMsg, self.theta_callback, queue_size=1) #joint angle callback
    self.mh_sub = rospy.Subscriber('mh_mh_dot',MeasMsg, self.mh_callback, queue_size=1) #mass (m), height (h) callback
    #self.h_filt_sub = rospy.Subscriber('h_filtered',Hfilt, self.h_filt_callback, queue_size=1) #mass (m), height (h) callback
    self.pouring_model = rospy.Subscriber('Pour_Model_opt',PourModelPred, self.model_callback, queue_size=1)
    # self.pouring_model_gp_selected = rospy.Subscriber('gp_selected_models',SelectedGPModels, self.gp_selected_models_callback, queue_size=1)
    self.pouring_model_gp_selected = rospy.Subscriber('gp_selected_models',SelectedGPModels, self.gp_selected_models_callback, queue_size=1)
    self.control_timer = rospy.Timer(rospy.Duration(1/30.), self.control_thread)
    self.time_delay_sub = rospy.Subscriber("time_delay_msg",PouringTimeDelay,self.time_delay_callback,queue_size=1)
    #Setup the action server
    self._action_server = actionlib.SimpleActionServer("pour_action", PourControlAction, auto_start = False)
    self._action_goal = []
    self._action_server.register_goal_callback(self.action_goal_callback)
    self._action_server.register_preempt_callback(self.action_preempt_callback)
    self._action_server.start()
    #for reset
    self.reset_srv = rospy.Service('reset_main_pouring_node', Trigger, self.reset_callback)
    self.reset_srv_gptrain = rospy.Service('reset_main_pouring_node_gp_training', Trigger, self.gp_train_reset_callback)
    rospy.spin()

def main(argv):
  rospy.init_node('pouring_control_main_node')
  ml_volume = float(rospy.get_param("~ml_des_volume", 100))
  rospy.logwarn("Desired volume ml: %f"%ml_volume)
  Area = float(rospy.get_param("~recieving_container_area", 41.66)) #cm**2
  rospy.logwarn("receiver area is: %f"%Area)
  #theta_start = float(rospy.get_param("~theta_start", 2.10))
  #rospy.logwarn("theta start is: %f"%theta_start)
  # theta_start = 1.57323046875
  #theta_start = 2.10 #dont spill
  theta_start = 0.0; #Not used in main anymore
  omega_max= 5.0*np.pi/180 #controller max commanded angular velocity for the wrist
  #for the plastic flask, the average dv/dth = 500 ml/rad, hence to be around Vmax for trapezoidal traj this means that dth/dt * dv/dth = dv/dt
  #delta_omega_speed = 0.1 #0.02 #This is the re-initiate pour speed in the hybrid state to reinitiate pouring  (was 0.1)
  delta_omega_speed = 0.03#0.1 #0.02 #This is the re-initiate pour speed in the hybrid state to reinitiate pouring  (was 0.1)
  #delta_omega_speed = 0.06#0.1 #0.02 #This is the re-initiate pour speed in the hybrid state to reinitiate pouring  (was 0.1)
  if ml_volume >= 150: delta_omega_speed = 0.06
  if ml_volume >= 200: delta_omega_speed = 0.1
 
  pouring_Kp = 1. #was 0.5 gain on height for pouring velocity

  # vol_funct_cm_ml=lambda x:Area*x  #Glass beaker 41.66cm**2, plastic beaker: 31.768 //38.515
  vol_funct_cm_ml=lambda x:x  #Volume is now being reported, not height

  theta_rotate_max=1.5*np.pi/180 #max angle system can rotate backwards to stop a pour
  cls_obj = Pouring_Main(des_volume=ml_volume, theta_start=theta_start, omega_max= omega_max, delta_omega_speed=delta_omega_speed, pouring_Kp=pouring_Kp, vol_funct_cm_ml=vol_funct_cm_ml, theta_rotate_max=theta_rotate_max)
  #When data is available control
  volume_solution_mode = rospy.get_param("~volume_solution_mode")
  cls_obj.volume_solution_mode = volume_solution_mode
  cls_obj.init_subscribers()

if __name__ == '__main__':
  main(sys.argv[1:])

'''
Note that in the incoming PourAngStateMsg, the angle difference must already be applied
'''
