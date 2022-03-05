#!/usr/bin/env python2
#Created by Monroe Kennedy III, 11/2017, All rights reserved

import numpy as np, math
import rospy

import matplotlib.pyplot as plt


class TrapTrajClass(object):
  """TrapTrajClass Documentation
    Input: 
      -h_init: height at t_0 (current time)
      -h_des: goal height (const for a given pour) :: dictionary: {'unit':'ml','val':100}
      -dVolume(lambda funct(h_des,h_init)): remaining volume at a given time 
      -v_init: initial velocity at t_0 (current time)
    -----------------------------------------------------------------------------------------
    Output:
      -dictionary: trap_dict  
        - {"legs":[leg_1,leg_2,leg_3], 'init_time':val}
          - leg_i: {"vel":[a_1,a_0]
                    "pos":[a_2,a_1,a_0]
                    "dtau":val
                    "leg_idx":idx}
    -----------------------------------------------------------------------------------------
    Usage:
      Generate Traj:
      cls_obj = TrapTrajClass(h_des={'unit':'ml','val':h_final}, vol_funct_cm_ml=lambda x:41.66*x, acceleration=pouring_acc, Vmax_upper=Vmax_upper,Vmax_lower=Vmax_lower, resid_max=15.0, resid_min=0.0, residual_update_radius=0.5)
      traj_dict = cls_obj.traj_gen(curr_time=0, h_init={'unit':'ml','val':0}, h_final={'unit':'ml','val':100}, vel_init={'unit':'ml/s','val':0}, dVolume=None, residual=0.0)

      #CHeck if updated residual requires new trajectory
      traj_recalc_bool = cls_obj.re_evaluate_residual(curr_residual=resid_curr)
      if traj_recalc_bool: traj_dict = cls_obj.traj_gen(curr_time=t_curr, h_init={'unit':'ml','val':h_curr}, h_final={'unit':'ml','val':h_final}, vel_init={'unit':'ml/s','val':v_curr}, dVolume=None, residual=resid_curr)
      

      Query Traj:
      traj_eval = cls_obj.calc_curr_traj_params(curr_t=t_curr, traj_dict=traj_dict) #{'h_t':h_t, 'v_t':v_t}
  """

  def wait(self):
      #Debug tool
      raw_input("Press Enter to continue...")


  def __init__(self, h_des={'unit':'ml','val':100}, vol_funct_cm_ml=lambda x:41.66*x, acceleration=0.05, Vmax_upper=0.4,Vmax_lower=0.1, resid_max=10.0, resid_min=0.0, residual_update_radius=0.5):
    #Set constant terms
    self.vol_funct_cm_ml = vol_funct_cm_ml; self.Vmax_upper = Vmax_upper; self.Vmax_lower = Vmax_lower; self.resid_min = resid_min; self.resid_max = resid_max; self.residual_update_radius = residual_update_radius; self.acceleration=acceleration  #acceleration of the fluid ml/s**2
    if h_des['unit'] in 'cm': volume = vol_funct_cm_ml(h_des['val']); self.h_des = {'unit':'ml','val':100}
    else: self.h_des = h_des
    self.traj_dict = {'legs':[],'init_time':0}
    self.prev_residual= None #Here the resdiual list is binned in order to determine later if the traj should be recalculated based on change in residual

  def eval_traj_leg(self,traj_leg=None, time=None, cumulated_time=0):
    t = time - cumulated_time; 
    pos = traj_leg['pos'][0]*t**2 + traj_leg['pos'][1]*t + traj_leg['pos'][2]; 
    vel =  traj_leg['vel'][0]*t + traj_leg['vel'][1]#Calc Position then vel
    return pos,vel

  def calc_curr_traj_params(self,curr_t=0, traj_dict=None, use_differential_time_bool=False, debug_flag= False):
    #1. First find out which leg of the trajectory the current time is in
    curr_leg_idx = 0;
    traj_time = 0
    if use_differential_time_bool: t_curr = curr_t #This uses dt_curr wihch is self.t_curr - self.t_start passed in
    else: t_curr = curr_t - traj_dict['init_time']#Find the curr time with possible shifting due to reset start time if system paused

    for idx in range(len(traj_dict['legs'])):
      traj_time += traj_dict['legs'][idx]['dtau']; curr_leg_idx = idx;
      if t_curr <= traj_time: break


    total_traj_time= np.sum([traj_dict['legs'][idx]['dtau'] for idx in range(len(traj_dict['legs']))])
    if t_curr > total_traj_time: t_curr = total_traj_time #this is if it is greater than total tiem

    #2. For the given leg, evaluate
    curr_leg = traj_dict['legs'][curr_leg_idx];
    cumulated_time = 0
    for idx in range(curr_leg_idx): cumulated_time += traj_dict['legs'][idx]['dtau']
    h_t, v_t = self.eval_traj_leg( traj_leg=curr_leg, time=t_curr, cumulated_time=cumulated_time)

    if debug_flag:
      #in this event the trajectory has returned a value above the desired height
      rospy.loginfo('DEBUGGING TRAJ EVAL current time: %.2f, traj leg summed time: %.2f, cumulated time: %.2f, total traj time: %f' %(t_curr,traj_time, cumulated_time, total_traj_time))
      self.wait()

    return {'h_t':h_t, 'v_t':v_t, 'h_init':traj_dict['legs'][0]['pos'][2]}


  def residual_vmax_calculator(self,residual=0.0):
    #This function uses the sigmoid function Vmax = A/(B + exp(-Cx))  with A = l*B + exp(-r_u) and B= (l*exp(-r_u)-u*exp(-r_l))/(u-l)
    r_max = self.resid_max; r_min = self.resid_min; upper = self.Vmax_upper; lower = self.Vmax_lower;
    #B = lower/(upper-lower)
    B = (upper-lower)/lower
    A = B*upper
    #Vmax = lower+ A/(B + np.exp(residual))
    c = 8.0
    Vmax = lower+ A/(B + np.exp( (residual/r_max)*c ))
    return Vmax


  def re_evaluate_residual(self,curr_residual=0.0):
    if type(self.prev_residual) == type(None): self.prev_residual = curr_residual; return True
    else:
      if np.abs(self.prev_residual - curr_residual) >= self.residual_update_radius: self.prev_residual = curr_residual; return True
      else: return False

  def adjust_vmax(self, dVolume=None, acceleration=None, vel_init=None):
    #this assumes vmax > vel_init, otherwise either 1 or 3 legs is neccessary, and handled in other cases
    Vmax = np.sqrt(dVolume*acceleration + vel_init['val']**2/2.0); return Vmax 


  def ramp_down_v_init(self,curr_time=0,vel_init=None,h_init=None,dVolume=None):
    #This is for the first case, when immediate ramp down is required. This is done by seeing if triangle (a/2)*V_init**2 >= dVolume, and if so prescribing triangular trajectory that brings this to zero (adjusting acceleration in this case only)
    if dVolume > 0.0005:
      if vel_init['val']**2/(self.acceleration*2.0) > dVolume: acceleration = vel_init['val']**2/(dVolume*2.0)#strictly greater, hence recalculate acceleration
      else: acceleration = self.acceleration
      dtau_3 = vel_init['val']/acceleration#Calculate transpired time dtau_3
      #Populate the traj dict with a single leg that decelerates
      leg_decelerate =  {"vel":[-acceleration,vel_init['val']], "pos":[-acceleration/2.0, vel_init['val'], h_init['val']], "dtau":dtau_3, "leg_idx":0}
    else:
      #this is if trajectory is started when there is no volume to pour
      dtau_3 = 10.0 #vel_init['val']/acceleration#Calculate transpired time dtau_3
      acceleration= 0.0
      leg_decelerate =  {"vel":[0.0,0.0], "pos":[0.0, 0.0, h_init['val']+dVolume], "dtau":dtau_3, "leg_idx":0}
    self.traj_dict['legs'].append(leg_decelerate)
    return self.traj_dict


  def triangular_traj_gen(self,vel_init=None, acceleration=None,Vmax=None, h_init=None):
    #1. Calculate times
    dtau_1 = (Vmax - vel_init['val'])/acceleration
    dtau_3 = Vmax/acceleration
    h_tau_1 = (acceleration/2.0)*dtau_1**2 + vel_init['val']*dtau_1 + h_init['val'] #find the position at the apex
    #2. Find acceleration leg
    leg_accelerate = {"vel":[acceleration,vel_init['val']], "pos":[acceleration/2.0, vel_init['val'], h_init['val']], "dtau":dtau_1, "leg_idx":0}
    #3. Find deceleration leg
    leg_decelerate = {"vel":[-acceleration,Vmax], "pos":[-acceleration/2.0, Vmax, h_tau_1], "dtau":dtau_3, "leg_idx":1}
    self.traj_dict['legs'].append(leg_accelerate); self.traj_dict['legs'].append(leg_decelerate)
    return self.traj_dict


  def full_trapezoid_traj_gen(self, vel_init=None, acceleration=None,Vmax=None, h_init=None, dVolume=None):
    #1. Calculate times and volumes
    dtau_1 = np.abs(Vmax - vel_init['val'])/acceleration; dtau_3 = Vmax/acceleration
    if vel_init['val'] <= Vmax: dVolume_leg1 = vel_init['val']*np.abs(Vmax - vel_init['val'])/acceleration +  (Vmax - vel_init['val'])**2/(acceleration*2.0)#the lower rectangle is defined by V_init
    else: dVolume_leg1 = Vmax*np.abs(Vmax - vel_init['val'])/acceleration +  (Vmax - vel_init['val'])**2/(acceleration*2.0)#the lower rectangle is defined by Vmax
    dVolume_leg3 = Vmax**2/(acceleration*2.0); dVolume_leg2 = dVolume - (dVolume_leg1 + dVolume_leg3); dtau_2 = dVolume_leg2/Vmax;
    #2. First leg between vel_init and Vmax
    if vel_init['val'] < Vmax:
      #2a. case v_init < v_max
      leg_1 = {"vel":[acceleration,vel_init['val']], "pos":[acceleration/2.0, vel_init['val'], h_init['val']], "dtau":dtau_1, "leg_idx":0}; h_tau_1 = (acceleration/2.0)*dtau_1**2 + vel_init['val']*dtau_1 + h_init['val'] #find the position at the apex
    else: 
      #2b. case vel_init >= v_max
      leg_1 = {"vel":[-acceleration,vel_init['val']], "pos":[-acceleration/2.0, vel_init['val'], h_init['val']], "dtau":dtau_1, "leg_idx":0}; h_tau_1 = (-acceleration/2.0)*dtau_1**2 + vel_init['val']*dtau_1 + h_init['val'] #find the position at the intersection
    #3. Second leg at Vmax
    leg_2 = {"vel":[0.0, Vmax], "pos":[0.0, Vmax, h_tau_1], "dtau":dtau_2, "leg_idx":1}; h_tau_2 = Vmax*dtau_2 + h_tau_1
    #4. Third leg between Vmax and vel=0 
    leg_3 = {"vel":[-acceleration,Vmax], "pos":[-acceleration/2.0, Vmax, h_tau_2], "dtau":dtau_3, "leg_idx":2}
    '''Remember for time, first leg: (t-t_0), second (t-(t_0+dtau1)), then (t-(t_0+dtau1+dtau2)) '''
    #Append and return the trajectory
    self.traj_dict['legs'].append(leg_1); self.traj_dict['legs'].append(leg_2); self.traj_dict['legs'].append(leg_3);
    return self.traj_dict


  def traj_gen(self, curr_time=0, h_init={'unit':'ml','val':0}, h_final={'unit':'ml','val':100}, vel_init={'unit':'ml/s','val':0}, dVolume=None, residual=0.0):
    self.traj_dict['init_time'] = curr_time; self.traj_dict['legs'] = [] #If traj gen is being called, then previously stored trajectory components need to be fully erased. 
    Vmax = self.residual_vmax_calculator(residual=residual)
    if not dVolume:
      #determine of the dVolume must be calculated
      #use h_init, h_final, vol_funct_cm_ml to generate dVolume
      if h_init['unit'] in 'cm': h_init['val'] = self.vol_funct_cm_ml(h_init['val']); h_init['unit'] = 'ml'
      if h_final['unit'] in 'cm': h_final['val'] = self.vol_funct_cm_ml(h_final['val']); h_final['unit'] = 'ml'
      if h_init['val'] >= h_final['val']: h_init['val'] = h_final['val']  #this is resetting the intial condition to ensure observed higher volumes are not h_init which translates to desired volume
      dVolume = h_final['val'] - h_init['val']
    if dVolume <= 0: dVolume=0; rospy.loginfo("0 dVolume")
    if vel_init['unit'] in 'cm/s':
      vel_init['unit'] = 'ml/s'; vel_init['val'] = self.vol_funct_cm_ml(vel_init['val'])
    #Trajectory Generation Algorithm:
    if vel_init['val']**2/(self.acceleration*2.0) >= dVolume:
      # print "Case A" 1. Determine if immediate deceleration is neccessary
      traj_dict = self.ramp_down_v_init(curr_time=curr_time,vel_init=vel_init,h_init=h_init,dVolume=dVolume)
      self.traj_dict = traj_dict; return self.traj_dict
    elif Vmax**2/(self.acceleration*2.0) >= dVolume:
      # print "Case B" #2. If Vmax will exceed desired volume, then adjust max speed, this assumes V_init < Vmax
      Vmax = self.adjust_vmax(dVolume=dVolume, acceleration=self.acceleration, vel_init=vel_init)
      traj_dict = self.triangular_traj_gen(vel_init=vel_init, acceleration=self.acceleration,Vmax=Vmax, h_init=h_init)
      self.traj_dict = traj_dict; return self.traj_dict
    else: # Vmax**2/(self.acceleration*2.0) < dVolume:
      # print "Case C"
      #3. Determine trapezoid traj with more than one leg
      if vel_init['val'] <= Vmax: dVolume_leg1 = (Vmax**2 - vel_init['val']**2)/(2*self.acceleration)
      else: dVolume_leg1 = (vel_init['val']**2 - Vmax**2)/(2*self.acceleration)
      dVolume_leg3 = Vmax**2/(self.acceleration*2.0)
      if (dVolume_leg1+ dVolume_leg3) >= dVolume:
        #3a. Two legs (using requested Vmax)
        if vel_init['val'] >= Vmax: 
          traj_dict = self.ramp_down_v_init(curr_time=curr_time, vel_init=vel_init, h_init=h_init, dVolume=dVolume); 
          self.traj_dict = traj_dict; return self.traj_dict #This implies that dV1+dV3 = (a/2)*((V_init**2 - Vmax**2) + Vmax**2) = (a/2)*V_init**2
        else: #vel_init['val'] < Vmax: 
          #Case C1-2: This means that dV1+dV3 = a(Vmax**2 - (Vinit**2)/2.) Hence a new Vmax must be selected
          Vmax = self.adjust_vmax(dVolume=dVolume, acceleration=self.acceleration, vel_init=vel_init)
          traj_dict = self.triangular_traj_gen(vel_init=vel_init, acceleration=self.acceleration,Vmax=Vmax, h_init=h_init)
          self.traj_dict = traj_dict; return self.traj_dict
      else: #(dVolume_leg1+ dVolume_leg3) > dVolume:
        # Case C2"
        #3b. Three legs (using requested Vmax)
        traj_dict = self.full_trapezoid_traj_gen(vel_init=vel_init, acceleration=self.acceleration,Vmax=Vmax, h_init=h_init, dVolume=dVolume)
        self.traj_dict = traj_dict; return self.traj_dict






def main():
  #Test out the trapezoidal script (each case) and plot results
  h_init = 0; #ml
  h_final = 100 #ml
  # vel_init = 0.0#ml/s
  pouring_acc = 0.5#2 #linear for recieving container ml/s
  Vmax_upper = 10; #ml/s
  Vmax_lower = 5 #ml/s
  #TEST CASES
  test = 'full_trap' #specify the test
  #1. Triangular
  if test in 'triangular':
    t_0 = 0.0 #inital time in seconds
    vel_init = 0.0 #rad/s
    h_final = 30 #ml
  elif test in 'ramp_down':
    #2. Ramp Down
    t_0 = 0.0 #inital time in seconds
    case = 'pure_ramp'
    if case in 'pure_ramp':
      vel_init = 40.0 #ml/s  #this is a pure ramp down
    elif case in 'dec_const_dec':
      #with Vmax upper being 10 ml/s, this creates decrease, constant, decrease (V_init > Vmax)
      h_final = 100 #ml
      vel_init = 15.0#40.0 #ml/s
    else:
      vel_init = 0.0#40.0 #rad/s    
  elif test in 'full_trap':
    h_final = 100
    t_0 = 0.0 #inital time in seconds
    vel_init = 0.0 #rad/s

  cls_obj = TrapTrajClass(h_des={'unit':'ml','val':h_final}, vol_funct_cm_ml=lambda x:41.66*x, acceleration=pouring_acc, Vmax_upper=Vmax_upper,Vmax_lower=Vmax_lower, resid_max=15.0, resid_min=0.0, residual_update_radius=0.5)
  traj_variable_residual = True #choose which cases to test below (residuals present or not)
  if not traj_variable_residual:
    traj_dict = cls_obj.traj_gen(curr_time=t_0, h_init={'unit':'ml','val':h_init}, h_final={'unit':'ml','val':h_final}, vel_init={'unit':'ml/s','val':vel_init}, dVolume=None, residual=0.0)
    for idx in range(len(traj_dict['legs'])): leg_curr = traj_dict['legs'][idx]
    t = np.linspace(t_0,t_0+50,100)
    h_list= []; v_list = []; t_final = [];
    for idx in range(len(t)):
      t_curr = t[idx];
      traj_eval = cls_obj.calc_curr_traj_params(curr_t=t_curr, traj_dict=traj_dict) #{'h_t':h_t, 'v_t':v_t}
      h_list.append(traj_eval['h_t']); v_list.append(traj_eval['v_t']); t_final.append(t_curr)
      if traj_eval['v_t'] < 0.0: break
    plt.figure(1);
    h_plot = plt.plot(t_final,h_list,label='h'); v_plot = plt.plot(t_final,v_list,label='dh');
    plt.ylabel('ml'); plt.xlabel('t(s)'); plt.title("Trajectory with constant residual"); plt.legend(loc=2);
    plt.show()
  else:
    #Variable Residual: as the residual varies the parameters for the vmax function vary
    A,B,C,D=1.0,.15,0,20.0 
    # A,B,C,D=1.0,1,0,1.0 
    t = np.linspace(t_0,t_0+50,1000)
    resid_list = A*np.sin(B*t + C) + D
    h_list= []; v_list = []; t_final = []; resid_disp_list = []; resid_prev = None;
    for idx in range(len(t)):
      t_curr = t[idx]
      #Update the traj based on the current residual
      resid_curr = resid_list[idx]
      if len(h_list) == 0: h_curr = h_init; v_curr = vel_init
      else: h_curr = h_list[-1]; v_curr = v_list[-1]
      traj_recalc_bool = cls_obj.re_evaluate_residual(curr_residual=resid_curr)#The cls object for traj gen was adjusted each time, when it should clear in this case for brand new trajectory
      if traj_recalc_bool:
        traj_dict = cls_obj.traj_gen(curr_time=t_curr, h_init={'unit':'ml','val':h_curr}, h_final={'unit':'ml','val':h_final}, vel_init={'unit':'ml/s','val':v_curr}, dVolume=None, residual=resid_curr)
      #Now use the updated traj in the calculation
      traj_eval = cls_obj.calc_curr_traj_params(curr_t=t_curr, traj_dict=traj_dict) #{'h_t':h_t, 'v_t':v_t}
      h_list.append(traj_eval['h_t']); v_list.append(traj_eval['v_t']); resid_disp_list.append(resid_curr); t_final.append(t_curr)
      # if traj_eval['v_t'] < 0.0: break
    Vmax_upper_list = [Vmax_upper for idx in range(len(t_final))]; Vmax_lower_list = [Vmax_lower for idx in range(len(t_final))];
    plt.figure(2)
    h_plot = plt.plot(t_final,h_list,label='h'); v_plot = plt.plot(t_final,v_list,label='dh');
    resid_plot = plt.plot(t_final,resid_disp_list,label='resid'); vmax_upper_plot = plt.plot(t_final,Vmax_upper_list, label='Vmax_upper');  vmax_lower_plot = plt.plot(t_final,Vmax_lower_list, label='Vmax_lower');
    plt.ylabel('ml'); plt.xlabel('t(s)'); plt.title("Trajectory with variable residual"); plt.legend(loc=2);

    rlist = np.linspace(-10,10,100).tolist()
    vmax_list = []
    for r in rlist: vmax_list.append(cls_obj.residual_vmax_calculator(residual=r))
    plt.figure(3)
    plt.plot(rlist,vmax_list)
    plt.show();


if __name__ == '__main__':
  main()
















