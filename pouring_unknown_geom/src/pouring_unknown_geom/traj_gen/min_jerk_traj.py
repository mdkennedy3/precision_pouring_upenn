import numpy as np
import matplotlib.pyplot as plt


class MinJerk(object):
  """docstring for MinJerk"""
  def __init__(self, *bc):
    self.bc = bc

  def set_bc(self, h_init=0.0 ,h_final = 100 , t_inital=0, t_bounds=[5,15] ):
    init_cond=[h_init,0,0]
    final_cond=[h_final,0,0]
    self.bc = {"i":init_cond, "f":final_cond, 't_inital':t_inital ,'tbounds':t_bounds}


  def time_subs(self,t):
    mat = np.matrix([[t**5, t**4, t**3, t**2, t, 1],
                     [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],
                     [20*t**3, 12*t**2, 6*t, 2, 0, 0]])
    return mat

  def min_jerk_poly(self, residual=0, Cval=1.0):
    # Cval in [0,1], where 1 penalizes large residuals, and lower C's allow for larger residuals for faster pours
    #generate the left hand side which are the initial and final pos, vel, acc
    LH_side = np.vstack([np.matrix(self.bc["i"]).T, np.matrix(self.bc["f"]).T])

    #find the time scaling from the residual
    tf_gen = self.time_scale(residual, self.bc['tbounds'], Cval)

    #use shifted time if neccessary
    if self.bc['t_inital']:
      t_init = self.bc['t_inital']
      t_final = t_init + tf_gen
    else:
      t_init = 0.0
      t_final = tf_gen
    # print "tf_gen", tf_gen
    #generate the time matrix
    Time_mat = np.vstack([self.time_subs(t_init), self.time_subs(t_final)])
    #return both the trajectory coefficients and the final time
    self.initial_time = t_init
    self.final_time = t_final
    return np.linalg.pinv(Time_mat)*LH_side, t_final

  def plot_poly(self, coeff):
      
    def yplot(t,coeff):
      v = np.matrix([t**5, t**4, t**3, t**2, t, 1])*coeff
      v = v.item(0)
      return v

    t_vect = np.linspace(self.initial_time,self.final_time,100)
    t_vect = list(t_vect)

    # print "\n tvect", t_vect, type(t_vect)
    # print "coeff, ", coeff, type(coeff)
    yvals = [yplot(t,coeff) for t in t_vect]
    # print "yvals", yvals
    plt.figure(1)
    plt.subplot(211)
    plt.plot(t_vect, yvals)
    plt.ylabel('min jerk')
    # plt.show()


  def pouring_rate(self,coeff, Tf, Kf, tcurr):
    #from the ode: dq(t) = -(1/Tf)*q(t) + (Kf/Tf)*w(t)
    tmat= self.time_subs(tcurr)
    sys_state = tmat*coeff #with elements [m;q;dq]
    omega = (Tf/Kf)*(sys_state.item(1) + (1/Tf)*sys_state.item(2))
    return omega


  def time_scale(self,residual,t_bounds, Cval):
    tmin = t_bounds[0]
    tmax = t_bounds[1]
    C2 = (tmin + tmax*np.exp(-Cval*residual))/(tmax - tmin)
    C1 = tmin*(C2 + 1)
    tfinal = (C1)/(C2 + np.exp(-Cval*residual))
    if type(residual) != float:
      tfinal = tmax
    elif residual > 1000:
      tfinal = tmax #catch divergent case
    print "tmin:", tmin, " tmax:", tmax, " tfinal: ", tfinal, "residual", residual
    return tfinal




def main():
  print("main")

  bc = {"i":[], "f":[], 't_inital':None ,'tbounds':[]}
  #sig_cval

  bc["i"] = [0,0,0] #value and derivatives  h,dh,ddh initial versus final
  bc["f"] = [100,0,0]
  bc["tbounds"] = [5,15]


  min_jer_obj = MinJerk(bc)
  residual = 10 #define residual for 
  min_jerk_poly, t_final = min_jer_obj.min_jerk_poly(residual=residual, Cval=1.0)

  print "\n\n ***Problem Setup***\n BC: ",bc, "\n\n polynomial coeffs:", min_jerk_poly
  min_jer_obj.plot_poly(min_jerk_poly)

  Tf = 1.; Kf = 1.

  t_vect = list(np.linspace(0,t_final))
  pour_rate = [min_jer_obj.pouring_rate(min_jerk_poly, Tf, Kf, t) for t in t_vect]

  plt.subplot(212)
  plt.plot(t_vect, pour_rate)
  plt.ylabel('pour rate')
  plt.xlabel('time (s)')
  plt.show()



if __name__ == '__main__':
  main()



