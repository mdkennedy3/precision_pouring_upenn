import numpy as np
import matplotlib.pyplot as plt



class MinJerk(object):
    """docstring for MinJerk"""
    def __init__(self, bc):
        self.bc = bc

    def time_subs(self,t):
        mat = np.matrix([[t**5, t**4, t**3, t**2, t, 1],
                         [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],
                         [20*t**3, 12*t**2, 6*t, 2, 0, 0]])
        return mat

    def min_jerk_poly(self):
        LH_side = np.vstack([np.matrix(self.bc["i"]).T, np.matrix(self.bc["f"]).T])
        t_list = []
        # for idx in range(len(self.bc[]))
        Time_mat = np.vstack([self.time_subs(self.bc["t"][0]), self.time_subs(self.bc["t"][1])])
        return np.linalg.pinv(Time_mat)*LH_side

    def plot_poly(self, coeff):
        
        def yplot(t,coeff):
            v = np.matrix([t**5, t**4, t**3, t**2, t, 1])*coeff
            v = v.item(0)
            return v

        t_vect = np.linspace(self.bc["t"][0],self.bc["t"][1],100)
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


    def pouring_rate(self,coeff, Q1,Q2,Q3, Af, tcurr):
        #from the ode: dq(t) = -(1/Tf)*q(t) + (Kf/Tf)*w(t)
        tmat= self.time_subs(tcurr)
        sys_state = tmat*coeff #with elements [m;dm;ddm]
        hf = sys_state.item(0)
        hf_dot = sys_state.item(1)
        hf_ddot = sys_state.item(2)
        omega = (Q2*Af*hf_dot + Q3*Af**(1./3.)*hf_dot**(1./3.))**(-1) * (Af*hf_ddot - Q1*Af**(4./3.)*hf_dot**(4./3.))
        return omega


    def system_state(self,coeff, tcurr):
        #from the ode: dq(t) = -(1/Tf)*q(t) + (Kf/Tf)*w(t)
        tmat= self.time_subs(tcurr)
        sys_state = tmat*coeff #with elements [m;q;dq]
        return sys_state







def main():
   print("main")
   bc = {"i":[], "f":[], 't':[]}
   t_end = 10
   bc["i"] = [0,0,0]
   bc["f"] = [1,0,0]
   bc["t"] = [0,t_end]

   min_jer_obj = MinJerk(bc)
   min_jerk_poly = min_jer_obj.min_jerk_poly()
   print "\n\n ***Problem Setup***\n BC: ",bc, "\n\n polynomial coeffs:", min_jerk_poly
   min_jer_obj.plot_poly(min_jerk_poly)

   Tf = 1.; Kf = 1.

   t_vect = list(np.linspace(bc["t"][0],bc["t"][1],100))
   #these terms actually change with angle, hence ode is required
   Q1, Q2, Q3, Af = 1,1,1,1
   pour_rate = [min_jer_obj.pouring_rate(min_jerk_poly, Q1,Q2,Q3, Af, t) for t in t_vect]

   plt.subplot(212)
   plt.plot(t_vect, pour_rate)
   plt.ylabel('pour rate')
   plt.xlabel('time (s)')
   plt.show()



if __name__ == '__main__':
    main()



