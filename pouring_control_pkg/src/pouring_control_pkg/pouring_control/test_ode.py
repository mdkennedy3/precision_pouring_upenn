from scipy.integrate import ode
import math, numpy as np
import matplotlib.pyplot as plt


y0,t0 = [math.pi/2, 0],0
#for pendulum
def f(t,y, args):
  l = args[0]
  g = args[1]
  b = args[2]
  dy1 = y[1]
  dy2 = -(g*np.sin(y[0])+b*y[1])/l
  return [dy1,dy2]


r = ode(f).set_integrator('zvode', method='bdf', with_jacobian=False)
f_params = [1,9.8,0.3] #args being passed
r.set_initial_value(y0, t0).set_f_params(f_params)

tf = 30
dt = 0.01
t_list = []
x1_list = []
x2_list = []
while r.successful() and r.t < tf:
  r.integrate(r.t+dt)
  # print "r.t", r.t, "r.y", r.y
  t_list.append(r.t)
  x1_list.append(np.real(r.y[0]).item(0))
  x2_list.append(np.real(r.y[1]).item(0))

  #print("%g %g"%(r.t, r.y))

plt.plot(t_list, x1_list)
plt.plot(t_list, x2_list)
plt.show()
