import numpy as np
import matplotlib.pyplot as plt

scaling_constant_low_pass_filt_value=0.50
gamma_t = {'gamma_t':None, 'a':scaling_constant_low_pass_filt_value,
                'gamma_t_prev':None,
                'update_gamma': lambda g_upd,g_old,a: (a)*g_upd + (1-a)*g_old,
                'gamma_new': lambda dV,th,poly_deg: np.linalg.norm(np.polynomial.polynomial.Polynomial(np.polynomial.polynomial.polyfit(dV,th,poly_deg), domain=[th[0],th[-1]], window=[th[0],th[-1]]).coef)}  #this is the scaling function
x =np.linspace(0,np.pi,1000); y = x**2; yn = 1.0*(np.random.random_sample(len(y),)-0.5); y = y+yn
gamma_list = []
rtype = type(None)
x_print = []
for idx in range(1,len(y)):
  gamma_curr = gamma_t['gamma_new'](x[:idx],y[:idx],5)
  # if type(gamma_t['gamma_t_prev']) == rtype and idx<20:
  if type(gamma_t['gamma_t_prev']) == rtype or x[idx]<0.5:
    gamma_t['gamma_t_prev'] = gamma_curr
  else:
    gamma_t['gamma_t'] =gamma_t['update_gamma'](gamma_curr, gamma_t['gamma_t_prev'], gamma_t['a'])
    gamma_t['gamma_t_prev'] = gamma_t['gamma_t']
    gamma_list.append(gamma_t['gamma_t'])
    x_print.append(x[idx])

plt.plot(x_print, gamma_list)
plt.show()
