from pouring_control_pkg.pouring_control import pouring_model_control_cls
import math
scale_or_vision = 'scale'
final_mh = 100 #ml
final_mh_time = 10 #seconds
pour_obj = pouring_model_control_cls.Pouring_Calculations_Class(scale_or_vision, final_mh, final_mh_time)
pour_obj.calc_meas_pram_min_jerk_traj('init')
# dt, theta, h, dh = 0., 0., 0.,0.01
dt, theta, h, dh = 0.1, 5.*math.pi/180., 0.,1.
w = pour_obj.calc_omega(dt, theta, h, dh)
print "output w: ", w
