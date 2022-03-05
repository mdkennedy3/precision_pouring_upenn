#!/usr/bin/env python

import rospy, rospkg
import numpy as np, math
from pouring_msgs.srv import *
import GPy

class GPEdgeProfileGeneratorClass(object):

  def gp_calculator(self,x_train=[],y_train=[],x_test=[]):
    #kernel = GPy.kern.RBF(input_dim=1, variance=10.0, lengthscale=0.03) + GPy.kern.White(input_dim=1, variance=0.03)
    #kernel = GPy.kern.RBF(input_dim=1, variance=10.0, lengthscale=0.03) + GPy.kern.White(input_dim=1, variance=0.01)
    #kernel = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=0.014) + GPy.kern.White(input_dim=1, variance=0.03)

    #kernel = GPy.kern.RBF(input_dim=1, variance=5.0, lengthscale=0.03) + GPy.kern.White(input_dim=1, variance=0.01)
    #kernel = GPy.kern.RBF(input_dim=1, variance=2.0, lengthscale=0.15) + GPy.kern.White(input_dim=1, variance=0.01)
    #kernel = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=0.02) + GPy.kern.White(input_dim=1, variance=0.01)
    kernel = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=0.03) + GPy.kern.White(input_dim=1, variance=0.01)
    y_train_mean = np.mean(y_train)
    y_train_shift = [y - y_train_mean for y in y_train] #get the mean radius
    model_gpy = GPy.models.GPRegression(np.matrix(x_train).T,np.matrix(y_train_shift).T,kernel)
    #model_gpy = GPy.models.GPRegression(np.matrix(x_train).T,np.matrix(y_train).T,kernel)
    #model_gpy.optimize(messages=False)
    print "\n\n\n\nnew length scale: ", model_gpy.kern.parameters[0][1], "\nnew var: ", model_gpy.kern.parameters[0][0]
    xtest_array = np.array(x_test)
    xtest_array = xtest_array.reshape(len(x_test),1)
    yp,Var = model_gpy.predict(xtest_array)
    yp_list = yp.transpose().tolist()[0]
    yp_list = [y + y_train_mean for y in yp_list]
    return yp_list

  def GP_edge_profile_callback(self,req):
    h_train = req.h_train
    r_train = req.rad_train
    h_test = req.h_test
    r_pred = self.gp_calculator(x_train=h_train, y_train=r_train, x_test=h_test)
    resp = GPEdgeProfileGeneratorResponse()
    resp.rad_pred = r_pred
    return resp





def main():
  rospy.init_node('gp_edge_profile_serv');
  cls_obj = GPEdgeProfileGeneratorClass();
  serv_pub = rospy.Service("~gp_edge_profile_gen_serv", GPEdgeProfileGenerator, cls_obj.GP_edge_profile_callback);

  rospy.spin()



if __name__ == '__main__':
  main()
