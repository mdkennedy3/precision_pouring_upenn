import rospy
import numpy as np
import matplotlib.pyplot as plt #for plotting just like in matlab
import GPy  #pip install GPy

class GuassianProcessClass(object):
  def __init__(self,x_train=[],y_train=[], N_max=50, white_noise_var=1.0, lowest_model_rmse=1.0, model_rmse_to_white_noise=lambda x:100*x, param_method="nonparam"):
    self.N_max = N_max
    x_train = [x_train[idx] for idx in range(len(x_train)) if np.remainder(idx,np.floor(len(x_train)/self.N_max)) == 0.0]
    y_train = [y_train[idx] for idx in range(len(y_train)) if np.remainder(idx,np.floor(len(y_train)/self.N_max)) == 0.0]
    self.x_train = x_train
    self.y_train = y_train
    self.y_train_vect = np.matrix(y_train).T
    self.x_train_vect = np.matrix(x_train).T
    #optimize coeff
    if param_method in "semiparam":
      if lowest_model_rmse > 0.0:
        gp_var = 1/(model_rmse_to_white_noise(lowest_model_rmse)*100.)
      else:
        gp_var = 1.0
    else:
      gp_var = 1.0
    # gp_var = gp_var**2
    # print "\nGP vars: ",gp_var 
    gp_length_scale = 0.1
    kernel = GPy.kern.RBF(input_dim=1, variance=gp_var, lengthscale=gp_length_scale)
    #kernel = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=1.0) + GPy.kern.White(white_noise_var)
    model_gpy = GPy.models.GPRegression(np.matrix(x_train).T,self.y_train_vect,kernel)
    # model_gpy.optimize(messages=False,max_iters=1000,optimizer="lbfgs") #lbfgs was good but not realtime
    #self.lengthscale= model_gpy.kern.parameters[0].lengthscale.values[0]  #from rbf

    self.lengthscale =  model_gpy.kern.lengthscale.values[0]  # k = variance*exp(-(1/lengthscale**2)*(xi-xj)**2) 0.24999
    self.kernel_variance = model_gpy.kern.variance.values[0] # 15.42
    white_noise_var = 1.0 #1ml meas noise# white_noise_var*model_rmse_to_white_noise(lowest_model_rmse) #this amplifies the white noise when there are poor models
    new_kernel = GPy.kern.RBF(input_dim=1, variance=self.kernel_variance, lengthscale=self.lengthscale) + GPy.kern.White(input_dim=1, variance=white_noise_var)
    self.model_gpy = GPy.models.GPRegression(np.matrix(x_train).T,self.y_train_vect,new_kernel)
# yp,V=m.predict(x.transpose())
    self.L = self.obtain_training_L_matrix()

  def kernel(self,xi=[],xj=[],l=5e-1, alpha=1):
    K = np.matrix(np.zeros([len(xi),len(xj)]))
    # yp,V=m.predict(x.transpose())
    for idx in range(len(xi)):
      for jdx in range(len(xj)):
        K[idx,jdx] = self.kernel_variance*np.exp(-(1/self.lengthscale**2)*(xi[idx] - xj[jdx])**2)
    return K

  def obtain_training_L_matrix(self,sigma = 1e-1):
    K_train= self.kernel(xi=self.x_train, xj=self.x_train)
    L = np.linalg.inv(K_train + sigma**2*np.eye(np.max(K_train.shape)))
    return L

  def obtain_variance(self,x_test=[]):
    K_train_test = self.kernel(xi=self.x_train,xj= x_test)
    K_test_test = self.kernel(xi=x_test,xj=x_test)
    L = self.obtain_training_L_matrix()
    V = K_test_test - K_train_test.T *L*K_train_test
    return V



  def prediction(self,x_test=[],y_mean_test=None, y_mean_train=None):
    #obtain test specific kernels
    if y_mean_train: y_mean_train = [y_mean_train[idx] for idx in range(len(y_mean_train)) if np.remainder(idx,np.floor(len(y_mean_train)/self.N_max)) == 0.0]
    
    K_train_test = self.kernel(xi=self.x_train,xj=x_test)
    K_test_test = self.kernel(xi=x_test,xj=x_test)
    if type(y_mean_train) == type(None):
      y_pred = K_train_test.T*self.L*self.y_train_vect
    else:
      y_mean_train_vect= np.matrix(y_mean_train).T
      if type(y_mean_test) == type(None):
        y_pred = K_train_test.T*self.L*(self.y_train_vect - y_mean_train_vect)
        err = self.y_train_vect - y_mean_train_vect
        #print "\n\nerr", err, "\ntrain vect", self.y_train_vect, "\nmean vect", y_mean_train_vect
      else:
        y_mean_test_vect = np.matrix(y_mean_test).T
        y_pred = y_mean_test_vect + K_train_test.T*self.L*(self.y_train_vect - y_mean_train_vect)
    y_return = y_pred.T.tolist()[0]
    return y_return

def main():
  x = np.linspace(0,2.4,2e3)
  #y = 2*np.sin(2*x) +x
  y = 10*x**2 + x
  xtest = np.linspace(0,3.5,100)

  # def func(x): return [xcurr**2 + 0.1*(np.random.rand()-0.5) for xcurr in x]
  def func(x): return [ np.sin(xcurr)*0.0 + xcurr  for xcurr in x]


  y_mean_train = func(x.tolist()) 
  y_mean_test = func(xtest.tolist()) 
  #Train:
  cls_obj = GuassianProcessClass(x_train=x.tolist(),y_train=y.tolist())
  #Predict
  # yp = cls_obj.prediction(x_test=xtest.tolist(), y_mean_train= y_mean_train, y_mean_test=y_mean_test)
  # yp = cls_obj.prediction(x_test=xtest.tolist(), y_mean_train= y_mean_train)
  yp = cls_obj.prediction(x_test=xtest.tolist())
  #Obtain variance
  V = cls_obj.obtain_variance(x_test=xtest.tolist())

  #trying with GPy library
  # kernel = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=1.0)
  # m = GPy.models.GPRegression(np.matrix(x).T,np.matrix(y).T,kernel)
  # m.optimize(messages=False)
  # yp,V=m.predict(x.transpose())
  # m.kern.lengthscale.values[0]  # k = variance*exp(-(1/lengthscale**2)*(xi-xj)**2)
  # m.kern.variance.values[0]

  #Plot
  fig = plt.figure()
  ax = fig.gca()
  #Plot the training data
  ax.plot(x.tolist(), y.tolist(),label="training data")
  ax.scatter(x.tolist(), y.tolist(),label="training data scatter")
  #Plot the mean test data
  ax.scatter(xtest.tolist(),yp,label="test data mean")
  #Plot the associated covariance with the test data, realize that the associated covariance with each point is the diagonal elements of V[f]
  Vtt = [V[idx,idx] for idx in range(V.shape[0])]
  upper = [yp[idx]+Vtt[idx] for idx in range(len(yp))]
  lower = [yp[idx]-Vtt[idx] for idx in range(len(yp))]
  # print "vtt", Vtt
  test_data_mean_cov = ax.fill_between(xtest.tolist(),lower,upper, alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
  ax.legend()
  plt.show()
  

if __name__ == '__main__':
  main()
