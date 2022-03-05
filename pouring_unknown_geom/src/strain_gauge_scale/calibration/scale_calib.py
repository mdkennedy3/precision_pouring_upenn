import numpy as np

def main():

  real = [28,50,70,102,124, 152, 178,202, 224, 260, 300 ]

  arduino = [150,265,378,550,672, 820,960,1090, 1216, 1407, 1633 ]


  y = np.matrix(real).T
  x = np.matrix(arduino).T

  M = np.hstack([np.power(x,3),np.power(x,2),x,np.ones([len(arduino),1])])

  coef = np.linalg.pinv(M)*y
  print "\ncoef", coef

  err = y - M*coef
  rmse = np.sqrt(np.mean(np.power(err,2)))
  print "\nerr", err, "\n\nrmse", rmse



if __name__=='__main__':
  main()
