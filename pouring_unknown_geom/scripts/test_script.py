
#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

class Test(object):
  """docstring for Test"""
  def circ_intersect_func(self,xb=None,yb=None,rb=None,Lb=None, pour_dir="left"):
    #xb,yb are beaker location in world frame, rb is beaker radius, and Lb is distance between world frame and beaker origin

    def query_point(x=None,y=None,xb=None,yb=None, rb=None,Lb=None):
      v1 = (x-xb)**2 + (y-yb)**2 - rb**2
      v2 = x**2 + y**2 - Lb**2
      return v1+v2

    def skew_mat(v):
      return np.matrix([[0,-v.item(2),v.item(1)],[v.item(2),0,-v.item(0)],[-v.item(1),v.item(0),0]])

    if np.abs(xb) > 0.0:
      A = (1+(yb/xb)**2)
      B = (yb/(xb**2))*(rb**2-Lb**2-(xb**2+yb**2))
      C =  (1/(4*xb**2))*(rb**2-Lb**2 -(xb**2+yb**2))**2 - Lb**2
      y1 = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
      y2 = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)

      print "y1**2 and y2**2", y1**2, y2**2
      x_arg = Lb**2 - y1**2
      x1 = np.sqrt(x_arg)
      x2 = -np.sqrt(x_arg)
      print "case xb>0.0"
      print "y_arg", y1**2
      print "x_arg", x_arg

    elif np.abs(yb) > 0.0:
      A = (1+(xb/yb)**2)
      B = (xb/(yb**2))*(rb**2-Lb**2-(xb**2+yb**2))
      C =  (1/(4*yb**2))*(rb**2-Lb**2 -(xb**2+yb**2))**2 - Lb**2
      x1 = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
      x2 = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)
      y_arg = Lb**2 - x1**2
      y1 = np.sqrt(y_arg)
      y2 = -np.sqrt(y_arg)
      print "case xb<0.0"
      print "y_arg", y_arg
      print "x_arg", x1**2
    else:
      print "beaker is at the world origin which is not allowed"

    combos = [[x1,y1],[x1,y2],[x2,y1],[x2,y2]]
    combo_scores = []
    for idx in range(len(combos)):
      score = query_point(x=combos[idx][0], y=combos[idx][1], xb=xb,yb=yb, rb=rb, Lb=Lb)
      combo_scores.append(score)
    combo_sort = np.argsort(combo_scores)
    final_set = np.array(combos)[combo_sort[:2]] #first two sorted are smallest
    #Figure out which is on the left and right of the beaker

    diff_vect1 = np.matrix([final_set[0][0] -xb, final_set[0][1] -yb, 0.0]).T
    diff_vect1_normed = np.divide(diff_vect1,np.linalg.norm(diff_vect1))

    diff_vect2 = np.matrix([final_set[1][0] -xb, final_set[1][1] -yb, 0.0]).T
    diff_vect2_normed = np.divide(diff_vect2,np.linalg.norm(diff_vect2))

    beaker_vect = np.matrix([xb,yb,0.0]).T
    beaker_vect_normed= np.divide(beaker_vect, np.linalg.norm(beaker_vect))

    beaker_skew_mat = skew_mat(beaker_vect_normed)

    cross_vect1 = beaker_skew_mat*diff_vect1_normed
    cross_vect2 = beaker_skew_mat*diff_vect2_normed

    left_pt = None
    if cross_vect1.item(2) > 0:
      left_pt = final_set[0]
      right_pt = final_set[1]
    else:
      left_pt = final_set[1]
      right_pt = final_set[0]

    #if pouring from the left pass the left, else pass the one on the right
    if pour_dir in "left":
      #choose the one on the left of the beaker line
      return left_pt
    else:
      return right_pt
      #choose the one on the right of the beaker line

def main():
  cls_obj = Test()

  xb = 2.0; yb = 0.0
  xb = 0.0; yb = 2.0
  Lb = 2.0
  rb= 1.0


  #pt = cls_obj.circ_intersect_func(xb=xb,yb=yb,rb=rb,Lb=Lb, pour_dir="left")  
  pt = cls_obj.circ_intersect_func(xb=xb,yb=yb,rb=rb,Lb=Lb, pour_dir="right")  
  print "point: ", pt, type(pt)
  print "point components", pt[0], "and", pt[1]

  #plot the point
  plt.scatter(0,0)
  plt.scatter(xb,yb)

  an = np.linspace(0, 2*np.pi, 100)
  plt.plot(xb+np.cos(an), yb+np.sin(an))

  plt.scatter(pt[0],pt[1])
  plt.axis('equal')
  plt.show()


if __name__ == '__main__':
  main()
