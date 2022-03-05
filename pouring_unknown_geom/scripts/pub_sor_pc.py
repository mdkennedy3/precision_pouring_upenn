#!/usr/bin/env python
import rospy
import numpy as np
import rospkg
import math
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64

from numpy import linalg as LA


import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

class PubSor(object):

  def __init__(self):

    self.type = 1 # 1 - cylinder, 2 - SOR cosine
    self.container_height = 1.5 #in m
    self.container_inner_r = 1.5 #in m
    self.height_count = 50
    self.r_count = 30
    self.angle_count = 100
    self.amplitude = 1.2
    #self.amplitude = 0.3

    self.rotation_ang = 10*math.pi/180 #deg
    self.Rx = np.matrix([[1, 0, 0], [0, math.cos(self.rotation_ang), -math.sin(self.rotation_ang)], [0, math.sin(self.rotation_ang), math.cos(self.rotation_ang)]])
    print self.Rx
    self.pc = PointCloud()
    self.pc_in = PointCloud()
    self.pc_out = PointCloud()
    self.pc_pub = rospy.Publisher('/sor_pc',PointCloud,queue_size=10)
    self.pc_pub_in = rospy.Publisher('/sor_pc_in',PointCloud,queue_size=10)
    self.pc_pub_out = rospy.Publisher('/sor_pc_out',PointCloud,queue_size=10)
    #self.ang_sub = rospy.Subscriber("sor_ang", Float64, self.ang_callback, queue_size = 1)

    rospack = rospkg.RosPack()
    self.directory_path = rospack.get_path("pouring_unknown_geom") + '/figs/'

  def gen_plot_sor1(self):

    th = []
    dV = []
    dV2 = []

    cnt = 1
    ag = 120;
    ag = 50;
    #for theta1 in [ag*np.pi/180]:
    for theta1 in np.linspace(0,np.pi, 100):

      self.pc = PointCloud()
      self.pc_in = PointCloud()
      self.pc_out = PointCloud()

      self.rotation_ang = theta1
      self.Rx = np.matrix([[1, 0, 0], [0, math.cos(self.rotation_ang), -math.sin(self.rotation_ang)], [0, math.sin(self.rotation_ang), math.cos(self.rotation_ang)]])

      self.pub_sor()

      th.append(theta1)

      points_out = len(self.pc_out.points)
      points_in = len(self.pc_in.points)
      tot_points = float(points_out + points_in)
      rat = float(points_out)/tot_points
      dV.append(rat)

      print theta1, points_out, points_in , rat

      #Get Plots for CYL
      self.pc = PointCloud()
      self.pc_in = PointCloud()
      self.pc_out = PointCloud()

      self.rotation_ang = theta1
      self.Rx = np.matrix([[1, 0, 0], [0, math.cos(self.rotation_ang), -math.sin(self.rotation_ang)], [0, math.sin(self.rotation_ang), math.cos(self.rotation_ang)]])

      self.pub_cyl()

      points_out = len(self.pc_out.points)
      points_in = len(self.pc_in.points)
      tot_points = float(points_out + points_in)
      rat = float(points_out)/tot_points
      dV2.append(rat)

      print theta1, points_out, points_in , rat

    print th
    print dV
    fig1 = plt.figure(figsize=(20, 6), dpi=80, facecolor='w', edgecolor='k')
    ax1 = fig1.gca()
    ax1.scatter(th,dV,label='dV',linewidth=5.0)
    ax1.plot(th,dV,label='dV',linewidth=1.0)

    ax1.scatter(th,dV2,label='dV2',linewidth=5.0)
    ax1.plot(th,dV2,label='dV2',linewidth=1.0)

    ax1.set_xlabel('$\\theta$ (rad)', fontsize=18)
    ax1.set_xlim(0,th[-1])#np.pi)
    ax1.set_ylim(0,1.5)
    ax1.set_ylabel('$dV$ (ml)', fontsize=18)

    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

    fig_name= self.directory_path + 'sor1.png'
    plt.savefig(fig_name)
    plt.show()

  def pub_sor(self):

    self.pc = PointCloud()
    self.pc_in = PointCloud()
    self.pc_out = PointCloud()

    self.gen_sor1()

    self.pc.header.stamp = rospy.Time.now()
    self.pc.header.frame_id = 'map'

    self.pc_pub.publish(self.pc)

    self.pc_in.header.stamp = rospy.Time.now()
    self.pc_in.header.frame_id = 'map'
    self.pc_pub_in.publish(self.pc_in)

    self.pc_out.header.stamp = rospy.Time.now()
    self.pc_out.header.frame_id = 'map'
    self.pc_pub_out.publish(self.pc_out)

  def pub_cyl(self):

    self.pc = PointCloud()
    self.pc_in = PointCloud()
    self.pc_out = PointCloud()

    self.gen_cyl()

    self.pc.header.stamp = rospy.Time.now()
    self.pc.header.frame_id = 'map'

    self.pc_pub.publish(self.pc)

    self.pc_in.header.stamp = rospy.Time.now()
    self.pc_in.header.frame_id = 'map'
    self.pc_pub_in.publish(self.pc_in)

    self.pc_out.header.stamp = rospy.Time.now()
    self.pc_out.header.frame_id = 'map'
    self.pc_pub_out.publish(self.pc_out)

  def gen_cyl(self):
    for h in np.linspace(0, self.container_height, self.height_count):
      r_max = self.container_inner_r + 0
      for r in np.linspace(0, r_max, self.r_count):
        for theta in np.linspace(0, 2*np.pi, self.angle_count):
          self.pc.points.append(Point32(r * math.cos(theta), r * math.sin(theta), h))

          rotated_points = self.Rx*np.matrix([r * math.cos(theta), r * math.sin(theta), h]).transpose()

          z_curr = rotated_points.item(2)
          if (z_curr >= self.rotated_tip_point[2]):
            self.pc_out.points.append(Point32(rotated_points.item(0), rotated_points.item(1), rotated_points.item(2)))
          else:
            self.pc_in.points.append(Point32(rotated_points.item(0), rotated_points.item(1), rotated_points.item(2)))

      points_out = len(self.pc_out.points)
      points_in = len(self.pc_in.points)
      tot_points = float(points_out + points_in)
      rat = float(points_out)/tot_points

  def gen_sor1(self):

    print self.Rx

    h_theta = np.linspace(0, 2*math.pi, self.height_count)

    #Point at the sor tip along y axis
    self.tip_point = np.matrix([0, -(self.container_inner_r+self.amplitude*math.sin(h_theta[self.height_count-1])), self.container_height]).transpose()

    V_vect = np.matrix([0, 0, 1])#vector in z dir

    self.rotated_tip_point = self.Rx*self.tip_point #sor tip rotated

    plane_d = -V_vect*self.rotated_tip_point
    plane_eq = np.hstack([V_vect, plane_d])
    #print 'planes ', plane_d, plane_eq

    curve_points = np.matrix([0, 0, 0])
    curve_tangents = np.matrix([0, 0, 0])
    rotated_curve_points = np.matrix([0, 0, 0])
    rotated_tangents = np.matrix([0,0,0])
    dot_vals = [0]
    cnt = 0
    for h in np.linspace(0, self.container_height, self.height_count):

      #Curve points in yz plane
      cp = np.matrix([0, -(self.container_inner_r+self.amplitude*math.sin(h_theta[cnt])), h])

      #Cuve tangents in yz plane
      ct = np.matrix([0, -self.amplitude*math.pi*2*math.cos(h_theta[cnt])/self.container_height, h])

      #append
      curve_points = np.vstack([curve_points, cp])
      curve_tangents = np.vstack([curve_tangents, ct])

      rt = self.Rx*ct.transpose()
      rt_norm = LA.norm(rt)
      rt = rt/rt_norm
      rotated_tangents = np.vstack([rotated_tangents, rt.transpose()])

      dot_vals.append(np.dot(V_vect, rt).item(0))

      rp = self.Rx*cp.transpose()
      rotated_curve_points = np.vstack([rotated_curve_points, rp.transpose()])
      self.pc.points.append(Point32(rp.item(0), rp.item(1), rp.item(2)))

      cnt = cnt + 1

    dot_vals.pop(0)
    curve_points = np.delete(curve_points, (0), axis=0)
    curve_tangents = np.delete(curve_tangents, (0), axis=0)
    rotated_tangents = np.delete(rotated_tangents, (0), axis=0)
    rotated_curve_points = np.delete(rotated_curve_points, (0), axis=0)

    print 'sfsf---------'
    #print curve_points
    #print curve_tangents
    #print rotated_tangents
    print plane_eq
    print 'dot vals--------'
    print  dot_vals
    print '-------------'

    angs = [math.acos(dot_vals[idx])*180.0/math.pi for idx in range(len(dot_vals))]
    print angs

    zero_sl_index = [] #sl - slope
    zero_sl_score = []
    eps = 0.9

    print len(rotated_curve_points), len(rotated_curve_points)/2.0
    index = int((len(rotated_curve_points)-1)/2.0)
    print index
    flag = False
    for ind in range(index,len(rotated_curve_points)-1):
      prod = dot_vals[ind]
      rcp = rotated_curve_points[index,:].tolist()[0]
      rcp.append(1)
      rcp_np = np.matrix(rcp)
      score = plane_eq*rcp_np.transpose()

      if score > 0.0:
        flag = True

      if abs(prod) < eps:
        if score > 0.0:
          zero_sl_index.append(index)
          zero_sl_score.append(score.tolist()[0][0])

      index = index +1

    print 'scores'
    print zero_sl_score
    print zero_sl_index

    #zero_sl_score_abs = [abs(zero_sl_score[idx]) for idx in range(len(zero_sl_score))]
    #print zero_sl_score_abs

    if flag == True:
      best_index = zero_sl_score.index(max(zero_sl_score))
      best_score = zero_sl_score[best_index]
      print 'best ',  best_index, best_score
      h_star_index = zero_sl_index[best_index]
      h_star = rotated_curve_points[h_star_index,:].tolist()[0]
      h_star_np = np.matrix(h_star)

      self.pc.points.append(Point32(h_star_np.item(0)+0.5, h_star_np.item(1), h_star_np.item(2)))

      print '@@@@@ ', h_star_np

      V_hor = np.matrix([0, 0, 1])#vector in z dir
      V_hor_rot = (self.Rx*V_hor.transpose()).transpose()

      plane_d_hstar = -V_hor_rot*h_star_np.transpose()
      plane_eq_hstar = np.hstack([V_hor_rot, plane_d_hstar])

      V_vect2 = np.matrix([0, 0, 1])#vector in z dir

      plane_d = -V_vect2*h_star_np.transpose()
      plane_eq = np.hstack([V_vect2, plane_d])

      print '****** ', plane_eq_hstar

      cnt = 0
      for h in np.linspace(0, self.container_height, self.height_count):
        r_max = self.container_inner_r + self.amplitude*math.sin(h_theta[cnt])
        cnt = cnt + 1

        for r in np.linspace(0, r_max, self.r_count):
          for theta in np.linspace(0, 2*np.pi, self.angle_count):

            rotated_point = (self.Rx*np.matrix([r * math.cos(theta), r * math.sin(theta), h]).transpose()).transpose()

            rotated_pt = rotated_point.tolist()[0]
            rotated_pt.append(1)

            rotated_point = np.matrix(rotated_pt)

            score_hstar = plane_eq_hstar*rotated_point.transpose()
            score = plane_eq*rotated_point.transpose()

            #print score_hstar, score
            if (score > 0.0):
              self.pc_out.points.append(Point32(rotated_point.item(0), rotated_point.item(1), rotated_point.item(2)))
              continue
            if (score_hstar > 0.0):
              self.pc_out.points.append(Point32(rotated_point.item(0), rotated_point.item(1), rotated_point.item(2)))
              continue
            else:
              self.pc_in.points.append(Point32(rotated_point.item(0), rotated_point.item(1), rotated_point.item(2)))
            #z_curr = rotated_points.item(2)
            #if (z_curr >= self.rotated_tip_point[2]):
            #  self.pc_out.points.append(Point32(rotated_point.item(0), rotated_point.item(1), rotated_point.item(2)))
            #else:
            #  self.pc_in.points.append(Point32(rotated_point.item(0), rotated_point.item(1), rotated_point.item(2)))

      points_out = len(self.pc_out.points)
      points_in = len(self.pc_in.points)
      tot_points = float(points_out + points_in)
      rat = float(points_out)/tot_points

      print self.rotation_ang, points_out, points_in , rat

    else:
      print 'sor not upside down'

      cnt = 0
      for h in np.linspace(0, self.container_height, self.height_count):
        r_max = self.container_inner_r + self.amplitude*math.sin(h_theta[cnt])
        cnt = cnt + 1

        for r in np.linspace(0, r_max, self.r_count):
          for theta in np.linspace(0, 2*np.pi, self.angle_count):

            rotated_points = self.Rx*np.matrix([r * math.cos(theta), r * math.sin(theta), h]).transpose()

            z_curr = rotated_points.item(2)
            if (z_curr >= self.rotated_tip_point[2]):
              self.pc_out.points.append(Point32(rotated_points.item(0), rotated_points.item(1), rotated_points.item(2)))
            else:
              self.pc_in.points.append(Point32(rotated_points.item(0), rotated_points.item(1), rotated_points.item(2)))

      points_out = len(self.pc_out.points)
      points_in = len(self.pc_in.points)
      tot_points = float(points_out + points_in)
      rat = float(points_out)/tot_points

      print self.rotation_ang, points_out, points_in , rat


  def ang_callback(self, msg):
    print 'received ', msg.data
    self.rotation_ang = msg.data*math.pi/180 #deg
    self.Rx = np.matrix([[1, 0, 0], [0, math.cos(self.rotation_ang), -math.sin(self.rotation_ang)], [0, math.sin(self.rotation_ang), math.cos(self.rotation_ang)]])
    print self.Rx

def main():

  rospy.init_node('pub_sor_pc')
  pub_sor = PubSor()

  pub_sor.gen_plot_sor1()

  r = rospy.Rate(2)
  #while not rospy.is_shutdown():
    #pub_sor.pub()
    #rospy.spin()
    #r.sleep()

if __name__ == '__main__':
  main()