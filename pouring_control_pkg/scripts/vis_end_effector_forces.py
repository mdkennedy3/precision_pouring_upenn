#!/usr/bin/env python2
import numpy as np, math
import rospy
from intera_core_msgs.msg import EndpointState  #to publish types of joint commands
#visualize force vectors
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point#[] points
class Visualize_Sawyer_Wrench(object):
    """docstring for Visualize_Sawyer_Wrench"""
    def __init__(self):
        self.force_vis_pub = rospy.Publisher('/force_vis', Marker ,queue_size=1, latch=True)    
        self.force_scale = 1.0# 0.5

    def wrench_callback(self,req):
        self.time = req.header.stamp.secs


        self.pose = req.pose #(position,orientation).(x,y,z,(w))
        self.twist = req.twist #(linear,angular).(x,y,z)
        self.force = req.wrench.force #(x,y,z)
        self.torque = req.wrench.torque #(x,y,z)

        #now visualize
        self.plot_forces()


    def plot_forces(self):
        m = Marker()
        m.type = 0
        m.header.frame_id = "base"
        m.header.stamp = rospy.Time.now()
        m.id = 1 #would iterate with more

        pt1 = Point(); pt2 = Point()
        pt1.x = self.pose.position.x; pt1.y = self.pose.position.y; pt1.z = self.pose.position.z

        pt2.x = self.pose.position.x + self.force.x*self.force_scale;  pt2.y = self.pose.position.y + self.force.y*self.force_scale;  pt2.z = self.pose.position.z + self.force.z*self.force_scale

        m.points = [pt1, pt2]

        m.scale.x = 0.2
        m.scale.y = 0.1
        m.scale.z = 0.1

        m.color.a = 1
        m.color.r = 1
        m.color.b = 0
        m.color.g = 0

        self.force_vis_pub.publish(m)









def main():
    rospy.init_node('visualize_end_forces')


    vis_obj = Visualize_Sawyer_Wrench()

    end_state_sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, vis_obj.wrench_callback)


    rospy.spin()

if __name__ == '__main__':
    main()
