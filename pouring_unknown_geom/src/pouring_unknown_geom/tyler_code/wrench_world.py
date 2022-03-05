#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

#whenever importing from your library, always make last import the function.py from which you can access its classes#e.g. don't do from pouring_unknown_geom import ts_math_operations as you won't be able to access ts_math_oper
#but what is allowed is import pouring_unknown_geom.ts_math_operations.ts_math_oper as fnct to get it directly
from pouring_unknown_geom.ts_math_operations import ts_math_oper #ts_math_oper_fncts. *TF_btw_frames, *.tf_to_mat, *transform_from_pose (multiply by vector: [])

#IN terminal that this is run in (or in the launch file)
# export ROS_NAMESPACE=/iiwa


class wrench_listener(object):
    def __init__(self):
        self.ts_math_oper_fncts = ts_math_oper.ts_math_oper_fncts()

    def transform_vector_wrist_to_world(self,vect):
        #necessary for forces, torques to put in world/base frame
        vect = np.matrix(vect)
        if vect.shape[0] != 3:
            vect = vect.T
        TF = self.ts_math_oper_fncts.TF_btw_frames('/iiwa_link_ee','/world')
        vect_in_world =  self.ts_math_oper_fncts.transform_vector(TF,vect)
        return vect_in_world


    def TF_wrench_wrist_to_world(self,wrench):
        print "get the wrenches at the wrist"
        #call the wrench msgs
        Fvect = wrench[:3,0]
        Tvect = wrench[3:,0]
        Fvect_world = self.transform_vector_wrist_to_world(Fvect)
        Tvect_world  = self.transform_vector_wrist_to_world(Tvect)
        wrench = np.vstack([Fvect_world, Tvect_world])
        return wrench

    def wrench_callback(self,call):
        # print "fx: ", call.wrench.force.x, "fy: ", call.wrench.force.y, "fz", call.wrench.force.z
        # print "torque_x: ", call.wrench.torque.x, "ty:",call.wrench.torque.y, " tz: ",call.wrench.torque.z
        wrench_vect_wrist = np.matrix([call.wrench.force.x,call.wrench.force.y,call.wrench.force.z,call.wrench.torque.x,call.wrench.torque.y,call.wrench.torque.z ]).T
        wrench_vect_world = self.TF_wrench_wrist_to_world(wrench_vect_wrist)
        print "\n\nwrench in base frame: \n fx: ",wrench_vect_world.item(0), "fy: ",wrench_vect_world.item(1), "fz: ",wrench_vect_world.item(2),"tx: ",wrench_vect_world.item(3), "ty: ",wrench_vect_world.item(4), "tz: ",wrench_vect_world.item(5)


def main():
    rospy.init_node('world_wrench')


    cls_wrench = wrench_listener()
    val = rospy.Subscriber('/iiwa/state/CartesianWrench', WrenchStamped,cls_wrench.wrench_callback)
    rospy.spin()



if __name__ == '__main__':
    main()

