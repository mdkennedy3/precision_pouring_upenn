#!/usr/bin/env python2

#Author: Monroe Kennedy III: kmonroe@seas.upenn.edu
#Date: 12/5/15
#Copyright: All rights reserved, please contact author before any usage. 
#University of Pennsylvania

import numpy as np
import math
import rospy
import tf


class tf_pose_object(object):
    def __init__(self):
        self.trans = []
        self.rot = []

class ts_math_oper_fncts(object):
    def __init__(self):
        self.tf_listener = tf.TransformListener()

    def TF_frame_to_base(self, end_pose):
        #this expects the desired end frame /end_frame and provides the translation, rotation from base to this frame
        tag_pose = tf_pose_object()
        try: 
            #(trans_H,rot_H) = self.tf_listener.lookupTransform('/base',self.end_frame,rospy.Time(0))
            (tag_pose.trans, tag_pose.rot) = self.tf_listener.lookupTransform('/base',end_pose,rospy.Time(0))
            #note that rot form is q= [q1,q2,q3,qw]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            tag_pose.trans = []
            tag_pose.rot = []
        return tag_pose

    def TF_btw_frames(self,start_tf,end_tf):
        #this script finds the transform of start_tf in end_tf frame or is the point transform from end to start tf (in end frame)
        transform_obj = tf_pose_object()
        try:
            # t = self.tf_listener.getLatestCommonTime(start_tf,end_tf)
            (transform_obj.trans, transform_obj.rot) = self.tf_listener.lookupTransform(end_tf,start_tf,rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            transform_obj.trans = []
            transform_obj.rot = []
        return transform_obj

    def quat_multiplication(self,quat2,quat1):
        #print('quat multiplication function')
        #rotate q (quat1) by p (quat2) r = pq = [p]q
        if type(quat2) == list:
            p0 = quat2[3]
            p1 = quat2[0]
            p2 = quat2[1]
            p3 = quat2[2]
        elif type(quat2) == np.matrixlib.defmatrix.matrix:
            p0 = quat2.item(3)
            p1 = quat2.item(0)
            p2 = quat2.item(1)
            p3 = quat2.item(2)

        if type(quat1) == list:
            q0 = quat1[3]
            q1 = quat1[0]
            q2 = quat1[1]
            q3 = quat1[2]
        elif type(quat1) == np.matrixlib.defmatrix.matrix:
            q0 = quat1.item(3)
            q1 = quat1.item(0)
            q2 = quat1.item(1)
            q3 = quat1.item(2)


        qvec = np.matrix([[q0],[q1],[q2],[q3]])
        R = np.matrix([[p0,-p1,-p2,-p3], 
                       [p1, p0,-p3, p2], 
                       [p2, p3, p0,-p1], 
                       [p3,-p2, p1, p0]])
        r_quat = R*qvec  #Return final quaternion
        #r_quat = r_quat.tolist()
        #r_quat = [r_quat[0][0], r_quat[1][0],r_quat[2][0],r_quat[3][0]]
        return r_quat

    def rot_to_quat(self,R):
        R = np.matrix(R)
        q0 = 0.5* np.sqrt(R[0,0] + R[1,1] + R[2,2] + 1 )
        q1 = np.divide((R[1,2] - R[2,1]),(4*q0))
        q2 = np.divide((R[2,0] - R[0,2]),(4*q0))
        q3 = np.divide((R[0,1] - R[1,0]),(4*q0))
        q = np.matrix([[q1],[q2],[q3],[q0]])
        return q

    def quat_to_rot(self,q):
        #This rotational form is the 'transformation' version, imagine rotating one reference frame into the other, this describes the location of the transformation axis: e.g. for 90deg about v =[1,1,1], vector would be 
        # i to j, but transform would take i to k (z axis would sit on old x axis).
        if isinstance(q,list):
            q0 = q[3]
            q1 = q[0]
            q2 = q[1]
            q3 = q[2]
        else:
            q0 = q.item(3)
            q1 = q.item(0)
            q2 = q.item(1)
            q3 = q.item(2)

        R11 = 2*(q0**2 + q1**2) - 1
        R21 = 2*(q1*q2 - q0*q3)
        R31 = 2*(q1*q3 + q0*q2)

        R12 = 2*(q1*q2 + q0*q3)
        R22 = 2*(q0**2 + q2**2) - 1
        R32 = 2*(q2*q3 - q0*q1)
        
        R13 = 2*(q1*q3 - q0*q2)
        R23 = 2*(q2*q3 +q0*q1)
        R33 = 2*(q0**2 + q3**2) - 1
        
        Rmat = np.matrix([[R11, R12, R13],
                        [R21, R22 ,R23],
                        [R31, R32, R33]])
        '''NOTE: this returns exactly what quat is, hence R1 -> q -> R2  then R1 == R2, this was tested '''
        #Note that this is frame rotation {frame}^{R}_{base}  [gimmic: frame has neg in front (R21 is neg)]
        return Rmat

    def hat_map(self,vect):
        #hat map, 3vector to skew symmetric
        if type(vect) == np.matrixlib.defmatrix.matrix:
            skew_mat = np.matrix([[0,-vect.item(2),vect.item(1)],
                                  [vect.item(2),0,-vect.item(0)],
                                  [-vect.item(1), vect.item(0),0]])
        else:
            #assume list as defult
            skew_mat = np.matrix([[0,-vect[2],vect[1]],
                                  [vect[2],0,-vect[0]],
                                  [-vect[1], vect[0],0]])
        return skew_mat

    def vee_map(self,S):
        #this function takes skew sym matrix and returns its vector
        vect = [S[2,1], S[0,2], S[1,0]]
        return vect

    def ang_err_vect_SO3(self, *args):
        #first element passed should be acutal rotation matrix or quaternion, second is desired rotatio matrix or quaternion
        #Rotates from actual quaternion to desired

        #Actual rotation or quatnerion (base to end assumed)
        if type(args[0]) == list:
            #list quaternion
            Ra = self.quat_to_rot(args[0])  #Note: this assumes form: q = [q1 q2 q3 q0]
            Ra = Ra.T  #as we want the err vector in base frame, hence transpose both
        else:
            if args[0].shape == (3,3):
                #matrix, assumed that it is vector base to end frame
                Ra = args[0] #should be frame to base
            else:
                #quaternion numpy matrix
                Ra = self.quat_to_rot(args[0])  #Note: this assumes form: q = [q1 q2 q3 q0]
                Ra = Ra.T  #as we want the err vector in base frame, hence transpose both

        #Desired rotation or quatnerion (base to end assumed)
        if type(args[1]) == list:
            #list quaternion
            Rd = self.quat_to_rot(args[1])  #Note: this assumes form: q = [q1 q2 q3 q0]
            Rd = Rd.T #as we want the err vector in base frame, hence transpose both
        else:
            if args[1].shape == (3,3):
                #matrix, assumed that it is vector base to end frame
                Rd = args[1] #should be frame to base
            else:
                #quaternion numpy matrix
                Rd = self.quat_to_rot(args[1])  #Note: this assumes form: q = [q1 q2 q3 q0]
                Rd = Rd.T #as we want the err vector in base frame, hence transpose both

        #This is in base frame (all matricies should go from respective frames to base)
        err_skew_mat = 0.5*(Rd*Ra.T - Ra*Rd.T)
        err_vect = self.vee_map(err_skew_mat)
        err_vect = np.matrix([[err_vect[0]],[err_vect[1]],[err_vect[2]]]) 
        return err_vect

    def ang_err_meas_val(self,quat1, quat2):
        #this returns a scalar value that measures difference in orientation, value is 0 when orientation is exactly matched, for err meas values less than 0.02 are reasonably close, smaller is always better just a matter of convergence rate
        Rot_1_comp = np.matrix(self.quat_to_rot(quat1))
        Rot_2_comp = np.matrix(self.quat_to_rot(quat2))
        R_diff = Rot_1_comp.T*Rot_2_comp
        R_eval = 1.*np.diag(np.ones(3)) - R_diff
        # print('rotatino matrix', R_eval)
        trace_val = np.trace(R_eval)
        # print('trace of diff', trace_val)
        return trace_val

    def align_vectors_ang_err(self, n_vect, k_vect):
        #Find the ang error that aligns two vectors where k is current vector and n_vect is desired vector
        dot_prod = max(-1,min((np.dot((k_vect.T),n_vect)),1))  #the normal vector extrudes out from surface, we use the normal to define the z-axis of this C frame (z_c = -n_vect)
        contact_angle_curr = math.acos(dot_prod)  #using dot product of vectors we can get the angle between them which is hte angle of rotation
        k_vect_mat = self.hat_map(k_vect) #for cross product with z_cframe
        rot_vector_h_c = np.dot(k_vect_mat,n_vect) # this vector of rotation rotates hand frame to contact frame using angle contact_angle_curr above
        rot_vector_h_c_mat = self.hat_map(rot_vector_h_c) #find skew-sym matrix of rotation vector to construct rotation
        Rh_to_c = np.matrix(np.diag([1.,1.,1.])) + np.dot(math.sin(contact_angle_curr),rot_vector_h_c_mat) + np.dot((1- math.cos(contact_angle_curr)),rot_vector_h_c_mat**2) #rodruiguez formula, from exponential map of rotation angle and vector
        ang_err_mat = np.dot(0.5,(Rh_to_c - Rh_to_c.T))
        ang_err = self.vee_map(ang_err_mat)
        ang_err = np.matrix([[ang_err[0]],[ang_err[1]],[ang_err[2]]])
        return ang_err, Rh_to_c


    def tf_to_mat(self,tf_obj):
        if len(tf_obj.trans) != 0:
            obj_mat = np.matrix([tf_obj.trans[0],tf_obj.trans[1],tf_obj.trans[2],tf_obj.rot[0],tf_obj.rot[1],tf_obj.rot[2],tf_obj.rot[3]]).T
        else:
            #or make sure this is not entered if info not available
            obj_mat = np.matrix([[],[],[],[],[],[],[]])
        return obj_mat
    def transform_from_pose(self,tf_vect):
        #takes in vector: [x,y,z,q1,q2,q3,qw]_frame1_to_frame2 and returns 4x4 transform of TF_frame1_to_frame2
        pos_vect = tf_vect[:3]
        quat = tf_vect[3:]
        '''Very sure of below, but issues may arise here if you get turned around '''
        R_F1_to_F2 = self.quat_to_rot(quat).T #function returned the opposite, R_f2_f1, so transpose gives same frame transformation
        Transform = np.matrix([[R_F1_to_F2[0,0], R_F1_to_F2[0,1], R_F1_to_F2[0,2], pos_vect[0]],
                               [R_F1_to_F2[1,0], R_F1_to_F2[1,1], R_F1_to_F2[1,2], pos_vect[1]],
                               [R_F1_to_F2[2,0], R_F1_to_F2[2,1], R_F1_to_F2[2,2], pos_vect[2]],
                               [0,0,0,1]])
        return Transform

    def pose_from_transform(self, TF_obj):
        R = TF_obj[0:3,0:3]
        # print('inside callback')
        # print('R:', R)

        v = TF_obj[0:3,3]
        # print('v:', v)
        quat = self.rot_to_quat(R) #q1,q2,q3,q0
        # print('quat: ', quat)
        tfp = np.matrix(np.vstack([v,quat]))
        # print('pose from callback', tf_pose)
        return tfp


    def transform_vector(self,TF,vect):
        #it is assumed vector contains 3 elements, and TF is raw form: .trans, .rot
        tf_vect = self.tf_to_mat(TF)
        TF_mat = self.transform_from_pose(tf_vect)

        init_vect = np.vstack([vect,1])
        final_vect = TF_mat*init_vect

        # print "a: "#, init_vect, "\n b: "
        # final_vect = []
        # new_vect = np.dot(TF_mat,init_vect)
        # print "new vect: ", new_vect
        # final_vect = np.matrix([new_vect[0:3]])
        return final_vect[:3,0]


    def pose_err_fnct(self,curr_pose,des_pose):
        #given current pose: [x,y,z,q1,q2,q3,q0] #expects column
        #given desired pose: [x,y,z,q1,q2,q3,q0]
        if curr_pose.shape[0]!= 7:
            curr_pose = curr_pose.T
        if des_pose.shape[0] != 7:
            des_pose = des_pose.T


        err_pos = des_pose[:3] - curr_pose[:3]
        ang_err = self.ang_err_vect_SO3(curr_pose[3:],des_pose[3:])
        pose_err = np.matrix(np.vstack([err_pos,ang_err]))
        return pose_err
