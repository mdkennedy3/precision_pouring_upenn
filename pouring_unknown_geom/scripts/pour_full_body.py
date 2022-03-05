#!/usr/bin/env python
import rospy
import numpy as np
import scipy
from scipy import linalg

import PyKDL as kdl
import urdf_parser_py
from urdf_parser_py.urdf import Robot
import pykdl_utils.kdl_parser as kdl_parser_fnct
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_kinematics import kdl_to_mat
from pykdl_utils.kdl_kinematics import joint_kdl_to_list
from pykdl_utils.kdl_kinematics import joint_list_to_kdl
from sensor_msgs.msg import JointState
#Because of transformations
import tf
from tf import TransformerROS
import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import Float64, Empty
from pouring_unknown_geom.msg import HybCntrlData, PourAngStateMsg
import threading

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

#for sawyer simulation
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import (
    UInt16,
)

from pouring_unknown_geom.msg import PourJoints
from pouring_unknown_geom.srv import *

from pouring_unknown_geom.sawyer_control import sawyer_cntrl_cls


class FullBodyPourClass(object):
  """docstring for FullBodyPourClass"""
  def __init__(self, base_link="iiwa_link_0", end_link="iiwa_link_ee", cup_link='cup_edge', joint_callback_key_frame='iiwa_joint_1', robot_type=None):
    if robot_type in "iiwa":
      self.urdf = Robot.from_parameter_server()
    if robot_type in "sawyer":
      self.urdf = Robot.from_parameter_server("/robot_description")
    self.base_link = base_link
    self.cup_tf = cup_link
    self.end_link = end_link
    self.joint_callback_key_frame = joint_callback_key_frame
    self.robot_type = robot_type
    # tree = kdl.Tree(self.urdf.get_root())
    self.kdl_tree = kdl_tree_from_urdf_model(self.urdf)
    self.chain = self.kdl_tree.getChain(base_link, end_link)
    self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
    self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
    self._ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_kdl, self._ik_v_kdl)
    self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)
    self._dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector(0,0,-9.81))
    self.kdl_kin = KDLKinematics(self.urdf, base_link, end_link)
    self.num_joints = self.kdl_kin.num_joints
    self.joint_names = self.kdl_kin.get_joint_names()


    self.broadcast_static_transform = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    self.broadcast_static_transform_second = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    self.broadcast_transform = tf2_ros.TransformBroadcaster()
    self.transform_dict = dict()

    # self.broadcast_static_transform.sendTransform()
    self.rate = rospy.Rate(100) #hz
    self.tf_rate = rospy.Rate(10) #hz
    #For constant angular rate (accept from publisher)
    #self.angular_vel_bff_pub = rospy.Publisher('/pouring_control/Hybrid_controller_data',HybCntrlData,queue_size=1,latch=True)
    self.thread_lock = threading.Lock()
    self.task_space_omega = 0.0
    self.task_space_omega_cntrl = None
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.tfros = TransformerROS()
    #wait robot controller service
    rospy.wait_for_service('/robot_pour_commander')
    self.robot_pour_commander_proxy = rospy.ServiceProxy('/robot_pour_commander',PourPlatformControl)
    #static transform for nominal pose during pouring
    self.cup_tf_setpoint = None
    self.receiving_beaker_tag_name = "receiving_beaker_tag"

    self.world_frame = base_link #this can be updated to world for kuka if desired

    #For PI control
    # self.cup_frame_pose_integral_error_list = []
    # KI = 0.2
    # self.cup_frame_pose_integral_error_gain = np.matrix(np.diag(np.ones(3)*KI))
    # self.cup_frame_pose_integral_error_length = 45 #number of values to keep

    KI = 0.1
    self.cup_position_int_err = {"error":[], "KI":np.matrix(np.diag(np.ones(3)*KI)), "err_length":10,"int_err_norm_max":0.3} #was 45
    KI_ang = 0.1
    self.cup_angle_int_err = {"error":[], "KI":np.matrix(np.diag(np.ones(3)*KI_ang)), "err_length":10,"int_err_norm_max":0.3}

    self.cup_frame_pos_err_Kp = 4.0
    self.cup_frame_ang_err_Kp = 2.0#10.0#1.0

    self.cup_frame_pos_err_Kd = 5.0
    self.cup_frame_ang_err_Kd = 0.50#0.5#1.0


    self.cup_frame_pos_err_Ki = 2e-4
    self.cup_frame_ang_err_Ki = 1e-2#0.0#1.0


    # kp_task_space_gain = 10.0
    # self.task_space_Kp = kp_task_space_gain* np.matrix(np.eye(6))
    # kd_task_space_gain = 3.0
    # self.task_space_Kd = kd_task_space_gain* np.matrix(np.eye(6))

    filler_mat = np.matrix(np.zeros([3,3]))
    cup_frame_pos_Kp = self.cup_frame_pos_err_Kp*np.matrix(np.eye(3))
    cup_frame_ang_Kp = self.cup_frame_ang_err_Kp*np.matrix(np.eye(3))
    self.task_space_Kp = np.vstack([np.hstack([cup_frame_pos_Kp, filler_mat]), np.hstack([filler_mat, cup_frame_ang_Kp])])

    cup_frame_pos_Kd = self.cup_frame_pos_err_Kd*np.matrix(np.eye(3))
    cup_frame_ang_Kd = self.cup_frame_ang_err_Kd*np.matrix(np.eye(3))
    self.task_space_Kd = np.vstack([np.hstack([cup_frame_pos_Kd, filler_mat]), np.hstack([filler_mat, cup_frame_ang_Kd])])


    cup_frame_pos_Ki = self.cup_frame_pos_err_Ki*np.matrix(np.eye(3))
    cup_frame_ang_Ki = self.cup_frame_ang_err_Ki*np.matrix(np.eye(3))
    self.task_space_Ki = np.vstack([np.hstack([cup_frame_pos_Ki, filler_mat]), np.hstack([filler_mat, cup_frame_ang_Ki])])

    self.sawyer_cntrl_cls = sawyer_cntrl_cls.Sawyer_Control() #New test


    self.pouring_control_pub = rospy.Publisher('/pouring_control/pour_angle_state', PourAngStateMsg, queue_size=1, latch=False)
    self.starting_angle = 0



    #give the static transform pub a chance to come up
    rospy.sleep(1.0) #give it a moment to make static transforms

  def reset_starting_angle(self):
    try:
      trans = self.tfBuffer.lookup_transform('Nominal_aligned', self.cup_tf, rospy.Time(0))
      quat = [trans.transform.rotation.x, trans.transform.rotation.y,
              trans.transform.rotation.z, trans.transform.rotation.w]
      roll, _, _ = tf.transformations.euler_from_quaternion(quat)

      self.starting_angle = roll
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
      rospy.logwarn("Could not find tf to set the inital angle %s", ex)

  def pub_static_transform_temp(self):
    #This function pubs the static transform (which will be done by other code)
    #1. Cup edge transform
    cup_transform = self.cup_tf
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = self.end_link #"iiwa_link_ee"
    static_transformStamped.child_frame_id = cup_transform
    if self.robot_type == "sawyer":
      static_transformStamped.transform.translation.x = -0.06
      static_transformStamped.transform.translation.y = 0.06
      static_transformStamped.transform.translation.z = 0.13
      quat = tf.transformations.quaternion_from_euler(-np.pi/2.0,-np.pi/2,0.0)
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]
    elif self.robot_type == "iiwa":
      static_transformStamped.transform.translation.x = 0.13
      static_transformStamped.transform.translation.y = -0.06
      static_transformStamped.transform.translation.z = 0.06
      quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]
    self.static_pub_frame(transform=static_transformStamped)
    #self.broadcast_static_transform.sendTransform(static_transformStamped)
    rospy.loginfo("send static transform with name %s and end link: %s"%(cup_transform,self.end_link))

    #2. Tag on recieving beaker transform
    tag_transform = geometry_msgs.msg.TransformStamped()
    tag_transform.header.stamp = rospy.Time.now()
    tag_transform.header.frame_id = self.base_link #this frame is w.r.t. the base frame
    tag_transform.child_frame_id = self.receiving_beaker_tag_name
    tag_transform.transform.translation.x = 0.90 #0.80  #required farther back for sawyer
    tag_transform.transform.translation.y = 0.0
    tag_transform.transform.translation.z = 0.06
    tag_transform.transform.rotation.x = 0.0
    tag_transform.transform.rotation.y = np.sin(-np.pi/4)
    tag_transform.transform.rotation.z = 0.0
    tag_transform.transform.rotation.w = np.cos(np.pi/4)
    self.static_pub_frame(transform=tag_transform)
    #self.broadcast_static_transform_second.sendTransform(tag_transform)
    rospy.loginfo("Send tag transform with name %s from parent link %s"%(self.receiving_beaker_tag_name,self.base_link))

  def obtain_cup_transform(self,parent_link ='iiwa_link_ee', parent_joint= 'iiwa_joint_ee'):
    cup_transform = self.cup_tf
    success_bool = True
    print "parent link, cup trans", parent_link, cup_transform

    try:
      trans = self.tfBuffer.lookup_transform(parent_link,cup_transform,rospy.Time())
      #continue as transform is obtained
      print "obtained trans", trans
      self.add_cup_to_kdl_tree(transform_ee_to_cup=trans,parent_link=parent_link,parent_joint=parent_joint)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      self.tf_rate.sleep()
      success_bool = False
    return success_bool

  def generic_obtain_transform(self,target_frame=None, source_frame=None):
    success_bool = True
    print target_frame, source_frame
    try:
      trans = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())  #target frame, source frame, time
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      trans = []
      success_bool = False
    trans_return = {"success_bool":success_bool, "transform":trans}
    return trans_return

  def add_cup_to_kdl_tree(self,transform_ee_to_cup=None, parent_link=None, parent_joint=None):
    #setup params for adding cup_edge to kdl_tree
    joint = parent_joint
    parent = parent_link
    child_name= self.cup_tf #'cup_link'  #this is the link we are adding
    cup_kdl_inert = kdl.RigidBodyInertia()
    cup_kdl_jnt = kdl.Joint("cup_jnt",kdl.Joint.None)
    # cup_kdl_jnt = kdl_parser_fnct.urdf_joint_to_kdl_joint(self.urdf.joint_map[joint])
    #Obtain the tranform between cup edge and ee link
    cup_origin_ee_frame = urdf_parser_py.urdf.Pose  #need to define position/rotation
    cup_origin_ee_frame.position = [transform_ee_to_cup.transform.translation.x,transform_ee_to_cup.transform.translation.y,transform_ee_to_cup.transform.translation.z] #x,y,z
    cup_origin_ee_frame.rotation = [transform_ee_to_cup.transform.rotation.x,transform_ee_to_cup.transform.rotation.y,transform_ee_to_cup.transform.rotation.z,transform_ee_to_cup.transform.rotation.w] #x,y,z,w
    cup_kdl_origin = kdl.Frame(kdl.Rotation.Quaternion(*cup_origin_ee_frame.rotation),
                     kdl.Vector(*cup_origin_ee_frame.position))
    cup_kdl_seg = kdl.Segment(child_name, cup_kdl_jnt, cup_kdl_origin, cup_kdl_inert)
    self.kdl_tree.addSegment(cup_kdl_seg, parent)
    # rospy.sleep(0.5)
    #Given the update tree update the kdl params
    self.update_kdl_params()

  def update_kdl_params(self):
    #kdl_tree is updated, update all params
    end_link = self.cup_tf #'cup_link'
    self.chain = self.kdl_tree.getChain(self.base_link, end_link)
    self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
    self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
    self._ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_kdl, self._ik_v_kdl)
    self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)
    self._dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector(0,0,-9.81))

  def jacobian_generator(self, q=[]):
    j_kdl = kdl.Jacobian(self.num_joints)
    q_kdl = joint_list_to_kdl(q)
    self._jac_kdl.JntToJac(q_kdl, j_kdl)
    # keep kdl format m[i,j]
    return kdl_to_mat(j_kdl)


  def inertia(self, q=[]):
    h_kdl = kdl.JntSpaceInertiaMatrix(self.num_joints)
    self._dyn_kdl.JntToMass(joint_list_to_kdl(q), h_kdl)
    return kdl_to_mat(h_kdl)

  def coriolis(self,q=[], qdot=[]):
    q = q #list
    qdot = qdot #list
    q_cori = [0.0 for idx in range(len(q))]
    q_kdl = joint_list_to_kdl(q)
    qdot_kdl = joint_list_to_kdl(qdot)
    q_cori_kdl = joint_list_to_kdl(q_cori)
    self._dyn_kdl.JntToCoriolis(q_kdl, qdot_kdl, q_cori_kdl)
    return q_cori_kdl

  def gravity(self,q=[]):
    q_grav = [0.0 for idx in range(len(q))]
    q_kdl = joint_list_to_kdl(q)
    q_grav_kdl = joint_list_to_kdl(q_grav)
    self._dyn_kdl.JntToGravity(q_kdl,q_grav_kdl)
    # print "the q and the gravity:", q, q_grav_kdl
    return q_grav_kdl


  def joint_state_callback(self,req):

    key_frame=self.joint_callback_key_frame
    if key_frame in req.name:
      #this msg contains info for the arm
      q = []
      dq = []
      effort = []
      for joint_name in self.joint_names:
        idx = req.name.index(joint_name)
        q.append(req.position[idx])
        dq.append(req.velocity[idx])
        effort.append(req.effort[idx])

      J = self.jacobian_generator(q=q)

      qdot_list, gravity, theta = self.pour_motion_velocity(J=J, q=q,dq=dq,effort=effort, joint_names = self.joint_names)

      self.control_robot_joint_vel(qd_cmd=qdot_list, gravity=gravity)

      # qddot_list = self.pour_motion_torque(q=q, dq=dq, effort=effort, joint_names= self.joint_names)
      # self.control_robot_joint_torque(qdd_cmd=qddot_list)

      Jw = J[3:,:]
      omegas = req.velocity[1:-1] # Remove Head and Torso

      omega_vector = Jw * np.reshape(omegas, (7, 1))
      print req.velocity
      print "Omega vector", omega_vector

      pour_msg = PourAngStateMsg()
      pour_msg.header = req.header
      pour_msg.theta = theta - self.starting_angle
      pour_msg.omega = omega_vector[0][0]
      self.pouring_control_pub.publish(pour_msg)

    else:
      pass

  def control_robot_joint_vel(self,qd_cmd=[],gravity=None):
    #takes in a vector of commanded joint  velocities and pubs on respective platform
    if self.robot_type in "iiwa":
      pass
    if self.robot_type in "sawyer":
      suc = self.pour_velocity_sawyer(qdot_vect=qd_cmd, gravity=gravity)

  def control_robot_joint_torque(self,qdd_cmd=[]):
    if self.robot_type in "sawyer":
      suc = self.pour_torque_sawyer(qdd_cmd)

  # def publish_constant_angular_vel(self):
  #   hyb_data = HybCntrlData()
  #   hyb_data.cmd_omega = 2.0
  #   # hyb_data.cmd_omega = 1.0
  #   self.angular_vel_bff_pub.publish(hyb_data)  #publish angular velocity of 2rad/s on body fixed frame axis

  def hyb_data_callback(self,data):
    self.thread_lock.acquire()
    print "Updating velocity"
    self.task_space_omega = data.cmd_omega
    self.thread_lock.release()

  def pour_motion_velocity(self, J=None, q=None,dq=None,effort=None, joint_names=None):
    #Given the jacobian, rotate the following constant velocity vector in body fixed frame to
    #1. Obtain Jacobian
    qdot_vect = np.matrix(np.zeros([7,1]))

    # inertia = self.inertia(q=q)
    # coriolis = self.coriolis(q=q,qdot=dq)
    gravity = self.gravity(q=q) #needed in some simulators (gravity comp not automatic)

    #2. Obtain the desired task space omega (rotation axis known a-priori)
    self.thread_lock.acquire()
    self.task_space_omega_cntrl = self.task_space_omega #query the commanded angular velocity
    self.thread_lock.release()

    # Sinusoidal velocity profile used for testing
    # t_cyc = 10.0 #time in one direction
    # ang_vel_mag = 0.3
    # self.task_space_omega_cntrl = ang_vel_mag*np.sin(rospy.Time.now().to_sec()*(np.pi/t_cyc)) #rotate back and forth instead

    if type(self.task_space_omega_cntrl) == type(None):
      rospy.loginfo("no task space omega found")
      return qdot_vect
    #3. Build Velocity Vector

    try:
      trans = self.tfBuffer.lookup_transform('Nominal_aligned', self.cup_tf, rospy.Time(0))
      quat = [trans.transform.rotation.x, trans.transform.rotation.y,
              trans.transform.rotation.z, trans.transform.rotation.w]
      roll, _, _ = tf.transformations.euler_from_quaternion(quat)

      # Keeps the container from over rotating
      if roll > 2 and self.task_space_omega_cntrl > 0:
        self.task_space_omega_cntrl = 0
      if roll < 0 and self.task_space_omega_cntrl < 0:
        self.task_space_omega_cntrl = 0
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
      rospy.logwarn("Could not find tf %s", ex)

    #3a. obtain velocity vector in cup frame
    omega_vect_cup_frame = np.matrix([self.task_space_omega_cntrl, 0, 0]).T
    vel_vect_cup_frame = np.matrix([0,0,0]).T
    twist_vect_cup_frame = np.vstack([vel_vect_cup_frame, omega_vect_cup_frame])

    #3b. Obtain transformation from cup frame to base frame
    try:
      #obtain transform
      base_frame = self.base_link
      cup_transform = self.cup_tf
      trans = self.tfBuffer.lookup_transform(base_frame,cup_transform,rospy.Time())  #Target frame, source frame, time
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.loginfo("pour motion tf exception")
      return qdot_vect

    #3c. Rotate angular velocity vector into base link frame
    translation_tuple = (trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z )
    rotation_tuple = (trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
    TF_mat = self.tfros.fromTranslationRotation(translation_tuple, rotation_tuple)
    R_cup_base = TF_mat[:3,:3] #rotation from cup to base frame
    R_nominal_cup = np.matrix(self.cup_tf_setpoint)[:3,:3]

    #obtain error for position
    #position error between the frames in base link frame
    cup_frame_dist_base_pos_err = np.matrix(self.cup_tf_setpoint)[:3,3] - np.matrix(TF_mat)[:3,3]

    #obtain error for rotation
    x_ax_curr = np.matrix(TF_mat)[:3,0]; x_ax_curr_normed = np.divide(x_ax_curr,np.linalg.norm(x_ax_curr))
    x_ax_setpt = np.matrix(self.cup_tf_setpoint)[:3,0]; x_ax_setpt_normed = np.divide(x_ax_setpt,np.linalg.norm(x_ax_setpt))
    rot_axis_raw = self.skew_mat(x_ax_curr)*x_ax_setpt
    rot_axis = np.divide(rot_axis_raw,np.linalg.norm(rot_axis_raw))
    rot_angle = np.arccos(x_ax_curr_normed.T*x_ax_setpt_normed)
    rot_angle_axis_error = np.multiply(self.cup_frame_ang_err_Kp*rot_angle,rot_axis)


    #integral error for angle
    angle_int_err = np.matrix([0.0,0.0,0.0]).T
    if len(self.cup_angle_int_err["error"]) > 0:
      for int_idx in range(len(self.cup_angle_int_err["error"])):
        angle_int_err += self.cup_angle_int_err["error"][int_idx]

    #update err
    self.cup_angle_int_err["error"].append(np.matrix(rot_angle_axis_error))
    if len(self.cup_angle_int_err["error"]) >= self.cup_angle_int_err["err_length"]:
      self.cup_angle_int_err["error"].pop(0)

    #commanded omega in base frame
    #Feed forward command
    #rotate the desired omega into base frame (base of jacobian (usually last link))
    omega_vect_base_frame = R_cup_base*R_nominal_cup*omega_vect_cup_frame
    task_space_angle_error = rot_angle_axis_error #Angle error


    pose_error_vect_base_frame = np.vstack([self.cup_frame_pos_err_Kp * cup_frame_dist_base_pos_err,
                                            self.cup_frame_ang_err_Kp * task_space_angle_error])
    task_space_feed_forward = np.vstack([np.matrix(np.zeros([3,1])), omega_vect_base_frame])
    #stop if you pass the threshold rotation:
    if rot_angle >= np.pi/2:
      pose_error_vect_base_frame = np.matrix(np.zeros([6,1]))
      rospy.loginfo("rot angle canceling err vect is %f"%rot_angle)

    xdot = pose_error_vect_base_frame + task_space_feed_forward
    # xdot = task_space_feed_forward
    Jv = J[:3,:]
    Jw = J[3:,:]

    #qdot_vect[6] = self.task_space_omega_cntrl
    qdot_vect = np.linalg.pinv(Jv)*xdot[:3] + (np.matrix(np.eye(7))- np.linalg.pinv(Jv)*Jv)*(np.linalg.pinv(Jw)*xdot[3:])
    #qdot_vect = np.linalg.pinv(J)*xdot
    qdot_list = qdot_vect.T.tolist()[0]


    return qdot_list, gravity, roll



  def pour_motion_torque(self,q=None,dq=None,effort=None, joint_names=None):
    #Given the jacobian, rotate the following constant velocity vector in body fixed frame to
    #1. Obtain Jacobian
    qdot_vect = np.matrix(np.zeros([7,1]))
    J = self.jacobian_generator(q=q)
    inertia = self.inertia(q=q)
    coriolis = self.coriolis(q=q,qdot=dq)
    gravity = self.gravity(q=q)
    #2. Obtain the desired task space omega (rotation axis known a-priori)
    self.thread_lock.acquire()
    self.task_space_omega_cntrl = self.task_space_omega
    self.thread_lock.release()

    # Sinusoidal velocity profile used for testing
    # t_cyc = 10.0 #time in one direction
    # ang_vel_mag = 0.3
    # self.task_space_omega_cntrl = ang_vel_mag*np.sin(rospy.Time.now().to_sec()*(np.pi/t_cyc))

    if type(self.task_space_omega_cntrl) == type(None):
      rospy.loginfo("no task space omega found")
      return qdot_vect
    #3. Build Velocity Vector
    #3a. obtain velocity vector in cup frame
    omega_vect_cup_frame = np.matrix([self.task_space_omega_cntrl, 0, 0]).T
    vel_vect_cup_frame = np.matrix([0,0,0]).T
    twist_vect_cup_frame = np.vstack([vel_vect_cup_frame, omega_vect_cup_frame])

    #3b. Obtain transformation from cup frame to base frame
    try:
      #obtain transform
      base_frame = self.base_link
      cup_transform = self.cup_tf
      trans = self.tfBuffer.lookup_transform(base_frame,cup_transform,rospy.Time())  #Target frame, source frame, time
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.loginfo("pour motion tf exception")
      return qdot_vect
    #3c. Rotate angular velocity vector into base link frame
    translation_tuple = (trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z )
    rotation_tuple = (trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
    TF_mat = self.tfros.fromTranslationRotation(translation_tuple, rotation_tuple)
    R_cup_base = TF_mat[:3,:3] #rotation from cup to base frame
    R_nominal_cup = np.matrix(self.cup_tf_setpoint)[:3,:3]
    #obtain error for position
    cup_frame_dist_base_pos_err = np.matrix(self.cup_tf_setpoint)[:3,3] - np.matrix(TF_mat)[:3,3]  #position error between the frames in base link frame

    #obtain error for rotation
    x_ax_curr = np.matrix(TF_mat)[:3,0]; x_ax_curr_normed = np.divide(x_ax_curr,np.linalg.norm(x_ax_curr))
    x_ax_setpt = np.matrix(self.cup_tf_setpoint)[:3,0]; x_ax_setpt_normed = np.divide(x_ax_setpt,np.linalg.norm(x_ax_setpt))
    rot_axis_raw = self.skew_mat(x_ax_curr)*x_ax_setpt
    rot_axis = np.divide(rot_axis_raw,np.linalg.norm(rot_axis_raw))
    rot_angle = np.arccos(x_ax_curr_normed.T*x_ax_setpt_normed)
    rot_angle_axis_error = np.multiply(self.cup_frame_ang_err_Kp*rot_angle,rot_axis)


    #integral error for angle
    angle_int_err = np.matrix([0.0,0.0,0.0]).T
    if len(self.cup_angle_int_err["error"]) > 0:
      for int_idx in range(len(self.cup_angle_int_err["error"])):
        angle_int_err += self.cup_angle_int_err["error"][int_idx]

    #update err
    self.cup_angle_int_err["error"].append(np.matrix(rot_angle_axis_error))
    if len(self.cup_angle_int_err["error"]) >= self.cup_angle_int_err["err_length"]:
      self.cup_angle_int_err["error"].pop(0)

    #commanded omega in base frame
    #Feed forward command
    omega_vect_base_frame = R_cup_base*R_nominal_cup*omega_vect_cup_frame #+ rot_angle_axis_error + angle_int_err #last term is fb to orient the x-z plane to original cup_edge plane
    task_space_angle_error = rot_angle_axis_error #Angle error

    cup_frame_dist_base_pos_err_integral = np.matrix([0.0,0.0,0.0]).T
    if len(self.cup_position_int_err["error"]) > 0:
      for int_idx in range(len(self.cup_position_int_err["error"])):
        cup_frame_dist_base_pos_err_integral += self.cup_position_int_err["error"][int_idx]
    self.cup_position_int_err["error"].append(np.matrix(cup_frame_dist_base_pos_err)) #add latest error
    if len(self.cup_position_int_err["error"]) > self.cup_position_int_err["err_length"]:
      self.cup_position_int_err["error"].pop(0)  #take of the first, but add the last

    #3d. Find the angular velocities
    # vel_vect_base_frame = np.matrix([0,0,0]).T + 10*cup_frame_dist_base_pos_err + cup_frame_dist_base_pos_err_integral #40
    # vel_vect_base_frame =  self.cup_frame_pos_err_Kp*cup_frame_dist_base_pos_err #+ cup_frame_dist_base_pos_err_integral #40
    # vel_vect_base_frame =  cup_frame_dist_base_pos_err #+ cup_frame_dist_base_pos_err_integral #40

    #Integral error
    pose_error_int_vect_base_frame = np.vstack([cup_frame_dist_base_pos_err_integral,angle_int_err])

    pose_error_vect_base_frame = np.vstack([cup_frame_dist_base_pos_err, task_space_angle_error])
    task_space_feed_forward = np.vstack([np.matrix(np.zeros([3,1])), omega_vect_base_frame])
    #stop if you pass the threshold rotation:
    if rot_angle >= np.pi/2:
      pose_error_vect_base_frame = np.matrix(np.zeros([6,1]))
      rospy.loginfo("rot angle canceling err vect is %f"%rot_angle)
    # print "twist_vect_base_frame", pose_error_vect_base_frame

    # print "pose_error_vect_base_frame", pose_error_vect_base_frame, "\n composed of x: ", vel_vect_base_frame, " \n and th: ", omega_vect_base_frame
    qdot_vect = np.linalg.pinv(J)*pose_error_vect_base_frame
    qdot_list = qdot_vect.T.tolist()[0]


    # qddot_vect = np.linalg.pinv(J)*xddot
    Jv = J[:3,:]
    Jw = J[3:,:]

    #Get the task space velocity
    qd_vect = np.matrix(dq).T
    vel_task_space = np.matrix(J)*qd_vect #only want to limit translational motion, rotation should come from input term, this is currently handled in zeroing the angle Kd term


    # print "task space error", self.task_space_Kp*pose_error_vect_base_frame, " vel ", self.task_space_Kd*vel_task_space, " e_int: ", self.task_space_Ki*pose_error_int_vect_base_frame  , " ff input: ", task_space_feed_forward


    xddot = self.task_space_Kp*pose_error_vect_base_frame - self.task_space_Kd*vel_task_space + self.task_space_Ki*pose_error_int_vect_base_frame+ task_space_feed_forward


    qddot_vect = np.linalg.pinv(Jv)*xddot[:3] + (np.matrix(np.eye(7))- np.linalg.pinv(Jv)*Jv)*(np.linalg.pinv(Jw)*xddot[3:])
    # print "type of gravity ", gravity, type(gravity), type(np.matrix(gravity)), " \n and as matrix ", np.matrix(gravity).T
    G = np.matrix([g for g in gravity]).T
    C = np.matrix([c for c in coriolis]).T
    # print "new gravity ", G

    # qddot_vect = np.matrix(np.zeros([7,1]))
    qddot_vect += G*0.01  #They didn't add the tool weight
    # qddot_vect += G*1.01  #They didn't add the tool weight
    # qddot_vect += C
    qddot_list = qddot_vect.T.tolist()[0]

    return qddot_list

  def move_to_pour_pose_iiwa(self):
    #Move iiwa to start pose
    pour_cntrl_msg = PourJoints()
    pour_cntrl_msg.header.stamp = rospy.Time().now()
    pour_cntrl_msg.header.frame_id = self.base_link
    pour_cntrl_msg.robot_name = self.robot_type
    pour_cntrl_msg.control_type = "position"
    pour_cntrl_msg.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    pour_cntrl_msg.positions = [-0.05548106943134723, 1.0663330526390924, 0.11734395223961602, -1.3391940140682825, 0.12953519125060708, -0.8623848674008077, -0.22567503607183603]
    serv_req =  PourPlatformControlRequest()
    serv_req.pour_data= pour_cntrl_msg
    resp = self.robot_pour_commander_proxy(serv_req)
    return resp.success

  def move_to_pour_pose_sawyer(self):
    #publish this to message for it to move to this position
    pour_cntrl_msg = PourJoints()
    pour_cntrl_msg.header.stamp = rospy.Time().now()
    pour_cntrl_msg.header.frame_id = self.base_link
    pour_cntrl_msg.robot_name = self.robot_type
    pour_cntrl_msg.control_type = "position"
    pour_cntrl_msg.joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    #pour_cntrl_msg.positions = [0.106501953125, -0.6191748046875, -0.0802666015625, 1.27628125, 4.6875e-05, -0.7341982421875, -1.237880859375]
    pour_cntrl_msg.positions = [0.1212236328125, -0.4567724609375, -0.424419921875, 1.083107421875, 0.1218349609375, -0.5848310546875, -0.5426650390625]
    serv_req =  PourPlatformControlRequest()
    serv_req.pour_data= pour_cntrl_msg
    resp = self.robot_pour_commander_proxy(serv_req)
    # self.sawyer_pub.publish(pour_cntrl_msg) #send to script outside of namespace to command
    rospy.loginfo("published pos message for sawyer")
    print "success bool: ", resp.success
    return resp.success

  def pour_velocity_sawyer(self,qdot_vect=[], gravity=None):
    #this function sends the desired velocities to the sawyer

    pour_cntrl_msg = PourJoints()
    pour_cntrl_msg.header.stamp = rospy.Time().now()
    pour_cntrl_msg.header.frame_id = self.base_link
    pour_cntrl_msg.robot_name = self.robot_type
    pour_cntrl_msg.control_type = "velocity"
    pour_cntrl_msg.joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    pour_cntrl_msg.velocities = qdot_vect
    # print "\n\ncmd velocities", qdot_vect
    serv_req =  PourPlatformControlRequest()
    serv_req.pour_data= pour_cntrl_msg
    #IF I need this then maybe I'm not doing it right... Should be using publisher instead (but using service just for iiwa moveit)
    rospy.wait_for_service('/robot_pour_commander')
    # self.sawyer_pub.publish(pour_cntrl_msg) #send to script outside of namespace to command
    resp = self.robot_pour_commander_proxy(serv_req)

    return resp.success

    #Trying old velocity commander
    #omega = 2.3
    #self.sawyer_cntrl_cls.cmd_joints({'right_j6':omega},'velocity')



    #torque = dict(zip(pour_cntrl_msg.joint_names, gravity))
    #print "publishing gravity", torque
    #self.sawyer_cntrl_cls.cmd_joints(torque,'torque')

    #return True



  def pour_torque_sawyer(self,qddot_vect=[]):
    #This function sends the desired torques to the sawyer
    pour_cntrl_msg = PourJoints()
    pour_cntrl_msg.header.stamp = rospy.Time().now()
    pour_cntrl_msg.header.frame_id = self.base_link
    pour_cntrl_msg.robot_name = self.robot_type
    pour_cntrl_msg.control_type = "torque"
    pour_cntrl_msg.joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    pour_cntrl_msg.torques = qddot_vect
    serv_req =  PourPlatformControlRequest()
    serv_req.pour_data= pour_cntrl_msg
    #IF I need this then maybe I'm not doing it right... Should be using publisher instead (but using service just for iiwa moveit)
    rospy.wait_for_service('/robot_pour_commander')
    # self.sawyer_pub.publish(pour_cntrl_msg) #send to script outside of namespace to command
    resp = self.robot_pour_commander_proxy(serv_req)
    return resp.success


  def set_pouring_nominal_static_pose(self, nom_frame=None):
    #obtain the transform to be used for error attentuation (position, orientation excluding rotation about cup x-axis)
    self.cup_tf_setpoint #type TF
    if type(nom_frame) == None:
      tf_collect_bool = False
      while not rospy.is_shutdown() and not tf_collect_bool:
        try:
          base_frame = self.base_link
          cup_transform = self.cup_tf
          trans = self.tfBuffer.lookup_transform(base_frame,cup_transform,rospy.Time())  #1. target 2. source
          translation_tuple = (trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z )
          rotation_tuple = (trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
          self.cup_tf_setpoint = np.matrix(self.tfros.fromTranslationRotation(translation_tuple, rotation_tuple)) #store the transform
          tf_collect_bool = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
          rospy.loginfo("TF exception %s"%e)
          pass
    else:
      trans = nom_frame
      translation_tuple = (trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z )
      rotation_tuple = (trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
      self.cup_tf_setpoint = np.matrix(self.tfros.fromTranslationRotation(translation_tuple, rotation_tuple)) #store the transform)

  def static_pub_frame(self,transform=None):
    #this function pubs the provided static transform
    self.broadcast_static_transform.sendTransform(transform)
    self.transform_dict[transform.header.frame_id + ' -> ' + transform.child_frame_id] = transform
    rospy.loginfo("Pub static transform with name %s and parent link: %s"%(transform.child_frame_id,transform.header.frame_id))
    self.publish_transforms()

  def publish_transforms(self):
    for transform in self.transform_dict.itervalues():
        transform.header.stamp = rospy.Time.now()
        self.broadcast_transform.sendTransform(transform)


  def obtain_pouring_setup_terms(self):
    '''
    Make static variables available for translation/rotation pour
    -------- Gripper/Pouring cup
    1. Distance from finger to cup edge (scalar (m))
    2. Finger safety distance (additional distance of safety plane to ensure fingers do not collide with reciever) (scalar (m))
    ------- Receiving Cup -----------
    3. z-height of recieving beaker in world/base frame (scalar (m))
    4. radius of beaker (scalar (m))
    5. pour past edge distance (scalar (m)): distance from the edge of the beaker that the pouring edge on pourer should translate on during pour
    '''
    self.cup_edge_to_finger_distance = None #scalar
    self.robot_finger_safety_distance = None #scalar
    self.z_world_beaker = None
    self.receiving_beaker_radius = None
    self.pour_past_edge_distance = None
    #1. Set/obtain the params (these can be passed in from a state machine, just scalars)
    #This can be found in practice, by getting the transform from the finger to the cup (with final frame being the cup), then use the y displacement (which is negative when pouring cw) as this distance
    if rospy.has_param('cup_edge_to_finger_distance'): self.cup_edge_to_finger_distance = rospy.get_param('cup_edge_to_finger_distance')
    else: self.cup_edge_to_finger_distance = 0.1 #m
    if rospy.has_param('robot_finger_safety_distance'): self.robot_finger_safety_distance = rospy.get_param('robot_finger_safety_distance')
    else: self.robot_finger_safety_distance = 0.015 #m
    if rospy.has_param('z_world_beaker'): self.z_world_beaker = rospy.get_param('z_world_beaker')
    else: self.z_world_beaker = 0.11 #m
    if rospy.has_param('receiving_beaker_radius'): self.receiving_beaker_radius = rospy.get_param('receiving_beaker_radius')
    else: self.receiving_beaker_radius = 0.04  #m
    if rospy.has_param('pour_past_edge_distance'): self.pour_past_edge_distance = rospy.get_param('pour_past_edge_distance')
    else: self.pour_past_edge_distance = 0.015  #m

    '''
    2. Set the neccessary frames given this info:
    a) Guard frame (G) defined by cup frame, cup to finger distance and robot finger safety distance (same orientation as cup frame)
    b) Beaker Frame
    c) Beaker edge frame
    d) Beaker pourline frame
    '''
    #Guard frame: (this updates with every cup)
    self.guard_frame_transform = geometry_msgs.msg.TransformStamped()
    self.guard_frame_transform.header.stamp = rospy.Time.now()
    self.guard_frame_transform.header.frame_id = self.cup_tf #this is the parent frame
    self.guard_frame_transform.child_frame_id = "cup_guard"
    self.guard_frame_transform.transform.translation.x = 0.0
    self.guard_frame_transform.transform.translation.y = -float(self.cup_edge_to_finger_distance+self.robot_finger_safety_distance)
    self.guard_frame_transform.transform.translation.z = 0.0
    self.guard_frame_transform.transform.rotation.x = 0.0
    self.guard_frame_transform.transform.rotation.y = 0.0
    self.guard_frame_transform.transform.rotation.z = 0.0
    self.guard_frame_transform.transform.rotation.w = 1.0

    #PUB THE STATIC TRANSFORM
    self.static_pub_frame(transform=self.guard_frame_transform)
    rospy.sleep(2.0)

    # w.r.t self.world_frame (currently same as base_link)
    #beaker frame (this is constant for a stationary recieving beaker)
    query_bool = False
    while not query_bool and not rospy.is_shutdown():
      trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame=self.receiving_beaker_tag_name)
      query_bool = trans_return["success_bool"]
    tf_tag_to_world = trans_return["transform"]

    query_bool = False
    while not query_bool and not rospy.is_shutdown():
      trans_return = self.generic_obtain_transform(target_frame=self.receiving_beaker_tag_name, source_frame=self.world_frame)
      query_bool = trans_return["success_bool"]
    tf_world_to_tag = trans_return["transform"]
    pos = (tf_tag_to_world.transform.translation.x, tf_tag_to_world.transform.translation.y, tf_tag_to_world.transform.translation.z)
    quat = (tf_tag_to_world.transform.rotation.x, tf_tag_to_world.transform.rotation.y, tf_tag_to_world.transform.rotation.z, tf_tag_to_world.transform.rotation.w )
    TF_mat_tag_to_world = np.matrix(self.tfros.fromTranslationRotation(pos,quat))

    rotated_z_vector_into_tag_frame = TF_mat_tag_to_world[:3,:3].T*np.matrix([0,0,(self.z_world_beaker - tf_tag_to_world.transform.translation.z)]).T

    self.beaker_frame_name = "beaker_frame"

    #Keeping this wrt the tag is useful if the beaker shifts but the tag is still visible
    self.beaker_frame_transform = geometry_msgs.msg.TransformStamped()
    self.beaker_frame_transform.header.stamp = rospy.Time.now()
    self.beaker_frame_transform.header.frame_id = self.receiving_beaker_tag_name #this is the parent frame
    self.beaker_frame_transform.child_frame_id = self.beaker_frame_name
    self.beaker_frame_transform.transform.translation.x = rotated_z_vector_into_tag_frame.item(0)
    self.beaker_frame_transform.transform.translation.y = rotated_z_vector_into_tag_frame.item(1)
    self.beaker_frame_transform.transform.translation.z = -self.receiving_beaker_radius + rotated_z_vector_into_tag_frame.item(2) #go to the center of the beaker
    self.beaker_frame_transform.transform.rotation.x = tf_world_to_tag.transform.rotation.x
    self.beaker_frame_transform.transform.rotation.y = tf_world_to_tag.transform.rotation.y
    self.beaker_frame_transform.transform.rotation.z = tf_world_to_tag.transform.rotation.z
    self.beaker_frame_transform.transform.rotation.w = tf_world_to_tag.transform.rotation.w

    """
    Here is where some fancy calculation is required, the edge frame is determined based on both the
    radius of the container and the beakers location wrt the base link frame for optimial pour motion
    """

    self.static_pub_frame(transform=self.beaker_frame_transform)
    rospy.sleep(2.0)

    #obtain the new transform from between the beaker and world
    query_bool = False
    while not query_bool and not rospy.is_shutdown():
      trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame=self.beaker_frame_name)
      query_bool = trans_return["success_bool"]
    tf_beaker_to_world = trans_return["transform"]

    # print "tf beaker to world", tf_beaker_to_world
    edge_frames =self.calculate_beaker_edge(transform=tf_beaker_to_world, beaker_radius=self.receiving_beaker_radius, beaker_frame_name=self.beaker_frame_name, beaker_edge_line_distance=self.pour_past_edge_distance)


    self.beaker_edge_frame_transform = edge_frames['edge']  # 'edge_line'
    self.beaker_pourline_frame_transform = edge_frames['edge_line']

    #Pub the beaker related transforms
    self.static_pub_frame(transform=self.beaker_edge_frame_transform)
    self.static_pub_frame(transform=self.beaker_pourline_frame_transform)
    rospy.sleep(1.0)

    rospy.loginfo("Published all complementary frames")

  def skew_mat(self,v):
    #Expects a np matrix/vector
    return np.matrix([[0,-v.item(2),v.item(1)],[v.item(2),0,-v.item(0)],[-v.item(1),v.item(0),0]])

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
      x_arg = Lb**2 - y1**2
      x1 = np.sqrt(x_arg)
      x2 = -np.sqrt(x_arg)
    elif np.abs(yb) > 0.0:
      A = (1+(xb/yb)**2)
      B = (xb/(yb**2))*(rb**2-Lb**2-(xb**2+yb**2))
      C =  (1/(4*yb**2))*(rb**2-Lb**2 -(xb**2+yb**2))**2 - Lb**2
      x1 = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
      x2 = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)
      y_arg = Lb**2 - x1**2
      y1 = np.sqrt(y_arg)
      y2 = -np.sqrt(y_arg)
    else:
      rospy.loginfo("beaker is at the world origin which is not allowed")
    combos = [[x1,y1],[x1,y2],[x2,y1],[x2,y2]]
    combo_scores = []
    for idx in range(len(combos)):
      score = query_point(x=combos[idx][0], y=combos[idx][1], xb=xb,yb=yb, rb=rb, Lb=Lb)
      combo_scores.append(score)
    combo_sort = np.argsort(combo_scores)
    print "combo scores", combo_scores
    final_set = np.array(combos)[combo_sort[:2]] #first two sorted are smallest
    print "final set", final_set
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
    print "diff vect before norm", diff_vect1, diff_vect2
    print "v1,v2", diff_vect1_normed, diff_vect2_normed
    print "cross w1, w2", cross_vect1, cross_vect2
    if cross_vect1.item(2) > 0:
      left_pt = final_set[0]; right_pt = final_set[1]
      print "left_pt", left_pt
    else:
      left_pt = final_set[1]; right_pt = final_set[0]
    #if pouring from the left pass the left, else pass the one on the right
    print "pour_dir", pour_dir
    if pour_dir in "left": return left_pt #choose the one on the left of the beaker line
    else: return right_pt #choose the one on the right of the beaker line

  def calculate_beaker_edge(self,transform=None, beaker_radius=None, beaker_frame_name=None, beaker_edge_line_distance=None):
    #TF is beaker to world
    #1. Beaker location in xy world frame
    beaker_xy = np.matrix([transform.transform.translation.x,transform.transform.translation.y]).T
    #2. Find the radius (x,y) of beaker center in world
    xy_dist_beaker_world = np.linalg.norm(beaker_xy)
    # print "beaker center", beaker_xy
    # print "beaker rad", beaker_radius
    # print "xy dist beaker", xy_dist_beaker_world
    #3. Circle equations
    '''
    Circle Equations:
    a) (x-xb)**2 + (y-yb)**2 = rb**2
    b) (x)**2 + (y)**2 = Lb**2
    '''
    pt_on_circle_world_frame = self.circ_intersect_func(xb=beaker_xy.item(0),yb=beaker_xy.item(1),rb=beaker_radius,Lb=xy_dist_beaker_world, pour_dir="left")  #returns nd_array (x,y)
    #Now this is the location of the edge in world frame, but what I need is the transform from beaker to edge (keep it locally defined in the chain starting with tag)
    #orientation of the edge is the same as the beaker
    quat_all = (transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w)
    pos_beaker = (transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z)
    pos_beaker_edge = (pt_on_circle_world_frame[0],pt_on_circle_world_frame[1],transform.transform.translation.z)

    TF_mat_beaker_to_world = np.matrix(self.tfros.fromTranslationRotation(pos_beaker,quat_all))
    TF_mat_beaker_edge_to_world = np.matrix(self.tfros.fromTranslationRotation(pos_beaker_edge,quat_all))
    TF_mat_beaker_edge_to_beaker = np.linalg.inv(TF_mat_beaker_to_world) * TF_mat_beaker_edge_to_world

    #Return both edge, and edge line wrt the beaker transform instead of world
    # skewmat = scipy.linalg.logm(TF_mat_beaker_edge_to_beaker[:3,:3])
    # print "resulting skewmat", skewmat
    # v_ang = np.matrix([skewmat[2,1],skewmat[0,2],skewmat[0,1]]).T
    # ang = np.linalg.norm(v_ang)
    # v = np.divide(v_ang,ang)
    B_E_px=TF_mat_beaker_edge_to_beaker[0,3]
    B_E_py=TF_mat_beaker_edge_to_beaker[1,3]
    B_E_pz=TF_mat_beaker_edge_to_beaker[2,3]
    B_E_qx = 0.0#v.item(0)*np.sin(ang/2)
    B_E_qy = 0.0#v.item(1)*np.sin(ang/2)
    B_E_qz = 0.0#v.item(2)*np.sin(ang/2)
    B_E_qw = 1.0#np.cos(ang/2)

    beaker_edge_in_beaker_frame = geometry_msgs.msg.TransformStamped()
    beaker_edge_in_beaker_frame.header.stamp = rospy.Time.now()
    beaker_edge_in_beaker_frame.header.frame_id = beaker_frame_name #this is the parent frame
    beaker_edge_in_beaker_frame.child_frame_id = "beaker_edge"
    beaker_edge_in_beaker_frame.transform.translation.x = B_E_px
    beaker_edge_in_beaker_frame.transform.translation.y = B_E_py
    beaker_edge_in_beaker_frame.transform.translation.z = B_E_pz
    beaker_edge_in_beaker_frame.transform.rotation.x =  B_E_qx
    beaker_edge_in_beaker_frame.transform.rotation.y = B_E_qy
    beaker_edge_in_beaker_frame.transform.rotation.z = B_E_qz
    beaker_edge_in_beaker_frame.transform.rotation.w =  B_E_qw

    E_frame_xy_norm = np.linalg.norm(np.matrix([B_E_px,B_E_py]))
    E_edge_frame_xy_norm = E_frame_xy_norm - beaker_edge_line_distance
    E_frame_xy_angle = np.arctan2(B_E_py, B_E_px)

    #NOW calculate the E' frame for the line
    beaker_edge_line_in_beaker_frame = geometry_msgs.msg.TransformStamped()
    beaker_edge_line_in_beaker_frame.header.stamp = rospy.Time.now()
    beaker_edge_line_in_beaker_frame.header.frame_id = beaker_frame_name #this is the parent frame
    beaker_edge_line_in_beaker_frame.child_frame_id = "beaker_edge_line"
    beaker_edge_line_in_beaker_frame.transform.translation.x = E_edge_frame_xy_norm*np.cos(E_frame_xy_angle)
    beaker_edge_line_in_beaker_frame.transform.translation.y = E_edge_frame_xy_norm*np.sin(E_frame_xy_angle)
    beaker_edge_line_in_beaker_frame.transform.translation.z = B_E_pz
    beaker_edge_line_in_beaker_frame.transform.rotation.x =  B_E_qx
    beaker_edge_line_in_beaker_frame.transform.rotation.y = B_E_qy
    beaker_edge_line_in_beaker_frame.transform.rotation.z = B_E_qz
    beaker_edge_line_in_beaker_frame.transform.rotation.w =  B_E_qw

    edge_frames = {"edge":beaker_edge_in_beaker_frame, "edge_line":beaker_edge_line_in_beaker_frame}

    return edge_frames

  def generate_desired_start_pose_for_pour(self,start_angle=None, pub_static_transforms=True):
    '''
    Start angle is in radians
    Assumes that the following frames are available:
      1. C (cup), name: self.cup_tf
      2. G (guard frame), name: "cup_guard"
      3. E (edge frame), name: "beaker_edge"
      4. E_line (pouring line frame), name: "beaker_edge_line"
      5. B (Beaker frame), name: "beaker_frame"
    All of these terms have been set (passed in as a func and/or set on param server)
      self.cup_edge_to_finger_distance  (distance from cup frame to edge of finger)
      self.robot_finger_safety_distance  (additional distance to protect finger)
      self.z_world_beaker  (height of the recieving beaker in world frame z)
      self.receiving_beaker_radius (radius of recieving beaker)
      self.pour_past_edge_distance (distance from the edge of the translation line for pouring cup edge frame)
    The starting height H given the starting angle th_s from a nominal pose is
      H = a/sin(th_s) + b/tan(th_s) where
      a = self.cup_edge_to_finger_distance+self.robot_finger_safety_distance and
      b = self.pour_past_edge_distance
    '''
    th_s = start_angle
    a = self.cup_edge_to_finger_distance+self.robot_finger_safety_distance
    b = self.pour_past_edge_distance
    H = a/np.sin(th_s) + b/np.tan(th_s)

    #Goal is to get nominal_start frame pose to start
    #1. Nominal frame x-axis direction: Get the orientation from the  relative poses between E_line and the world
    #2. Nominal frame z-axis direction: N_consistent same as world z-axis* / N_start (rotated by th_s from world z) *(however the pose to move to is not the same here (so one is start and one is nominal))
    #3. Nominal frame x,y position: same as E_line
    #4. Nominal frame z position: E_line + H

    #A. Obtain transform from E_line to world
    query_bool = False
    while not query_bool and not rospy.is_shutdown():
      trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame="beaker_edge_line")
      query_bool = trans_return["success_bool"]
    tf_E_line_to_world = trans_return["transform"]
    #convert to mat
    position_tuple_E_W = (tf_E_line_to_world.transform.translation.x,tf_E_line_to_world.transform.translation.y,tf_E_line_to_world.transform.translation.z)
    orientation_tuple_E_W = (tf_E_line_to_world.transform.rotation.x,tf_E_line_to_world.transform.rotation.y,tf_E_line_to_world.transform.rotation.z, tf_E_line_to_world.transform.rotation.w)
    TF_mat = np.matrix( self.tfros.fromTranslationRotation(position_tuple_E_W,orientation_tuple_E_W))
    #Find axis
    #a) x_axis
    N_x_axis = np.vstack([np.divide(TF_mat[:2,3], np.linalg.norm(TF_mat[:2,3])), 0])  #[dx,dy,0]
    N_z_axis = TF_mat[:3,2] #R_z
    R_start_to_world = np.matrix([[1,0,0],[0,np.cos(th_s),-np.sin(th_s)],[0,np.sin(th_s), np.cos(th_s)]]) #Rotation of rotated frame to world frame: z-vector in new frame to world frame (R_zs_to_world)
    N_s_z_axis = R_start_to_world*TF_mat[:3,2]

    N_y_axis = self.skew_mat(N_z_axis)*N_x_axis
    N_s_y_axis = self.skew_mat(N_s_z_axis)*N_x_axis
    #Define Ns_z_axis (with start angle, rotate about this N_x_axis cw)
    N_x_pos = TF_mat[0,3]
    N_y_pos = TF_mat[1,3]
    N_z_pos = TF_mat[2,3] + H

    #make this a transform
    N_R = np.hstack([N_x_axis, N_y_axis, N_z_axis])
    N_d = np.matrix([N_x_pos,N_y_pos,N_z_pos]).T
    bottom_row = np.matrix([0,0,0,1])
    N_TF = np.vstack([np.hstack([N_R, N_d]),bottom_row])
    #Make start pose transform
    Ns_R = np.hstack([N_x_axis, N_y_axis, N_s_y_axis])
    Ns_d = np.matrix([N_x_pos,N_y_pos,N_z_pos]).T
    bottom_row = np.matrix([0,0,0,1])
    Ns_TF = np.vstack([np.hstack([Ns_R, Ns_d]),bottom_row])
    #Get transforms as pos/quat
    N_tf_dict = self.convert_TF_to_pos_quat(TF=N_TF)
    Ns_tf_dict = self.convert_TF_to_pos_quat(TF=Ns_TF)
    #Make transform frames (w.r.t. the base, as this is reset for every new container)
    N_transform = self.TF_to_transform_stamped(tf_dict=N_tf_dict, parent_id=self.world_frame, child_id="Nominal_aligned")
    Ns_transform = self.TF_to_transform_stamped(tf_dict=Ns_tf_dict, parent_id=self.world_frame, child_id="Nominal_start")

    #Pub the static transforms (and return them to the caller)
    if pub_static_transforms:
      self.static_pub_frame(transform=N_transform)
      self.static_pub_frame(transform=Ns_transform)
      rospy.loginfo("published the new nominal static frames")

    nom_frames = {"N":N_transform, "Ns":Ns_transform}
    return nom_frames

  def TF_to_transform_stamped(self,tf_dict=None,TF_mat=None, parent_id=None, child_id="child_TF_unnamed"):
    if type(parent_id) == type(None):
      parent_id = self.world_frame
    if type(TF_mat) != type(None):
      tf_dict = self.convert_TF_to_pos_quat(TF=N_TF)
    elif type(tf_dict) == type(None):
      rospy.loginfo("pass either pos/quat or TF_mat")
      return geometry_msgs.msg.TransformStamped()
    pos = tf_dict["position"]
    quat = tf_dict["quaternion"]
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_id #this is the parent frame
    transform.child_frame_id = child_id
    transform.transform.translation.x = pos[0]
    transform.transform.translation.y = pos[1]
    transform.transform.translation.z = pos[2]
    transform.transform.rotation.x =  quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w =  quat[3]
    return transform

  def convert_TF_to_pos_quat(self,TF=None):
    #This function takes a TF function and returns the pose,quat
    #Quat first
    R = TF[:3,:3]
    skew_mat = scipy.linalg.logm(R)
    v_ang = np.matrix([skew_mat[2,1],skew_mat[0,2],skew_mat[0,1]]).T
    ang = np.linalg.norm(v_ang)
    if ang == 0:
      qx = 0.0
      qy = 0.0
      qz = 0.0
      qw = 1.0
    else:
      v = np.divide(v_ang,ang)
      qx = v.item(0)*np.sin(ang/2)
      qy = v.item(1)*np.sin(ang/2)
      qz = v.item(2)*np.sin(ang/2)
      qw = np.cos(ang/2)
    #Position
    d = TF[:3,3]
    trans_dict = {"position":[d.item(0),d.item(1),d.item(2)], "quaternion":[qx,qy,qz,qw]}
    return trans_dict

def main():
  rospy.init_node('full_body_pour_node')

  if rospy.has_param('robot_type'):
    robot_type = rospy.get_param("robot_type")

  print "robot type", robot_type
  if robot_type in "iiwa":
    #For Kuka
    base_link= 'iiwa_link_0'
    end_link = 'iiwa_link_ee'
    joint_callback_key_frame='iiwa_joint_1'  #this joint should be in callback message
    joint_state_callback_msg = '/iiwa/joint_states'
    end_effector_parent_joint = 'iiwa_joint_ee'
    # os.system("export ROS_NAMESPACE=/iiwa") #This should be done from launch file
  elif robot_type in "sawyer":
    #For Sawyer
    base_link= 'right_arm_base_link'
    end_link = 'right_l6'
    joint_callback_key_frame='right_j0'
    joint_state_callback_msg = '/robot/joint_states'
    end_effector_parent_joint = 'right_j6'

    print "Waiting for setup"
    rospy.wait_for_message('/setup_finished', Empty)
    print "Setup is complete"
    rospy.sleep(2)


  cup_transform="cup_edge"
  joint_state_callback_msg = 'joint_states'

  cls_obj = FullBodyPourClass(base_link=base_link, end_link=end_link, cup_link=cup_transform, joint_callback_key_frame=joint_callback_key_frame, robot_type=robot_type)


  #Estabilish external Transforms (Cup edge, Tag pose)
  cls_obj.pub_static_transform_temp() #THIS IS TEMPORARILY A static transform publisher
  rospy.sleep(2.0) #give it a moment to make static transforms

  #Publish the setup frames (guard frame, beaker frame, edge frame, edge pour line)
  cls_obj.obtain_pouring_setup_terms()
  rospy.sleep(2.0)

  #Find the initial nominal frames
  start_angle = 30*np.pi/180.  #This is passed in from state machine
  nom_frames = cls_obj.generate_desired_start_pose_for_pour(start_angle=start_angle)


  rospy.loginfo("about to obtain the static transforms")
  success_bool = False
  while not rospy.is_shutdown() and success_bool == False:
    success_bool = cls_obj.obtain_cup_transform(parent_link = end_link, parent_joint= end_effector_parent_joint)

  rospy.loginfo("obtained the static transforms")

  #Publish the constant joint angle (will be adjusted to listen to actual commanded joint in real scenario)
  # cls_obj.publish_constant_angular_vel()


  #Move to the pour pose (needed in testing only)
  if robot_type in "iiwa":
    move_success_bool = False
    rospy.loginfo("made it to moveit part")
    while not rospy.is_shutdown() and move_success_bool == False:
      move_success_bool = cls_obj.move_to_pour_pose_iiwa()
  if robot_type in "sawyer":
    print "move sawyer to start"
    success = cls_obj.move_to_pour_pose_sawyer()

  rospy.loginfo("Move to start Succeeded")
  rospy.sleep(2.5)

  #This function stores the frame fo the cup to be used for error attentuation (fb & ff(current method))
  cls_obj.set_pouring_nominal_static_pose(nom_frame=nom_frames["Ns"])
  cls_obj.reset_starting_angle()



  sub = rospy.Subscriber(joint_state_callback_msg, JointState, cls_obj.joint_state_callback, queue_size=1)
  sub_cntrl = rospy.Subscriber('/pouring_control/Hybrid_controller_data',HybCntrlData,cls_obj.hyb_data_callback, queue_size=1)
  omega_sub = rospy.Subscriber('/omega_desired', HybCntrlData, cls_obj.hyb_data_callback, queue_size=1)


  while not rospy.is_shutdown():

    cls_obj.publish_transforms()
    rospy.sleep(1)
  rospy.spin()

if __name__ == '__main__':
  main()

