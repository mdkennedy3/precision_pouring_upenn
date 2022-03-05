#!/usr/bin/env python
import rospy
import numpy as np
import scipy
from scipy import linalg
import tf
from tf import TransformerROS
import tf2_ros
import geometry_msgs.msg
import math

from pouring_unknown_geom.srv import *

from std_srvs.srv import *

class PourFrameClass(object):
  """PourFrameClass
    This class allows the user to pass in the world frame and tag frame for pouring (either the transforms or the names for transforms to be calculated here)
    and
    This class is divided into two parts:
    A. Find Beaker frames
      1. Input: (Either transforms, or strings of frame-id for lookup in this script) with access variables "world":(transform/str),"tag":(transform/str), "beaker_height":float (type will be either i) geometry_msgs.msg._TransformStamped.TransformStamped or ii) str for world or tag )
        a) world frame (or arm base frame): (str of frame ID)
        b) world_is_camera_bool, if true, then assumed world is rotated about cam x-axis -90deg
        c) beaker tag frame: (str of frame ID )
        d) beaker world height (this value is a float) and is the z-height in world frame of the given receiving beaker on the objective
        e) receiving_beaker_radius (beaker radius): (float) (m)
        f) pour_past_edge_distance (beaker pour past edge distance, distance from beaker edge to cup transform in plane of beaker): (float) (m)
        g) pour side (str) (optional), ['left' or 'right']
      2. Given these frames, calculate the beaker frame B (position determined by tag, height given, frame orientation same as world)
      3. Given B (beaker),W(world),T(tag) frames, find the edge frame (E), and pouring line frame (Ep) which makes the assumption the center of the beaker is at a reachable distance
      hence the edge that is also this distance from the world on the chosen pour side (default left)

    B. Find start pose (nominal frames)
      1. Input:
        a) frames (beaker, world, tag, edge, pouring line)
        b) Cup edge frames (edge of pouring cup): (str of frame ID or geometry_msgs.msg.TransformStamped)  (cup frame: C)
        c) Cup edge to finger distance (distance in cup frame to pouring edge (along cup y-axis)): (float) (m)
        d) finger safety distance (additional distance for guard frame to protect finger) (will be applied along cup y-axis): (float) (m)
        e) starting angle (desired starting tilt of pourer): (float) (rad)
      2. Given cup edge frame, finger distance and safety distance, calculate guard frame w.r.t. the cup frame (guard frame: G)
      3. Given G,C,E,Ep, calculate the nominal pose N (z points same as world z), and starting pose for frame C: Ns

  """
  def __init__(self):
    self.broadcast_static_transform = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.tfros = TransformerROS()
    #frame names:
    self.world_frame = "world" #string
    self.beaker_frame_name = "beaker_frame"
    self.cup_edge_frame = "cup_edge"
    self.receiving_beaker_tag_name = "cylinder_Tag"
    self.beaker_edge_frame = "beaker_edge"
    self.beaker_edge_line_frame ="beaker_edge_line"
    self.nominal_frame = "nominal_aligned"
    self.nominal_start_frame = "nominal_start"

    #persistent frames
    self.beaker_frame_transform = None
    self.beaker_edge_frame_transform = None
    self.beaker_pourline_frame_transform = None
    self.guard_frame_transform = None
    #constant variables for frames
    # self.cup_edge_to_finger_distance = 0.1 #m
    # self.robot_finger_safety_distance = 0.02 #m
    # self.z_world_beaker = 0.11 #m
    # self.receiving_beaker_radius = 0.04  #m
    # self.pour_past_edge_distance = 0.015  #m
    # self.pour_side ="left" #which side will the beaker be approached (from world frame perspective)

    self.start_angle = math.radians(30)
    self.tf_timeout = rospy.Duration(2.0)

    #Get the params neccessary to define frames
    self.cup_edge_to_finger_distance = rospy.get_param('~cup_edge_to_finger_distance', 0.045)  #updated with measurements 4/20/18
    self.robot_finger_safety_distance = rospy.get_param('~robot_finger_safety_distance', 0.015)#updated with measurements 4/20/18
    self.z_world_beaker = rospy.get_param('~z_world_beaker', 0.21)#updated with measurements 4/20/18
    self.receiving_beaker_radius = rospy.get_param('~receiving_beaker_radius', 0.04)
    self.pour_past_edge_distance = rospy.get_param('~pour_past_edge_distance', 0.015)

    self.pour_side = rospy.get_param('~pour_side', 'left')

    self.robot_name = rospy.get_param('~robot_name', 'iiwa')  #updated with measurements 4/20/18


  def make_camera_world_link(self, camera_frame="camera_link"):
    camera_frame_id = camera_frame  #It is expected that the camera frame id is world if this flag is true
    theta_rotate_camera_to_world = 90*np.pi/180
    world_camera_transform = geometry_msgs.msg.TransformStamped()
    world_camera_transform.header.stamp = rospy.Time.now()
    world_camera_transform.header.frame_id = camera_frame_id #this is the parent frame (camera)
    world_camera_transform.child_frame_id = 'world'
    world_camera_transform.transform.translation.x = 0.0
    world_camera_transform.transform.translation.y = 0.0
    world_camera_transform.transform.translation.z = 0.0
    world_camera_transform.transform.rotation.x = np.sin(theta_rotate_camera_to_world/2.)
    world_camera_transform.transform.rotation.y = 0
    world_camera_transform.transform.rotation.z = 0
    world_camera_transform.transform.rotation.w = np.cos(theta_rotate_camera_to_world/2.)
    self.static_pub_frame(transform=world_camera_transform)
    rospy.sleep(2.0)
  def make_frame_world_link(self, synonomous_world_frame="base_link"):
    parent_frame_id = synonomous_world_frame  #assumes no rotation is required, just need frame named world
    theta_rotate_camera_to_world = -90*np.pi/180
    world_camera_transform = geometry_msgs.msg.TransformStamped()
    world_camera_transform.header.stamp = rospy.Time.now()
    world_camera_transform.header.frame_id = parent_frame_id #this is the parent frame (camera)
    world_camera_transform.child_frame_id = 'world'
    world_camera_transform.transform.translation.x = 0.0
    world_camera_transform.transform.translation.y = 0.0
    world_camera_transform.transform.translation.z = 0.0
    world_camera_transform.transform.rotation.x = 0
    world_camera_transform.transform.rotation.y = 0
    world_camera_transform.transform.rotation.z = 0
    world_camera_transform.transform.rotation.w = 1.0
    self.static_pub_frame(transform=world_camera_transform)
    rospy.sleep(2.0)

  def obtain_beaker_frames(self, world_frame=None, beaker_tag_frame=None,beaker_frame_name=None, beaker_world_height=None, receiving_beaker_radius=None, pour_past_edge_distance=None, pour_side=None):
    '''
    This function accepts the frame names for world, beaker tag, and params for beaker dimensions (its radius, height in the world and distance for pouring line)
    '''
    #1. Set Parameters
    if type(world_frame) != type(None): self.world_frame = world_frame #Set the world frame to what's passed in (assumes z-axis points upward)
    if type(beaker_frame_name) != type(None): self.beaker_frame_name = beaker_frame_name
    if type(beaker_tag_frame) != type(None): self.receiving_beaker_tag_name = beaker_tag_frame
    #set beaker dimensions
    if type(beaker_world_height) != type(None): self.z_world_beaker = beaker_world_height  #m
    if type(receiving_beaker_radius) != type(None): self.receiving_beaker_radius = receiving_beaker_radius #m
    if type(pour_past_edge_distance) != type(None): self.pour_past_edge_distance = pour_past_edge_distance  #m
    #2. obtain transform from tag to world  #beaker frame (this is constant for a stationary recieving beaker)

    trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame=self.receiving_beaker_tag_name)
    if not trans_return["success_bool"]:
      return False
    # query_bool = False
    # while not query_bool and not rospy.is_shutdown():
    #   trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame=self.receiving_beaker_tag_name)
    #   query_bool = trans_return["success_bool"]
    tf_tag_to_world = trans_return["transform"]

    trans_return = self.generic_obtain_transform(target_frame=self.receiving_beaker_tag_name, source_frame=self.world_frame)
    if not trans_return["success_bool"]:
      return False
    # query_bool = False
    # while not query_bool and not rospy.is_shutdown():
    #   trans_return = self.generic_obtain_transform(target_frame=self.receiving_beaker_tag_name, source_frame=self.world_frame)
    #   query_bool = trans_return["success_bool"]
    tf_world_to_tag = trans_return["transform"]
    pos = (tf_tag_to_world.transform.translation.x, tf_tag_to_world.transform.translation.y, tf_tag_to_world.transform.translation.z)
    quat = (tf_tag_to_world.transform.rotation.x, tf_tag_to_world.transform.rotation.y, tf_tag_to_world.transform.rotation.z, tf_tag_to_world.transform.rotation.w )
    TF_mat_tag_to_world = np.matrix(self.tfros.fromTranslationRotation(pos,quat))
    rotated_z_vector_into_tag_frame = TF_mat_tag_to_world[:3,:3].T*np.matrix([0,0,(self.z_world_beaker - tf_tag_to_world.transform.translation.z)]).T
    #3. calculate/pub beaker pose
    self.beaker_frame_transform = geometry_msgs.msg.TransformStamped() #Keeping this wrt the tag is useful if the beaker shifts but the tag is still visible
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
    self.static_pub_frame(transform=self.beaker_frame_transform)
    # rospy.sleep(1.5)
    #4. Calculate intersection for E frame
    #obtain the new transform from between the beaker and world
    # query_bool = False
    # while not query_bool and not rospy.is_shutdown():
    trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame=self.beaker_frame_name)  #This can wait because essential frames are here
    query_bool = trans_return["success_bool"]
    if not query_bool:
      return False
    tf_beaker_to_world = trans_return["transform"]
    #calculate edge frames
    edge_frames =self.calculate_beaker_edge(transform=tf_beaker_to_world, beaker_radius=self.receiving_beaker_radius, beaker_frame_name=self.beaker_frame_name, beaker_edge_line_distance=self.pour_past_edge_distance)
    self.beaker_edge_frame_transform = edge_frames['edge']  # 'edge_line'
    self.beaker_pourline_frame_transform = edge_frames['edge_line']
    #Pub the beaker related transforms
    #5. Calculate the Ep frame
    self.static_pub_frame(transform=self.beaker_edge_frame_transform)
    self.static_pub_frame(transform=self.beaker_pourline_frame_transform)
    # rospy.sleep(2.0)

    return True

  def obtain_nomial_poses(self, start_angle=30*np.pi/180, cup_edge_frame=None, cup_edge_to_finger_distance=None, robot_finger_safety_distance=None,  pour_side=None, pub_static_transforms=True):
    '''
    Start angle is in radians
    Assumes that the following frames are available:
      1. C (cup), name: self.cup_edge_frame
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
    #set params
    if type(cup_edge_frame) != type(None): self.cup_edge_frame = cup_edge_frame
    if type(pour_side) != type(None): self.pour_side = pour_side
    if type(cup_edge_to_finger_distance) != type(None): self.cup_edge_to_finger_distance = cup_edge_to_finger_distance
    if type(robot_finger_safety_distance) != type(None): self.robot_finger_safety_distance = robot_finger_safety_distance

    #1. Find the intersecting point on the circle
    th_s = self.start_angle
    a = self.cup_edge_to_finger_distance+self.robot_finger_safety_distance
    b = self.pour_past_edge_distance
    H = a/np.sin(th_s) + b/np.tan(th_s)
    #2. Calculate the N, Ns
    '''
    Goal is to get nominal_start frame pose to start
      1. Nominal frame x-axis direction: Get the orientation from the  relative poses between E_line and the world
      2. Nominal frame z-axis direction: N_consistent same as world z-axis* / N_start (rotated by th_s from world z) *(however the pose to move to is not the same here (so one is start and one is nominal))
      3. Nominal frame x,y position: same as E_line
      4. Nominal frame z position: E_line + H
    '''
    #A. Obtain transform from E_line to world
    # query_bool = False
    # while not query_bool and not rospy.is_shutdown():
    trans_return = self.generic_obtain_transform(target_frame=self.world_frame, source_frame= self.beaker_edge_line_frame)
    query_bool = trans_return["success_bool"]

    if not query_bool:
      nom_frames = {"N":[], "Ns":[]}
      return nom_frames, False

    tf_E_line_to_world = trans_return["transform"]

    #convert to mat
    position_tuple_E_W = (tf_E_line_to_world.transform.translation.x,tf_E_line_to_world.transform.translation.y,tf_E_line_to_world.transform.translation.z)
    orientation_tuple_E_W = (tf_E_line_to_world.transform.rotation.x,tf_E_line_to_world.transform.rotation.y,tf_E_line_to_world.transform.rotation.z, tf_E_line_to_world.transform.rotation.w)
    TF_mat = np.matrix( self.tfros.fromTranslationRotation(position_tuple_E_W,orientation_tuple_E_W))
    #Find axis
    #a) x_axis
    N_x_axis = np.vstack([np.divide(TF_mat[:2,3], np.linalg.norm(TF_mat[:2,3])), 0])  #[dx,dy,0]
    N_z_axis = TF_mat[:3,2] #R_z
    N_y_axis = self.skew_mat(N_z_axis)*N_x_axis
    N_y_axis = np.divide(N_y_axis, np.linalg.norm(N_y_axis))
    N_s_x_axis = N_x_axis
    #rotate about x axis by theta degrees
    if self.pour_side in "left":
      R_for_z = self.rodruigez_formula(v=N_s_x_axis,th=-th_s)
    else:
      R_for_z = self.rodruigez_formula(v=N_s_x_axis,th=th_s)
    N_s_z_axis = R_for_z*TF_mat[:3,2]
    N_s_y_axis = self.skew_mat(N_s_z_axis)*N_s_x_axis
    N_s_y_axis = np.divide(N_s_y_axis, np.linalg.norm(N_s_y_axis))
    #Define Ns_z_axis (with start angle, rotate about this N_x_axis cw)
    N_x_pos = TF_mat[0,3]
    N_y_pos = TF_mat[1,3]
    N_z_pos = TF_mat[2,3] + H
    #make this a transform  (transpose is needed as they are defined with parent as world, but the rotation axis were designed from world perspective)

    if self.robot_name == 'iiwa':
        N_R = np.matrix(np.hstack([N_x_axis, N_y_axis, N_z_axis])).T
    elif self.robot_name == 'sawyer':
        N_R = np.matrix(np.hstack([-N_z_axis, -N_y_axis, -N_x_axis])).T

    N_d = np.matrix([N_x_pos,N_y_pos,N_z_pos]).T
    bottom_row = np.matrix([0,0,0,1])
    N_TF = np.vstack([np.hstack([N_R, N_d]),bottom_row])
    #Make start pose transform
    if self.robot_name == 'iiwa':
        Ns_R = np.matrix(np.hstack([N_s_x_axis, N_s_y_axis, N_s_z_axis])).T
    elif self.robot_name == 'sawyer':
        Ns_R = np.matrix(np.hstack([-N_s_z_axis, -N_s_y_axis, -N_s_x_axis])).T
    Ns_d = np.matrix([N_x_pos,N_y_pos,N_z_pos]).T
    bottom_row = np.matrix([0,0,0,1])
    Ns_TF = np.vstack([np.hstack([Ns_R, Ns_d]),bottom_row])
    #Get transforms as pos/quat
    N_tf_dict = self.convert_TF_to_pos_quat(TF=N_TF)
    Ns_tf_dict = self.convert_TF_to_pos_quat(TF=Ns_TF)

    #These transforms are defined in world frame, hence they are T_n_to_world, want T_n_to_tag. Already have T_Ep_to_W, so define wrt Ep
    T_N_to_Ep = np.linalg.inv(TF_mat) * N_TF
    T_Ns_to_Ep = np.linalg.inv(TF_mat) * Ns_TF

    N_tf_to_Ep_dict = self.convert_TF_to_pos_quat(TF=T_N_to_Ep)
    Ns_tf_to_Ep_dict = self.convert_TF_to_pos_quat(TF=T_Ns_to_Ep)

    #Make transform frames (w.r.t. the base, as this is reset for every new container)
    # N_transform = self.TF_to_transform_stamped(tf_dict=N_tf_dict, parent_id=self.world_frame, child_id=self.nominal_frame)
    # Ns_transform = self.TF_to_transform_stamped(tf_dict=Ns_tf_dict, parent_id=self.world_frame, child_id=self.nominal_start_frame)

    N_transform = self.TF_to_transform_stamped(tf_dict=N_tf_to_Ep_dict, parent_id=self.beaker_edge_line_frame, child_id=self.nominal_frame)
    Ns_transform = self.TF_to_transform_stamped(tf_dict=Ns_tf_to_Ep_dict, parent_id=self.beaker_edge_line_frame, child_id=self.nominal_start_frame)

    #Pub the static transforms (and return them to the caller)
    if pub_static_transforms:
      self.static_pub_frame(transform=N_transform)
      self.static_pub_frame(transform=Ns_transform)
      rospy.loginfo("published the new nominal static frames")
    nom_frames = {"N":N_transform, "Ns":Ns_transform}
    success_bool = True
    return nom_frames, success_bool

  def generic_obtain_transform(self,target_frame=None, source_frame=None):
    # query_bool = False
    # t_start = rospy.Time.now()
    # dt = 0.0
    # while not query_bool and not rospy.is_shutdown() and dt < self.tf_timeout:
    query_bool = True
    try:
      trans = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(), timeout=self.tf_timeout)  #target frame, source frame, time
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      trans = []
      query_bool = False
      rospy.logerr("failed to get transform %s and %s"%(target_frame,source_frame))
    trans_return = {"success_bool":query_bool, "transform":trans}
    # dt = rospy.Time.now().to_sec() - t_start.to_sec()
    return trans_return

  def circ_intersect_func(self,xb=None,yb=None,rb=None,Lb=None, pour_dir="left"):
    #xb,yb are beaker location in world frame, rb is beaker radius, and Lb is distance between world frame and beaker origin
    def query_point(x=None,y=None,xb=None,yb=None, rb=None,Lb=None):
      v1 = (x-xb)**2 + (y-yb)**2 - rb**2
      v2 = x**2 + y**2 - Lb**2
      return v1+v2
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
    final_set = np.array(combos)[combo_sort[:2]] #first two sorted are smallest
    #Figure out which is on the left and right of the beaker
    diff_vect1 = np.matrix([final_set[0][0] -xb, final_set[0][1] -yb, 0.0]).T
    diff_vect1_normed = np.divide(diff_vect1,np.linalg.norm(diff_vect1))
    diff_vect2 = np.matrix([final_set[1][0] -xb, final_set[1][1] -yb, 0.0]).T
    diff_vect2_normed = np.divide(diff_vect2,np.linalg.norm(diff_vect2))
    beaker_vect = np.matrix([xb,yb,0.0]).T
    beaker_vect_normed= np.divide(beaker_vect, np.linalg.norm(beaker_vect))
    beaker_skew_mat = self.skew_mat(beaker_vect_normed)
    cross_vect1 = beaker_skew_mat*diff_vect1_normed
    cross_vect2 = beaker_skew_mat*diff_vect2_normed
    if cross_vect1.item(2) > 0:
      left_pt = final_set[0]; right_pt = final_set[1]
    else:
      left_pt = final_set[1]; right_pt = final_set[0]
    #if pouring from the left pass the left, else pass the one on the right
    if pour_dir in "left": return left_pt #choose the one on the left of the beaker line
    else: return right_pt #choose the one on the right of the beaker line

  def calculate_beaker_edge(self,transform=None, beaker_radius=None, beaker_frame_name=None, beaker_edge_line_distance=None):
    #TF is beaker to world
    #1. Beaker location in xy world frame
    beaker_xy = np.matrix([transform.transform.translation.x,transform.transform.translation.y]).T
    #2. Find the radius (x,y) of beaker center in world
    xy_dist_beaker_world = np.linalg.norm(beaker_xy)
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
    #Get TF
    TF_mat_beaker_to_world = np.matrix(self.tfros.fromTranslationRotation(pos_beaker,quat_all))
    TF_mat_beaker_edge_to_world = np.matrix(self.tfros.fromTranslationRotation(pos_beaker_edge,quat_all))
    TF_mat_beaker_edge_to_beaker = np.linalg.inv(TF_mat_beaker_to_world) * TF_mat_beaker_edge_to_world
    #Return both edge, and edge line wrt the beaker transform instead of world
    B_E_px=TF_mat_beaker_edge_to_beaker[0,3]
    B_E_py=TF_mat_beaker_edge_to_beaker[1,3]
    B_E_pz=TF_mat_beaker_edge_to_beaker[2,3]
    B_E_qx = 0.0
    B_E_qy = 0.0
    B_E_qz = 0.0
    B_E_qw = 1.0
    beaker_edge_in_beaker_frame = geometry_msgs.msg.TransformStamped()
    beaker_edge_in_beaker_frame.header.stamp = rospy.Time.now()
    beaker_edge_in_beaker_frame.header.frame_id = beaker_frame_name #this is the parent frame
    beaker_edge_in_beaker_frame.child_frame_id = self.beaker_edge_frame
    beaker_edge_in_beaker_frame.transform.translation.x = B_E_px
    beaker_edge_in_beaker_frame.transform.translation.y = B_E_py
    beaker_edge_in_beaker_frame.transform.translation.z = B_E_pz
    beaker_edge_in_beaker_frame.transform.rotation.x =  B_E_qx
    beaker_edge_in_beaker_frame.transform.rotation.y = B_E_qy
    beaker_edge_in_beaker_frame.transform.rotation.z = B_E_qz
    beaker_edge_in_beaker_frame.transform.rotation.w =  B_E_qw
    #calculate edge frame E
    E_frame_xy_norm = np.linalg.norm(np.matrix([B_E_px,B_E_py]))
    E_edge_frame_xy_norm = E_frame_xy_norm - beaker_edge_line_distance
    E_frame_xy_angle = np.arctan2(B_E_py, B_E_px)
    #NOW calculate the E' frame for the line
    beaker_edge_line_in_beaker_frame = geometry_msgs.msg.TransformStamped()
    beaker_edge_line_in_beaker_frame.header.stamp = rospy.Time.now()
    beaker_edge_line_in_beaker_frame.header.frame_id = beaker_frame_name #this is the parent frame
    beaker_edge_line_in_beaker_frame.child_frame_id = self.beaker_edge_line_frame
    beaker_edge_line_in_beaker_frame.transform.translation.x = E_edge_frame_xy_norm*np.cos(E_frame_xy_angle)
    beaker_edge_line_in_beaker_frame.transform.translation.y = E_edge_frame_xy_norm*np.sin(E_frame_xy_angle)
    beaker_edge_line_in_beaker_frame.transform.translation.z = B_E_pz
    beaker_edge_line_in_beaker_frame.transform.rotation.x =  B_E_qx
    beaker_edge_line_in_beaker_frame.transform.rotation.y = B_E_qy
    beaker_edge_line_in_beaker_frame.transform.rotation.z = B_E_qz
    beaker_edge_line_in_beaker_frame.transform.rotation.w =  B_E_qw
    edge_frames = {"edge":beaker_edge_in_beaker_frame, "edge_line":beaker_edge_line_in_beaker_frame}
    return edge_frames

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

  def skew_mat(self,v):
    #Expects a np matrix/vector
    return np.matrix([[0,-v.item(2),v.item(1)],[v.item(2),0,-v.item(0)],[-v.item(1),v.item(0),0]])

  def static_pub_frame(self,transform=None):
    #this function pubs the provided static transform
    self.broadcast_static_transform.sendTransform(transform)
    rospy.loginfo("Pub static transform with name %s and parent link: %s"%(transform.child_frame_id,transform.header.frame_id))

  def rodruigez_formula(self,v=None,th=None):
    I=np.matrix(np.eye(3))
    w = self.skew_mat(v)
    ws = w*w
    R = I + np.sin(th)*w + (1-np.cos(th))*ws
    return R

  def define_guard_frame(self,cup_edge_frame=None,cup_edge_to_finger_distance=None, robot_finger_safety_distance=None):
    #set params
    if type(cup_edge_frame) != type(None): self.cup_edge_frame = cup_edge_frame
    if type(cup_edge_to_finger_distance) != type(None): self.cup_edge_to_finger_distance = cup_edge_to_finger_distance
    if type(robot_finger_safety_distance) != type(None): self.robot_finger_safety_distance = robot_finger_safety_distance
    #1. Calculate the G frame: Guard frame (G) defined by cup frame, cup to finger distance and robot finger safety distance (same orientation as cup frame)
    self.guard_frame_transform = geometry_msgs.msg.TransformStamped() #Guard frame: (this updates with every cup)
    self.guard_frame_transform.header.stamp = rospy.Time.now()
    self.guard_frame_transform.header.frame_id = self.cup_edge_frame #this is the parent frame
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

  def generate_artificial_cup_edge(self,cup_parent_jnt="right_wrist", cup_frame="cup_edge"):
    joint_cup_edge_transform = geometry_msgs.msg.TransformStamped() #Guard frame: (this updates with every cup)
    joint_cup_edge_transform.header.stamp = rospy.Time.now()
    print "cup parent joint", cup_parent_jnt
    joint_cup_edge_transform.header.frame_id = cup_parent_jnt #this is the parent frame
    joint_cup_edge_transform.child_frame_id = cup_frame
    joint_cup_edge_transform.transform.translation.x = 0.0
    joint_cup_edge_transform.transform.translation.y = 0.0
    joint_cup_edge_transform.transform.translation.z = 0.3
    joint_cup_edge_transform.transform.rotation.x = 0.0
    joint_cup_edge_transform.transform.rotation.y = 0.0
    joint_cup_edge_transform.transform.rotation.z = 0.0
    joint_cup_edge_transform.transform.rotation.w = 1.0
    #PUB THE STATIC TRANSFORM
    self.static_pub_frame(transform=joint_cup_edge_transform)
    rospy.sleep(2.0)

  def pouring_frames_service_callback(self,req):
    '''
    #For PouringFrames service
    #establish frame names
    if len(req.world_frame) > 0:
      pass
    if len(req.camera_frame) > 0:
      pass
    if len(req.beaker_tag_frame) > 0:
      pass
    #Determine pouring side
    if len(req.pour_side) > 0:
      pass
    #These must be provided: (default values in init)
    self.cup_edge_to_finger_distance = req.cup_edge_to_finger_distance
    self.robot_finger_safety_distance = req.robot_finger_safety_distance
    self.z_world_beaker = req.z_world_beaker
    self.receiving_beaker_radius = req.receiving_beaker_radius
    self.pour_past_edge_distance = req.pour_past_edge_distance

    #obtain beaker frames
    self.obtain_beaker_frames() #variables can be passed here as well (see above)
    #Obtain the nominal pose given a start angle
    #this is hardcoded, bad form...
    start_angle = req.start_angle #(start angle in radians)
    self.obtain_nomial_poses(start_angle= start_angle, pub_static_transforms=True)  #pub static transforms add these to the TF tree with a chain connecting to the beaker tag

    #if it is desired to generate the guard frame as well:
    if req.generate_guard_bool:
      req.pouring_cup_frame
    #Check if operations were successful and return boolean response
    resp = PouringFramesResponse() #this is a bool with success field
    resp.success = True
    return resp
    '''

    resp = TriggerResponse()

    #obtain beaker frames
    success_bool = self.obtain_beaker_frames() #variables can be passed here as well (see above)
    #Obtain the nominal pose given a start angle
    if not success_bool:
      resp.success = success_bool
      resp.message = "obtaining beaker tf failed"
      return resp

    nom_frames, success_bool = self.obtain_nomial_poses(start_angle= self.start_angle, pub_static_transforms=True)  #pub static transforms add these to the TF tree with a chain connecting to the beaker tag
    if not success_bool:
      resp.success = success_bool
      resp.message = "nominal_frame generation failed"
      return

    resp.success = True
    resp.message = "published beaker related frames"
    return resp

  def pouring_frames_service(self):
    # s = rospy.Service('pouring_frames_service', PouringFrames, self.pouring_frames_service_callback)
    s = rospy.Service('pouring_frames_service', Trigger, self.pouring_frames_service_callback)
    rospy.spin()

def main():

  rospy.init_node('pouring_frames_setup')
  cls_obj = PourFrameClass()
  run_as_service = rospy.get_param('~run_as_service', True)
  running_node_with_camera = rospy.get_param('~running_node_with_camera', True)

  start_angle_deg = rospy.get_param('~start_angle_deg', 30)
  start_angle = math.radians(start_angle_deg)
  cls_obj.start_angle = start_angle

  if run_as_service:
    rospy.logwarn("Running pouring frame as service server")
    cls_obj.pouring_frames_service()
  else:

    if running_node_with_camera:
      rospy.loginfo("Running node with camera")
      cls_obj.make_camera_world_link(camera_frame="pg_13344889")
    generate_artificial_cup_edge_and_guard = False
    if generate_artificial_cup_edge_and_guard:
      #This is there for testing (need to define the transform for this cup edge)
      cls_obj.generate_artificial_cup_edge(cup_parent_jnt="right_wrist", cup_frame="cup_edge") #w.r.t world frame
      #Define guard frame
      cls_obj.define_guard_frame() #simply requires cup frame exist and required params (robot finger safey distance and finger distance)
    #on other platforms can use this to establish world frame if it does not exist
    # cls_obj.make_frame_world_link(synonomous_world_frame="base_link")
    while not rospy.is_shutdown():
      #Obtain the beaker frames
      cls_obj.obtain_beaker_frames() #variables can be passed here as well (see above)
      #Obtain the nominal pose given a start angle

      cls_obj.obtain_nomial_poses(start_angle= start_angle, pub_static_transforms=True)  #pub static transforms add these to the TF tree with a chain connecting to the beaker tag
      rospy.sleep(2.0)

if __name__ == '__main__':
  main()