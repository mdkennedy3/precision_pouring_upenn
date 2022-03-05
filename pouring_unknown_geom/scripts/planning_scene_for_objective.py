#!/usr/bin/env python
import rospy
import numpy as np
import scipy
from scipy import linalg
import tf
from tf import TransformerROS
import tf2_ros
import geometry_msgs.msg

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

from std_srvs.srv import *


class PlanningSceneObjectiveProtectionClass(object):
  """docstring for PlanningSceneObjectiveProtectionClass
    Input:
      1. Base/world frame id  (default 'world')
      2. Camera frame id  (default: 'pg_13344889')
      3. Beaker tag frame id (default: 'cylinder_Tag')
      4. Background tag(s) frame id(s)  (default(s): tag_82, tag_83) *all of these define the same TF for the backboard and lightbox/scale

      5. Beaker params and beaker frame (published)
        5a. Beaker radius
        5b. Beaker frame (default: 'beaker_frame')

    Method:
      Generate 4 planning scene cubes (baesd on inputs as *# (priority 1) --> *# (priority 2))
      A: Around camera frame  *2
      B: Around Beaker  *3 --> *4
      C: Around objective box (lightbox)  *4 --> *3
      D: Around Background (backboard)    *4 --> *3

      1. This node runs at a specified rate, expects the frames of interest to be published, if not, then at least the world and camera frame must be available
      2. If the other frames are not presented, then either their last pose (saved from previous observation) or a generic (only upon starting) are used to generate the cubes

  """
  def __init__(self):
    self.broadcast_static_transform = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.tfros = TransformerROS()
    #persistent frames
    self.beaker_frame_transform = None

    #frame names:
    self.camera_frame_id = "pg_13344889"
    self.world_frame_id = "world"

    self.beaker_frame_name = "beaker_frame"
    self.receiving_beaker_tag_name = "cylinder_Tag"
    self.beaker_edge_frame = "beaker_edge"
    self.backboard_tag_bot_left_frame = "tag_82"
    self.backboard_tag_bot_right_frame = "tag_83"
    #transforms to maintain
    self.camera_world_transform = None
    self.beaker_world_transform = None
    self.backboard_bot_left_cam_transform = None
    self.backboard_bot_right_cam_transform = None
    self.scene_list = {'cam':False, 'beaker':False, 'backboard_82':False, 'backboard_83':False}
    self.tf_timeout = rospy.Duration(2.0)

  def get_params(self):
    #Get the params neccessary to define frames

    self.cup_edge_to_finger_distance = rospy.get_param('~cup_edge_to_finger_distance', 0.045)  #updated with measurements 4/20/18
    self.robot_finger_safety_distance = rospy.get_param('~robot_finger_safety_distance', 0.015)#updated with measurements 4/20/18
    self.z_world_beaker = rospy.get_param('~z_world_beaker', 0.21)#updated with measurements 4/20/18
    self.receiving_beaker_radius = rospy.get_param('~receiving_beaker_radius', 0.04)
    self.pour_past_edge_distance = rospy.get_param('~pour_past_edge_distance', 0.015)
    self.backboard_height = rospy.get_param('~backboard_height', 0.17)



    self.camera_frame_id = rospy.get_param('~camera_frame_id', 'pg_13344889')
    self.world_frame_id = rospy.get_param('~world_frame_id', 'world')

    rospy.logwarn("z_world_beaker %g receiving_beaker_radius %g", self.z_world_beaker, self.receiving_beaker_radius)

  def planning_scene_update(self):
    if type(self.camera_world_transform) == type(None):
      rospy.loginfo("no camera world transform, returning")
      return #no boxes can be drawn without the camera (camera/world connection)
    #Camera
    cam_planning_scene = PlanningSceneInterface(self.camera_frame_id)  #the frame with which all things are respect to
    cam_box_edge_length = 0.15
    cam_planning_scene.removeCollisionObject("camera_box")
    cam_planning_scene.addCube("camera_box",cam_box_edge_length,0.0,0.0,0.0)

    table_planning_scene = PlanningSceneInterface(self.world_frame_id)  #the frame with which all things are respect to
    table_box_edge_length = 2
    table_planning_scene.removeCollisionObject("table_box")
    table_planning_scene.addCube("table_box",table_box_edge_length,0.0,0.0,-0.99)

    #Beaker
    if type(self.beaker_world_transform) != type(None):
      beaker_planning_scene = PlanningSceneInterface(self.beaker_frame_name)  #the frame with which all things are respect to
      beaker_planning_scene.removeCollisionObject("beaker_cyl")
      beaker_cyl_height = 0.6
      beaker_cyl_rad = self.receiving_beaker_radius + 0.02
      beaker_planning_scene.addCylinder("beaker_cyl",beaker_cyl_height, beaker_cyl_rad, 0,0,-0.5*beaker_cyl_height)
    #Background objectives
    # backboard_box
    if type(self.backboard_bot_left_cam_transform) != type(None) and type(self.backboard_bot_right_cam_transform) != type(None):
      #use this to define the objective
      backboard_planning_scene = PlanningSceneInterface(self.camera_frame_id)
      backboard_planning_scene.removeCollisionObject("backboard")
      backboard_planning_scene.removeCollisionObject("objective_base")
      #The orientation is unusable due to the focus of the lense, but the position is descent
      tag_left_position = self.backboard_bot_left_cam_transform.transform.translation
      tag_right_position = self.backboard_bot_right_cam_transform.transform.translation
      mid_point = Vector3()
      mid_point.x = 0.5*(tag_left_position.x+tag_right_position.x)
      mid_point.y = 0.5*(tag_left_position.y+tag_right_position.y)
      mid_point.z = 0.5*(tag_left_position.z+tag_right_position.z)
      distance = np.linalg.norm(np.matrix([np.abs(tag_left_position.x-tag_right_position.x), np.abs(tag_left_position.y-tag_right_position.y), np.abs(tag_left_position.z-tag_right_position.z)]))
      bx = distance
      by = self.backboard_height #0.3 #30cm tall
      bz = 0.05
      px = mid_point.x
      py = mid_point.y - by*0.5
      pz = mid_point.z+bz*0.5
      backboard_planning_scene.addBox("backboard",bx,by,bz,px,py,pz)
      #add protected base under the objective
      base_cube = 0.3# mid_point.z
      pcx = 0.0
      pcy = mid_point.y + 0.5*base_cube
      pcz = 0.5*base_cube
      backboard_planning_scene.addCube("objective_base",base_cube, pcx,pcy,pcz)

  def get_latest_transforms(self):
    #try camera world transform
    cam_trans = self.generic_obtain_transform(target_frame=self.world_frame_id, source_frame=self.camera_frame_id)
    return_msg = {'bool':True,'fail_mode':[]}
    if not cam_trans["success_bool"]:
      rospy.loginfo("camera_transform doesnt exist in tf-tree")
      return_msg['bool'] = False
      return_msg['fail_mode'].append("no camera tf")
      return return_msg
    self.camera_world_transform = cam_trans["transform"]
    #Get beaker transform
    beaker_trans = self.generic_obtain_transform(target_frame=self.world_frame_id, source_frame=self.beaker_frame_name)
    if beaker_trans["success_bool"]:
      self.beaker_world_transform = beaker_trans["transform"]
    else:
      return_msg['fail_mode'].append("no camera tf")
    #Get objective transform
    #bottom left
    backboard_trans_bot_left = self.generic_obtain_transform(target_frame=self.camera_frame_id, source_frame=self.backboard_tag_bot_left_frame)
    if backboard_trans_bot_left["success_bool"]:
      self.backboard_bot_left_cam_transform = backboard_trans_bot_left["transform"]
    else:
      return_msg['fail_mode'].append("no left backboard tf")
    #bottom right
    backboard_trans_bot_right = self.generic_obtain_transform(target_frame=self.camera_frame_id, source_frame=self.backboard_tag_bot_right_frame)
    if backboard_trans_bot_right["success_bool"]:
      self.backboard_bot_right_cam_transform = backboard_trans_bot_right["transform"]
    else:
      return_msg['fail_mode'].append("no right backboard tf")
    return return_msg

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

  def planning_scene_service_callback(self, req):
    #cls_obj.get_params()
    return_msg = self.get_latest_transforms()
    resp = TriggerResponse()
    resp.success = True
    if not return_msg['bool']:
      resp.success = False
      resp.message = str(return_msg['fail_mode'][0])
      return resp
    if len(return_msg["fail_mode"]) > 0:
      str_msg = ' '.join(return_msg["fail_mode"])
      resp.message = str(str_msg)
    self.planning_scene_update()
    rospy.loginfo("updating planning scene")
    return resp

  def planning_scene_objective_service(self):
    s = rospy.Service('objective_projection_planning_scene_service', Trigger, self.planning_scene_service_callback)
    rospy.spin()

def main():
  rospy.init_node('objective_planning_scene_protection')

  cls_obj = PlanningSceneObjectiveProtectionClass()
  #get params
  cls_obj.get_params()

  run_as_service = rospy.get_param('~run_as_service', False)

  if run_as_service:
    rospy.logwarn("Running objective planning as service server")
    cls_obj.planning_scene_objective_service()
  else:
    #Get latest transforms
    while not rospy.is_shutdown():
      cls_obj.get_latest_transforms()
      cls_obj.planning_scene_update()
      rospy.loginfo("updating planning scene")

if __name__ == '__main__':
  main()
