#!/usr/bin/env python
import rospy
import numpy as np
import tf
from tf import TransformerROS
import tf2_ros
import geometry_msgs.msg

rospy.init_node('tf_test')
broadcast_static_transform = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
broadcast_static_transform_second = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()

cup_transform = "cup_edge"
static_transformStamped = geometry_msgs.msg.TransformStamped()
static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "right_l6"
static_transformStamped.child_frame_id = cup_transform

static_transformStamped.transform.translation.x = -0.06
static_transformStamped.transform.translation.y = 0.06
static_transformStamped.transform.translation.z = 0.13
quat = tf.transformations.quaternion_from_euler(-np.pi/2.0,-np.pi/2,0.0)
static_transformStamped.transform.rotation.x = quat[0]
static_transformStamped.transform.rotation.y = quat[1]
static_transformStamped.transform.rotation.z = quat[2]
static_transformStamped.transform.rotation.w = quat[3]

broadcast_static_transform.sendTransform(static_transformStamped)
rospy.loginfo("send static transform with name %s and end link: %s"%(cup_transform, "right_l6"))

#2. Tag on recieving beaker transform
tag_transform = geometry_msgs.msg.TransformStamped()
tag_transform.header.stamp = rospy.Time.now()
tag_transform.header.frame_id = "right_arm_base_link" #this frame is w.r.t. the base frame
tag_transform.child_frame_id = "receiving_beaker_tag"
tag_transform.transform.translation.x = 0.90 #0.80  #required farther back for sawyer
tag_transform.transform.translation.y = 0.0
tag_transform.transform.translation.z = 0.06
tag_transform.transform.rotation.x = 0.0
tag_transform.transform.rotation.y = np.sin(-np.pi/4)
tag_transform.transform.rotation.z = 0.0
tag_transform.transform.rotation.w = np.cos(np.pi/4)
broadcast_static_transform_second.sendTransform(tag_transform)
rospy.loginfo("Send tag transform with name %s from parent link %s"%("receiving_beaker_tag","right_arm_base_link"))

while not rospy.is_shutdown():
  #static_transformStamped.header.stamp = rospy.Time.now()
  #broadcast_static_transform.sendTransform(static_transformStamped)
  rospy.sleep(1)
rospy.spin()
