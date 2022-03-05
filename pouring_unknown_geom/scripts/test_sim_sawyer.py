#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from intera_core_msgs.msg import JointCommand

import intera_interface


def main():
  rospy.init_node('test2')
  pub1 = rospy.Publisher('/robot/right_joint_velocity_controller/joints/right_j6_controller/command',Float64,queue_size=1,latch=True)

  jnt = 'right_j6'
  val = -2.0
  cmd_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=1, latch=False)

  dt = 0
  rospy.sleep(0.3)
  ts = rospy.Time().now()
  
  right_arm = intera_interface.limb.Limb("right")

  print "before while loop"
  while not rospy.is_shutdown() and dt < 10.0: 
    pub_msg = JointCommand()
    pub_msg.names = [jnt]
    pub_msg.velocity = [val]
    pub_msg.header.stamp = rospy.Time.now()
    #cmd_pub.publish(pub_msg)
    pub1.publish(val)
    tf = rospy.Time().now()
    dt = tf.to_sec() - ts.to_sec()
    print "dt:",dt, tf.to_sec(), ts.to_sec()

    cmd = dict(zip([jnt],[val]))
    right_arm.set_joint_velocities(cmd)

if __name__ == '__main__':
  main()


