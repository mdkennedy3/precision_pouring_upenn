#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Transform

class beakerstruct(object):
  def __init__(self):
    self.tag_id = None
    self.beaker_id = None
    self.transform = Transform()
    
  def populate_struct(self,beaker=None):
      self.tag_id = beaker["tag_id"]
      self.beaker_id = beaker['beaker_id']
      self.transform.translation.x = beaker['x']
      self.transform.translation.y = beaker['y']
      self.transform.translation.z = beaker['z']
      self.transform.rotation.x = beaker['qx']
      self.transform.rotation.y = beaker['qy']
      self.transform.rotation.z = beaker['qz']
      self.transform.rotation.w = beaker['qw']


def main():
  rospy.init_node("python_test_param_node")

  beaker_frames_param = rospy.get_param('beaker_edge_frames')
  """
  Has form:
  [{'beaker_id': 'blue_cylinder',
            'qw': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'tag_id': 21,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0},
           {'beaker_id': 'red_cylinder',
            'qw': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'tag_id': 22,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0},
           {'beaker_id': 'green_cylinder',
            'qw': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'tag_id': 23,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0},
           {'beaker_id': 'black_cylinder',
            'qw': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'tag_id': 24,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0}]
        """  


  blue_cylinder = beakerstruct()
  red_cylinder = beakerstruct()
  green_cylinder = beakerstruct()
  black_cylinder = beakerstruct()


  for beaker in beaker_frames_param:
    if beaker['beaker_id'] in "blue_cylinder":
      blue_cylinder.populate_struct(beaker=beaker)
    elif beaker['beaker_id'] in "red_cylinder":
      red_cylinder.populate_struct(beaker=beaker)
    elif beaker['beaker_id'] in "green_cylinder":
      green_cylinder.populate_struct(beaker=beaker)
    elif beaker['beaker_id'] in "black_cylinder":
      black_cylinder.populate_struct(beaker=beaker)
    else:
      rospy.loginfo("beaker not queried")

  print "the beakers are "
  print "blue: ",blue_cylinder.beaker_id
  print "red: ",red_cylinder.beaker_id
  print "green: ",green_cylinder.beaker_id
  print "black: ",black_cylinder.beaker_id


  rospy.spin()



if __name__ == '__main__':
  main()