#!/usr/bin/env python
import rospy
import intera_interface


def main():
  rospy.init_node('turn_head')
  head = intera_interface.Head()
  head.set_pan(22.0)


if __name__ == '__main__':
  main()



