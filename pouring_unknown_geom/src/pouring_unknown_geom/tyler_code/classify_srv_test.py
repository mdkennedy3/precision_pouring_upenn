#!/usr/bin/env python
# This sript tests the classification ROS service

import sys
import rospy
from pouring_unknown_geom.srv import *

def classify_container_client(a,v):
    rospy.wait_for_service('classify_container')
    try:
        classify_container = rospy.ServiceProxy('classify_container', ClassifyContainer)
        print("Set up service")
        resp1 = classify_container(angles=a,volumes=v)
        return resp1.coefs
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    a = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]
    v = [0.0, 0.0, 2.0, 2.0, 4.0, 6.0, 10.0, 16.0, 18.0, 24.0, 32.0]
    test = dict(angles=a, volumes=v)
    print("Classifying polynomial curve with Legendre coefficients {}".format(a))
    print("Classification results: {}".format(classify_container_client(a,v)))