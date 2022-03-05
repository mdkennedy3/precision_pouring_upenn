#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest


if __name__=='__main__':
  rospy.init_node("profile_scanner_triggers")
  print "Waiting for container edge"
  rospy.wait_for_service('/pouring_control/publish_container_profile_serv');
  t_req = TriggerRequest();
  cont_edge_srv = rospy.ServiceProxy('/pouring_control/publish_container_profile_serv',Trigger);
  print "Sent service request"
  resp = cont_edge_srv(t_req)
  print resp
  cont_edge_srv_reset = rospy.ServiceProxy('/pouring_control/reset_publish_container_profile_serv',Trigger);
  cont_edge_srv_reset()
  classification_srv = rospy.ServiceProxy('/pouring_control/model_prior_classificaton_serv', Trigger)
  classification_srv(TriggerRequest())

