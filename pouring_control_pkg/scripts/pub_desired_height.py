#!/usr/bin/env python2
import rospy
import sys, getopt
from std_msgs.msg import Float64


def main(argv):

    try: 
        opts, args = getopt.getopt(argv, "hd:",["des_height="])
    except getopt.GetoptError:
        print "pub_desired_height.py -d value"

    ml_height = 100. #ml

    for opt, arg in opts:
        if opt == '-h':
            print "pub_desired_height.py -d value"
            # sys.exit()
        elif opt in ["-d","--des_height"]:
            print "req arg: ", arg
            ml_height = float(arg) 




    #the other should be a service
    rospy.init_node('pub_des_height')
    pub = rospy.Publisher('/des_height', Float64, queue_size=1, latch=False)

    des_height = Float64()
    
    Area = 41.66 #cm**2
    if rospy.has_param('recieving_container_area'):
      Area = float(rospy.get_param("recieving_container_area"))

    des_height.data = ml_height/Area #2.4 #units are cm, 2.4 for 100ml
    #the relationship btw cm, ml is from D=7.28cm, A = 41.66cm**2,    100.0
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(des_height)
        r.sleep()



if __name__ == '__main__':
    main(sys.argv[1:])
