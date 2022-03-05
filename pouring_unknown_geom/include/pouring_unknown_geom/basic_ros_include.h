#pragma once //This checks to make sure includes aren't done more than once

#include <ros/ros.h>
#include <Eigen/Dense>
//#include <Eigen/Eigenvalues>
//#include <Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
//#include <time.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <map> //dictionary equivalent: unordered map: <unordered_map> c++11
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Header.h>

//#define PI 3.14159265
//M_PI //from Cmath


