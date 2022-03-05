#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>//To extract bounds of point cloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>
#include <pouring_unknown_geom/ContainerEdgeProfile.h>
#include "pouring_unknown_geom/ContainerEdgeProfileSrv.h"
#include "pouring_unknown_geom/ShowFinalEdgeProfileSrv.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h> //http://docs.pointclouds.org/1.0.0/transforms_8h.html

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/common/transforms.h>

#include <pcl_ros/transforms.h>

//For passthrough filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
//Eigen
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

//For linspace (the main script depends on this as well)
#include <pouring_unknown_geom/linspace_fnct.h>

//For static transform publishing
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


//For container cylinder visualizations
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

//For using Gpy python service
#include <pouring_msgs/GPEdgeProfileGenerator.h>


typedef pcl::PointXYZ PointT;

struct CylinderData {
  double radius, height;
}; //default is public, class needs to be speciefied public explcitly

struct PlaneEqnStruct {
  double x, y, z, d;
}; //default is public, class needs to be speciefied public explcitly


geometry_msgs::Vector3 NormalizeVector(geometry_msgs::Vector3 inpt_vector)
{
  geometry_msgs::Vector3 normed_vect;
  Eigen::Vector3d vect, vect_normed;
  tf::vectorMsgToEigen(inpt_vector, vect);
  vect_normed = vect.normalized();
  tf::vectorEigenToMsg(vect_normed,normed_vect);
  return normed_vect;
}   
    
geometry_msgs::Vector3 PerpVectorNormed(geometry_msgs::Vector3 vect1, geometry_msgs::Vector3 vect2)
{   
 // This find the vector perp to V1 given V2 (not aligned with V1): V1_perp = V2 - V1*(V1 dot V2)
 geometry_msgs::Vector3 vect1_normed = NormalizeVector(vect1);
 geometry_msgs::Vector3 vect2_normed = NormalizeVector(vect2);
 Eigen::Vector3d v1, v2, v1perp;
 tf::vectorMsgToEigen(vect1_normed,v1);
 tf::vectorMsgToEigen(vect2_normed,v2);
 v1perp = v2 - v1*v1.dot(v2);
 geometry_msgs::Vector3 perp_vect;
 tf::vectorEigenToMsg(v1perp,perp_vect);
 return perp_vect;
}

void AddContainerPoints( std::vector <double> angles, geometry_msgs::Vector3 cup_axis, geometry_msgs::Vector3 cup_axis_perp, geometry_msgs::Vector3 cup_axis_point, double curr_radius, pcl::PointCloud<PointT>::Ptr &cloudPtr)
{
  //Loop:
  for(int idx = 0; idx < angles.size(); ++idx)
  {
  //extract current angle, convert axis vector and perp vector to eig vects
  double curr_angle = angles[idx]; 
  Eigen::Vector3d cup_axis_eig, cup_axis_perp_eig; 
  tf::vectorMsgToEigen(cup_axis,cup_axis_eig);
  tf::vectorMsgToEigen(cup_axis_perp,cup_axis_perp_eig);
  //1. For each angle, rotate the perp vector by that radian 
  Eigen::Affine3d transform_vect(Eigen::AngleAxisd(curr_angle,cup_axis_eig));
  Eigen::Vector3d rot_cup_axis_perp_eig = transform_vect.rotation()*cup_axis_perp_eig; //resulting vector is also unit vector
  //2. populate a point cloud that is the axis point + the rotated_vector*radius
  PointT new_pt;
  new_pt.x = cup_axis_point.x + curr_radius*rot_cup_axis_perp_eig.x();
  new_pt.y = cup_axis_point.y + curr_radius*rot_cup_axis_perp_eig.y();
  new_pt.z = cup_axis_point.z + curr_radius*rot_cup_axis_perp_eig.z();
  //Send this point to the point cloud
  cloudPtr->points.push_back(new_pt); 
  }
}

void PlotCylinderIntersectionPoints(visualization_msgs::Marker &intersection_pt_markers, Eigen::Vector3d &container_clust_pt_eig, std::string &frame)
{
  intersection_pt_markers.header.frame_id = frame;
  intersection_pt_markers.id = 1;
  intersection_pt_markers.type=2;
  intersection_pt_markers.pose.position.x = container_clust_pt_eig.x();
  intersection_pt_markers.pose.position.y = container_clust_pt_eig.y();
  intersection_pt_markers.pose.position.z = container_clust_pt_eig.z();
  intersection_pt_markers.scale.x = 0.01;
  intersection_pt_markers.scale.y = 0.01;
  intersection_pt_markers.scale.z = 0.01;
  intersection_pt_markers.color.a = 1.0;
  intersection_pt_markers.color.r = 1.0;
}


void FindCylinderIntersectionPoint(sensor_msgs::PointCloud2 cyl_slices_filt, 
                                   pcl::ModelCoefficients coefficients_cylinder,
                                   pcl::ModelCoefficients coefficients_table, 
                                   geometry_msgs::Vector3 &container_table_base_point,
                                   visualization_msgs::Marker &intersection_pt_markers)
{
  /** 
  Algorithm:
  Given: 1. Cylinder pt on axis p1 and axis vector vc 2. cloud points c_i 3. table plane equation ; Find: point on table below cylinder
  a) Find point in cluster of cloud on cylinder axis
    i) d_{i,avg} = avg([vc^T,p1^Tvc]*[c_i^T, 1]^T)
    ii) p2 = p1 + d_{i,avg}*vc
  b) Find point below this cluster point that is on the table 
    i) d_p2 = t_v*p2 + td   (plane eqn is such that t_vz >= 0)
    ii) p3 = p2 - d_p2 * t_v  
  */
  //First Orient Vectors correctly
  PlaneEqnStruct cyl_eqn, table_eqn;
  /*
  if(coefficients_cylinder.values[5] >= 0){
    cyl_eqn.x = coefficients_cylinder.values[3]; cyl_eqn.y = coefficients_cylinder.values[4]; cyl_eqn.z = coefficients_cylinder.values[5]; 
  }else{
    cyl_eqn.x = -coefficients_cylinder.values[3];  cyl_eqn.y = -coefficients_cylinder.values[4];  cyl_eqn.z = -coefficients_cylinder.values[5];
  }
  */
  cyl_eqn.x = coefficients_cylinder.values[3]; cyl_eqn.y = coefficients_cylinder.values[4]; cyl_eqn.z = coefficients_cylinder.values[5]; 
  cyl_eqn.d = -(coefficients_cylinder.values[0]*cyl_eqn.x + coefficients_cylinder.values[1]*cyl_eqn.y + coefficients_cylinder.values[2]*cyl_eqn.z);
  //Now for table coefficients
  if(coefficients_table.values[2] >=0){
    table_eqn.x = coefficients_table.values[0]; table_eqn.y = coefficients_table.values[1]; table_eqn.z = coefficients_table.values[2]; table_eqn.d = coefficients_table.values[3];
  }else{
    table_eqn.x = -coefficients_table.values[0]; table_eqn.y = -coefficients_table.values[1]; table_eqn.z = -coefficients_table.values[2]; table_eqn.d = -coefficients_table.values[3];
  }
  geometry_msgs::Vector3 cyl_axis_vect, table_vect;
  cyl_axis_vect.x = cyl_eqn.x; cyl_axis_vect.y = cyl_eqn.y; cyl_axis_vect.z = cyl_eqn.z;
  table_vect.x = table_eqn.x; table_vect.y = table_eqn.y; table_vect.z = table_eqn.z;
  Eigen::Vector3d cyl_axis_vect_eig, table_vect_eig;
  tf::vectorMsgToEigen(cyl_axis_vect, cyl_axis_vect_eig);
  tf::vectorMsgToEigen(table_vect, table_vect_eig);
  //A. Find point in cluster of cloud on cylinder axis
  //A0. Convert point clouds to easy to manipulate type
  pcl::PointCloud<PointT>  cyl_slices_filt_pcl;
  pcl::fromROSMsg(cyl_slices_filt, cyl_slices_filt_pcl);
  // std::cout << "frame of slice point cloud: " << cyl_slices_filt.header.frame_id << std::endl; #was cam frame
  //A1. Average distances of point cloud points from plane described
  double avg_cld_dist = 0.0; 
  for(int idx=0; idx<cyl_slices_filt_pcl.points.size(); ++idx){
    PointT curr_point = cyl_slices_filt_pcl.points[idx];
    avg_cld_dist += curr_point.x*cyl_eqn.x + curr_point.y*cyl_eqn.y + curr_point.z*cyl_eqn.z + cyl_eqn.d;
  }
  avg_cld_dist = avg_cld_dist/float(cyl_slices_filt_pcl.points.size()); //avg
  std::cout << "avg cld distance" << avg_cld_dist << std::endl;
  //A2. Calculate new point in the center of the cld given the axis & point on the axis
  Eigen::Vector3d cyl_axis_pt_eig, container_clust_pt_eig, container_base_pt_eig; //axis pt is generic point on axis, container clust point is point on axis in the middle of points which defined the axis, container base point is the point below the cluster point that intersects the table 
  cyl_axis_pt_eig.x() = coefficients_cylinder.values[0]; cyl_axis_pt_eig.y() = coefficients_cylinder.values[1]; cyl_axis_pt_eig.z() = coefficients_cylinder.values[2];
  container_clust_pt_eig = cyl_axis_pt_eig + avg_cld_dist*cyl_axis_vect_eig; //Find the cluster center point
  //B. Find point below this cluster point that is on the table
  //B1. Find distance cluster point is above the table
  double clust_pt_dist_above_table = container_clust_pt_eig.dot(table_vect_eig) + table_eqn.d;
  if(clust_pt_dist_above_table <0){
    ROS_WARN("clust point is below the table");
  }
  //B2. calculate point on table
  container_base_pt_eig = container_clust_pt_eig- clust_pt_dist_above_table*table_vect_eig;
  tf::vectorEigenToMsg(container_base_pt_eig,container_table_base_point);
  //Plot the points that are the intersection of the cluster
  PlotCylinderIntersectionPoints(intersection_pt_markers, container_clust_pt_eig, cyl_slices_filt.header.frame_id);
}


void AddCylinderToMarkerArray(visualization_msgs::MarkerArray &marker_array, pcl::ModelCoefficients cyl_coeff, std::string camera_frame, int marker_id)
{
  visualization_msgs::Marker cyl_marker;
  cyl_marker.header.frame_id = camera_frame;
  cyl_marker.id = marker_id;
  cyl_marker.type = 0;
  cyl_marker.color.r = 1;
  cyl_marker.color.a = 1;
  cyl_marker.scale.x = 0.01; //shaft diam
  cyl_marker.scale.y = 0.03; //head diam
  // cyl_marker.scale.z = 0.04; //auto head length if unspecified
  //Construct the vector
  geometry_msgs::Point p1, p2;
  p1.x = cyl_coeff.values[0];
  p1.y = cyl_coeff.values[1];
  p1.z = cyl_coeff.values[2];
  if (cyl_coeff.values[5] >= 0){
    p2.x = cyl_coeff.values[0] + cyl_coeff.values[3];
    p2.y = cyl_coeff.values[1] + cyl_coeff.values[4];
    p2.z = cyl_coeff.values[2] + cyl_coeff.values[5];
  }else{
    p2.x = cyl_coeff.values[0] - cyl_coeff.values[3];
    p2.y = cyl_coeff.values[1] - cyl_coeff.values[4];
    p2.z = cyl_coeff.values[2] - cyl_coeff.values[5];
  }
  //Add the vector to marker
  cyl_marker.points.push_back(p1);
  cyl_marker.points.push_back(p2);
  //Add marker to the marker array
  marker_array.markers.push_back(cyl_marker);
}


void PlotTableVector(visualization_msgs::Marker &marker, geometry_msgs::Vector3 &table_point, pcl::ModelCoefficients table_coef, std::string camera_frame)
{
  marker.header.frame_id = camera_frame;
  marker.id = 0;
  marker.type = 0;
  marker.color.b = 1;
  marker.color.a = 1;
  marker.scale.x = 0.01;
  marker.scale.y = 0.03;
  geometry_msgs::Point p1, p2;
  p1.x = table_point.x;  p1.y = table_point.y;  p1.z = table_point.z;
  p2.x = p1.x + table_coef.values[0];  p2.y = p1.y + table_coef.values[1];  p2.z = p1.z + table_coef.values[2];
  marker.points.push_back(p1);  marker.points.push_back(p2);
}
