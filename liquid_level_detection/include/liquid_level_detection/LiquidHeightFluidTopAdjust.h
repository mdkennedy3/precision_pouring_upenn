#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include <liquid_level_detection/LDHeight.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>



void TrueLiquidHeightCalc(liquid_level_detection::LDHeight &ld_height,
                          const int &img_pix_height,
                          const geometry_msgs::TransformStamped &tf_stamped_tag_to_cam,
                          const double &recieving_container_area,
                          const int &cylinder_bottom_pix,
                          const double apriltag_size,
                          const int &apriltag_length_pix)
{
  /*This function uses similar triangles to determine the true position of the height of the fluid
   * Given:
   * 1. Height of camera in cam frame: C_h (center of img)
   * 2. Apriltag dist from cam: d_tag (meters)
   * 3. Beaker diameter/radius:  B_len  (for background subtraction use diameter, for NN use radius)
   * 4. measured height (in aprltag frame): h_meas_pix(pix), h_meas_m(meters), comes from current ld_height.h_clust
   * (note that origin img is top left, y increases downward, x to the right)
   * Find:
   * if h_meas_pix < C_h (below center of cam frame), this is y axis
   *   wrong:: h_true = C_h - (d_tag/B_len)*h_meas_m
   *   correct:: using sim triangles, since C_h, h_meas are from based of platform, the simliar triangle is above these heights yeilding:
       h_true = C_h - (d_tag+B_len)/(d_tag)*(C_h - h_meas)
   * else:
   *   h_true = h_meas_m
   * Output:
   * adjusted ld_height.h_clust = h_true
   */
   //Obtain neccessary terms:
   double B_len = 2.0*std::sqrt(recieving_container_area/M_PI); //obtain the diameter [assumes NN which returns the center of the liquid]
   double d_tag = tf_stamped_tag_to_cam.transform.translation.z; //distance from cam
   double C_h =(((cylinder_bottom_pix- img_pix_height/2)*apriltag_size)/apriltag_length_pix); //meters
   double h_meas = ld_height.h_clust;//100.0;

   //Find the true height and return
   float h_report = 0.0;
   if (h_meas < C_h)
   {
   // float h_true = C_h - (d_tag/B_len)*h_meas;
   float h_true = C_h - ((d_tag+B_len)/d_tag)*(C_h - h_meas);
   h_report = h_true;
   if (h_true < 0.0)
   {
    h_true = 0.0; //this is a check, because initially the height is forced to be zero, which doesn't align with this fix, its after pouring begins that this takes affect
   }
   ld_height.h_clust = h_true;
   // std::cout << "entered adjustement" << std::endl;
   }
   // std::cout << "\n\ndebugging liquid height, \nbeaker diameter" << B_len << "\nd_tag: " << d_tag << "\nC_h: " << C_h << "\nh_meas" << h_meas <<"\nh_final: "<< h_report << "\n h_clust" << ld_height.h_clust << "\n Container area: " << recieving_container_area << std::endl;
   // double ration = apriltag_size/apriltag_length_pix;
   // std::cout << "pix to length scale m/pix" << ration << std::endl;


}


