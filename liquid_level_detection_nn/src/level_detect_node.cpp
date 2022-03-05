/*
* Copyright (c) <2017>, <Dinesh Thakur>
* All rights reserved.
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. Neither the name of the University of Pennsylvania nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <std_msgs/Float64.h>
#include <liquid_level_detection/LDHeight.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Trigger.h>

#include <boost/thread/lock_guard.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>

#include <eigen3/Eigen/Dense>
#include <liquid_level_detection/LiquidHeightFluidTopAdjust.h>

class LevelDetect
{
public:
  LevelDetect();
  ~LevelDetect();
  void publishLevel();
private:
  void displayImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  bool resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response  &res);

  void resetApriltag();

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Subscriber image_sub_;
  ros::Subscriber display_image_sub_;
  ros::Publisher height_pub_;
  image_transport::Publisher image_pub_, fg_image_pub_;

  double desired_height_cm_;
  int apriltag_id_;
  double apriltag_size_;
  double apriltag_cut_area_;
  double cylinder_height_;

  int apriltag_length_pix_;
  int top_cutoff_pix_;
  int cylinder_bottom_pix_;
  int x_offset_pix_;
  int x_center_pix_;

  int y_offset_pix_;

  bool debug_;

  int y_current_height_pix_;
  int y_cluster_height_pix_;


  sensor_msgs::ImageConstPtr display_image_msg_;
  bool has_display_image_;

  apriltag_msgs::ApriltagArrayStampedConstPtr april_tag_;


  ros::ServiceServer reset_service_;

  double avg;

  //For TF
  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  tf2_ros::TransformBroadcaster tfbroadcast;
  std::string camera_frame_name_;
  geometry_msgs::TransformStamped tf_stamped_tag_to_cam;
  //For adjusting fluid height
  double recieving_container_area_;


};

LevelDetect::LevelDetect() : it_(nh_)
{
  nh_ = ros::NodeHandle("~");

  nh_.param("desired_height_cm", desired_height_cm_, 10.0);
  nh_.param("apriltag_id", apriltag_id_, 4);
  nh_.param("apriltag_size", apriltag_size_, 0.017);
  nh_.param("apriltag_cut_area", apriltag_cut_area_, 0.025);
  nh_.param("cylinder_height", cylinder_height_, 0.0996);
  nh_.param("x_offset_pix", x_offset_pix_, 50);
  nh_.param("y_offset_pix", y_offset_pix_, 50);
  nh_.param("debug", debug_, true);

  nh_.param<std::string>("camera_frame_name", camera_frame_name_, "pg_13344889");
  nh_.param<double>("recieving_container_area", recieving_container_area_, 0.005296); //units in meters sqrd



  //For TF:
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));


  height_pub_ = nh_.advertise<liquid_level_detection::LDHeight>("detected_height", 1);
  has_display_image_ = false;
  image_sub_ = nh_.subscribe<sensor_msgs::Image>("image", 10, boost::bind(&LevelDetect::imageCallback, this, _1));
  display_image_sub_ = nh_.subscribe<sensor_msgs::Image>("display_image", 10, boost::bind(&LevelDetect::displayImageCallback, this, _1));
  image_pub_ = it_.advertise("LDOutput", 1);
  fg_image_pub_ = it_.advertise("LDOutput_fg", 1);

  resetApriltag();

  reset_service_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("reset", boost::bind(&LevelDetect::resetCallback, this, _1, _2));

  if(debug_)
  {
    cv::namedWindow("Input image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Foreground", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("GradX", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("GradY", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Grad", CV_WINDOW_AUTOSIZE);
  }
}

LevelDetect::~LevelDetect()
{

  if(debug_)
  {
    cv::destroyWindow("Input image");
    cv::destroyWindow("Foreground");
    cv::destroyWindow("GradX");
    cv::destroyWindow("GradY");
    cv::destroyWindow("Grad");
  }
}

bool LevelDetect::resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response  &res)
{
  resetApriltag();
  res.success = true;

  return true;
}

void LevelDetect::resetApriltag()
{
  y_current_height_pix_ = 0;
  y_cluster_height_pix_ = 0;

  bool valid_tag = false;

  ros::Rate loop_rate(1);

  while (ros::ok() && (!valid_tag))
  {
    ROS_WARN("waiting for valid apriltag id %d", apriltag_id_);

    has_display_image_ = false;
    april_tag_ = ros::topic::waitForMessage<apriltag_msgs::ApriltagArrayStamped>("apriltags", ros::Duration(0.5));

    if(april_tag_)
    {
      for(int i=0; i<april_tag_->apriltags.size(); i++)
      {
        apriltag_msgs::Apriltag tag = april_tag_->apriltags[i];
        if(tag.id == apriltag_id_)
        {
          ROS_INFO("Got valid apriltag %d", apriltag_id_);

          bool tf_success = true; //To determine if tf's were obtained to continue with calculations
          try{
              tf_stamped_tag_to_cam = tfBuffer.lookupTransform(camera_frame_name_,"cylinder_Tag", ros::Time(0));
             }
          catch (tf2::TransformException &ex) {
              ROS_WARN("%s",ex.what());
              ROS_WARN("TF between camera and tag not found");
              ros::Duration(1.0).sleep();
              tf_success = false;
          }
          if(tf_success)
          {
            valid_tag = true;
          }

          geometry_msgs::Point bottom_right = tag.corners[1];
          geometry_msgs::Point top_right = tag.corners[2];

          apriltag_length_pix_ = std::sqrt(std::pow((bottom_right.x - top_right.x),2) + std::pow((bottom_right.y - top_right.y),2));

          top_cutoff_pix_ = int(tag.center.y +((apriltag_cut_area_* apriltag_length_pix_)/apriltag_size_));

          int pix_difference = int((apriltag_length_pix_*cylinder_height_)/apriltag_size_);

          x_center_pix_ = int(tag.center.x);

          cylinder_bottom_pix_ = int(tag.center.y) + pix_difference;

          x_offset_pix_ = apriltag_length_pix_;

          y_current_height_pix_ = cylinder_bottom_pix_;
          y_cluster_height_pix_ = cylinder_bottom_pix_;
          ROS_INFO("Apriltag size %g pix - top_cutoff %d cy_bottom %d x_offset_pix_ %d", apriltag_size_, top_cutoff_pix_, cylinder_bottom_pix_, x_offset_pix_);
          break;
    	 }
      }
    }
    loop_rate.sleep();
  }

}

void LevelDetect::displayImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  display_image_msg_ = msg;
  has_display_image_ = true;
}

void LevelDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!has_display_image_) {
    ROS_WARN("Waiting for display_image");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr_orig, cv_ptr_disp;
  try
  {
    cv_ptr_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    cv_ptr_disp = cv_bridge::toCvCopy(display_image_msg_, sensor_msgs::image_encodings::BGR8);
    // cv::cvtColor(cv_ptr_disp->image, cv_ptr_disp->image, CV_GRAY2RGB);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat fgmask1;



  int ch = cv_ptr_orig->image.channels();
  // ROS_INFO("nchn %d", ch);

  cv::Mat fgmask;
  fgmask = cv_ptr_orig->image.clone();

  int scale = 10;
  int delta = 0;
  int ddepth = CV_16S;
  cv::Mat src_gray, grad_x, abs_grad_x, grad_y, abs_grad_y, grad;

  int threshold_type = 0; // 0 - binary, 1 - binary inverted, there are more options
  // cv::threshold(fgmask,fgmask,254,255,threshold_type);

  //GaussianBlur( fgmask, fgmask, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  //Gradient in X
  cv::Sobel(fgmask, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_x, abs_grad_x );

  //Gradient in Y
  cv::Sobel(fgmask, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

  convertScaleAbs( grad_y, abs_grad_y );
  // cv::threshold(abs_grad_y, abs_grad_y,150,255,0);
  // fgmask = abs_grad_y;

  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
  // ROS_INFO("Displaying images");
  if(debug_){
    cv::imshow("Foreground", fgmask);
    cv::imshow("GradX", abs_grad_x);
    cv::imshow("GradY", abs_grad_y);
    cv::imshow("Grad", grad);
    cv::waitKey(3);
  }

  //Center line, left line, right line
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_, cylinder_bottom_pix_), cv::Point(x_center_pix_, top_cutoff_pix_),CV_RGB(0,0,255),2);
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_ - x_offset_pix_, cylinder_bottom_pix_), cv::Point(x_center_pix_ - x_offset_pix_, top_cutoff_pix_),CV_RGB(0,0,255),2);
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_ + x_offset_pix_, cylinder_bottom_pix_), cv::Point(x_center_pix_ + x_offset_pix_, top_cutoff_pix_),CV_RGB(0,0,255), 2);

  //cutoff line based on current height
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_ - x_offset_pix_, y_current_height_pix_ - y_offset_pix_), cv::Point(x_center_pix_ + x_offset_pix_, y_current_height_pix_ - y_offset_pix_),CV_RGB(0,255,0), 2);

  //Line at the desired fill height
   int desired_height_pix = cylinder_bottom_pix_ - int((desired_height_cm_*apriltag_length_pix_)/(apriltag_size_*100.0));
  //int desired_height_pix = cylinder_bottom_pix_ - int((desired_height_cm_*apriltag_length_pix_)/(apriltag_size_)); //units meters
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_ - x_offset_pix_, desired_height_pix), cv::Point(x_center_pix_ + x_offset_pix_, desired_height_pix),CV_RGB(255,0,0), 2);

  int channels = fgmask.channels();
  int nRows = fgmask.rows;
  int nCols = fgmask.cols * channels;

  std::vector<int> y_pix_vect;
  std::vector<cv::Point2f> y_pix_vect2;
  std::vector<int> x_pix_vect;

  int avg_y_pix = 0;

  int best_weight = 0;
  std::vector<int> weights;
  for(int y = (y_current_height_pix_ - y_offset_pix_); y < cylinder_bottom_pix_; y++)
  {
    int curr_weight = 0;

    for(int x = (x_center_pix_ - x_offset_pix_); x < (x_center_pix_ + x_offset_pix_); x++)
    {
      // curr_weight += (int)abs_grad_y.at<uchar>(y,x);
      curr_weight += (int)fgmask.at<uchar>(y,x);
    }

    if (curr_weight > best_weight) {
      best_weight = curr_weight;
    }
    weights.push_back(curr_weight);
  }


  for (int i = 0; i<weights.size(); i++)
  {
    // std::cout << weights[i] << " " << best_weight << "\n";
    if (weights[i] > 0.75 * best_weight) {
      avg = (y_current_height_pix_ - y_offset_pix_) + i + 10;
      break;
    }
  }

  if (avg < y_current_height_pix_) {
    y_current_height_pix_ = avg;
  }

  int height_pix = cylinder_bottom_pix_ - avg;

  liquid_level_detection::LDHeight ld_height;
  // ld_height.h = ((height_pix*apriltag_size_)/apriltag_length_pix_)*100.0;
  ld_height.h = ((height_pix*apriltag_size_)/apriltag_length_pix_); //units are meters
  ld_height.h_clust = ld_height.h;

  //temp hack
  if(y_current_height_pix_ > cylinder_bottom_pix_)
    y_current_height_pix_ = cylinder_bottom_pix_;

  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, avg_y_pix), 5, CV_RGB(0,255,255));
  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, y_current_height_pix_), 10, CV_RGB(255,0,255));

  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, avg), 10, CV_RGB(255,255,255));

  //Account for seeing the top of the fluid before publishing:
  int img_pix_height = msg->height;
  TrueLiquidHeightCalc(ld_height,
                       img_pix_height,
                       tf_stamped_tag_to_cam,
                       recieving_container_area_,
                       cylinder_bottom_pix_,
                       apriltag_size_,
                       apriltag_length_pix_);

  ld_height.header = cv_ptr_disp->header;
  height_pub_.publish(ld_height);

  cv_ptr_disp->toImageMsg(ld_height.adjusted_image);
  // sensor_msgs::ImagePtr output_msg = cv_ptr_disp->toImageMsg();
  // output_msg->encoding = sensor_msgs::image_encodings::RGB8;
  image_pub_.publish(cv_ptr_disp->toImageMsg());

  cv_bridge::CvImage out_msg;
  out_msg.header   = cv_ptr_disp->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_msg.image    = fgmask; // Your cv::Mat
  fg_image_pub_.publish(out_msg.toImageMsg());

  if(debug_)
  {
    // Update GUI Window
    cv::imshow("Input Image", cv_ptr_disp->image);
    cv::waitKey(3);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trigger_server");

  LevelDetect lv_det;
  ros::spin();
}
