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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>


#include <liquid_level_detection/GRANSAC.hpp>
#include <liquid_level_detection/LineModel.hpp>
#include <liquid_level_detection/LiquidHeightFluidTopAdjust.h>



class LevelDetect
{
public:
  LevelDetect();
  ~LevelDetect();
  void publishLevel();
private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  bool resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response  &res);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  ros::Subscriber image_sub_;
  ros::Publisher height_pub_;
  image_transport::Publisher image_pub_, fg_image_pub_;

  double desired_height_cm_;
  double apriltag_size_;
  double apriltag_cut_area_;
  double cylinder_height_;

  int apriltag_id_;
  int apriltag_length_pix_;
  int top_cutoff_pix_;
  int cylinder_bottom_pix_;
  int x_offset_pix_;
  int x_center_pix_;
  int y_center_pix_;
  //int z_center_pix_;

  int y_offset_pix_;

  int count_;
  bool debug_;

  int y_current_height_pix_;
  int y_cluster_height_pix_;

  apriltag_msgs::ApriltagArrayStampedConstPtr april_tag_;
  cv::Ptr<cv::BackgroundSubtractorMOG2> bgm_;
  cv::Mat background_im_;

  ros::ServiceServer reset_service_;
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
  pnh_ = ros::NodeHandle("~");

  pnh_.param("desired_height_cm", desired_height_cm_, 10.0);
  pnh_.param("apriltag_id", apriltag_id_, 4);
  pnh_.param("apriltag_size", apriltag_size_, 0.017);
  pnh_.param("apriltag_cut_area", apriltag_cut_area_, 0.025);
  pnh_.param("cylinder_height", cylinder_height_, 0.0996);
  pnh_.param("x_offset_pix", x_offset_pix_, 50);
  pnh_.param("y_offset_pix", y_offset_pix_, 50);
  pnh_.param("debug", debug_, true);
  pnh_.param<std::string>("camera_frame_name", camera_frame_name_, "pg_13344889");
  pnh_.param<double>("recieving_container_area", recieving_container_area_, 41.66);

  height_pub_ = pnh_.advertise<liquid_level_detection::LDHeight>("detected_height", 1);
  image_sub_ = pnh_.subscribe<sensor_msgs::Image>("image", 10, boost::bind(&LevelDetect::imageCallback, this, _1));
  image_pub_ = it_.advertise("LDOutput", 1);
  fg_image_pub_ = it_.advertise("LDOutput_fg", 1);

  bool valid_tag = false;

  ros::Rate loop_rate(1);

  //For TF: 
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));


  y_current_height_pix_ = 0;
  y_cluster_height_pix_ = 0;
  while (ros::ok() && (!valid_tag))
  {

    april_tag_ = ros::topic::waitForMessage<apriltag_msgs::ApriltagArrayStamped>("apriltags", ros::Duration(0.5));


    if(april_tag_)
    {
      for(int i=0; i<april_tag_->apriltags.size(); i++)
      {
        if(april_tag_->apriltags[i].id == apriltag_id_)
        {
          ROS_INFO("Got valid apriltag");

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

          geometry_msgs::Point bottom_right = april_tag_->apriltags[i].corners[1];
          geometry_msgs::Point top_right = april_tag_->apriltags[i].corners[2];

          apriltag_length_pix_ = std::sqrt(std::pow((bottom_right.x - top_right.x),2) + std::pow((bottom_right.y - top_right.y),2));

          top_cutoff_pix_ = int(april_tag_->apriltags[i].center.y +((apriltag_cut_area_* apriltag_length_pix_)/apriltag_size_));

          int pix_difference = int((apriltag_length_pix_*cylinder_height_)/apriltag_size_);

          x_center_pix_ = int(april_tag_->apriltags[i].center.x);
          y_center_pix_ = int(april_tag_->apriltags[i].center.y);
          //z_center_pix_ = int(april_tag_->apriltags[i].center.z);

          cylinder_bottom_pix_ = int(april_tag_->apriltags[i].center.y) + pix_difference;

          x_offset_pix_ = apriltag_length_pix_;

          y_current_height_pix_ = cylinder_bottom_pix_;
          y_cluster_height_pix_ = cylinder_bottom_pix_;
          ROS_INFO("Apriltag size %g pix - top_cutoff %d cy_bottom %d x_offset_pix_ %d", apriltag_size_, top_cutoff_pix_, cylinder_bottom_pix_, x_offset_pix_);
          break;
        }
      }
    }
    ROS_WARN("waiting for valid apriltag id %d", apriltag_id_);
    loop_rate.sleep();
  }

  count_ = 0;
  bgm_ = cv::createBackgroundSubtractorMOG2(100, 16, false);

  reset_service_ = pnh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("reset", boost::bind(&LevelDetect::resetCallback, this, _1, _2));

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

  count_ = 0;

  y_current_height_pix_ = 0;
  y_cluster_height_pix_ = 0;

  bool valid_tag = false;

  ros::Rate loop_rate(1);

  while (ros::ok() && (!valid_tag))
  {

    april_tag_ = ros::topic::waitForMessage<apriltag_msgs::ApriltagArrayStamped>("apriltags", ros::Duration(0.5));

    if(april_tag_)
    {
      for(int i=0; i<april_tag_->apriltags.size(); i++)
      {
        if(april_tag_->apriltags[i].id == apriltag_id_)
        {
          ROS_INFO("Got valid apriltag");
          valid_tag = true;

          geometry_msgs::Point bottom_right = april_tag_->apriltags[i].corners[1];
          geometry_msgs::Point top_right = april_tag_->apriltags[i].corners[2];

          apriltag_length_pix_ = std::sqrt(std::pow((bottom_right.x - top_right.x),2) + std::pow((bottom_right.y - top_right.y),2));

          top_cutoff_pix_ = int(april_tag_->apriltags[i].center.y +((apriltag_cut_area_* apriltag_length_pix_)/apriltag_size_));

          int pix_difference = int((apriltag_length_pix_*cylinder_height_)/apriltag_size_);

          x_center_pix_ = int(april_tag_->apriltags[i].center.x);

          cylinder_bottom_pix_ = int(april_tag_->apriltags[i].center.y) + pix_difference;

          x_offset_pix_ = apriltag_length_pix_;

          y_current_height_pix_ = cylinder_bottom_pix_;
          y_cluster_height_pix_ = cylinder_bottom_pix_;
          ROS_INFO("Apriltag size %g pix - top_cutoff %d cy_bottom %d x_offset_pix_ %d", apriltag_size_, top_cutoff_pix_, cylinder_bottom_pix_, x_offset_pix_);
          break;
        }
      }
    }
    ROS_WARN("waiting for valid apriltag id %d", apriltag_id_);
    loop_rate.sleep();
  }

  count_ = 0;
  bgm_ = cv::createBackgroundSubtractorMOG2(100, 16, false);

  res.success = true;

  return true;
}

void LevelDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr_orig, cv_ptr_disp;
  try
  {
    cv_ptr_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_disp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat fgmask1;

  if(count_ < 10)
  {
    ROS_INFO("Learning background");
    bgm_->apply(cv_ptr_orig->image, fgmask1);
    count_ += 1;
    return;
  }

  int ch = cv_ptr_orig->image.channels();
  //ROS_INFO("nchn %d", ch);

  cv::Mat fgmask;
  bgm_->apply(cv_ptr_orig->image, fgmask);

  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::Mat src_gray, grad_x, abs_grad_x, grad_y, abs_grad_y, grad;

  int threshold_type = 0; // 0 - binary, 1 - binary inverted, there are more options
  cv::threshold(fgmask,fgmask,254,255,threshold_type);

  //GaussianBlur( fgmask, fgmask, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  //Gradient in X
  cv::Sobel(fgmask, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_x, abs_grad_x );

  //Gradient in Y
  cv::Sobel(fgmask, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_y, abs_grad_y );

  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  if(debug_){
    cv::imshow("Foreground", fgmask);
    cv::imshow("GradX", abs_grad_x);
    cv::imshow("GradY", abs_grad_y);
    cv::imshow("Grad", abs_grad_y);
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
  cv::line(cv_ptr_disp->image, cv::Point(x_center_pix_ - x_offset_pix_, desired_height_pix), cv::Point(x_center_pix_ + x_offset_pix_, desired_height_pix),CV_RGB(255,0,0), 2);

  int channels = fgmask.channels();
  int nRows = fgmask.rows;
  int nCols = fgmask.cols * channels;

  std::vector<int> y_pix_vect;
  std::vector<cv::Point2f> y_pix_vect2;
  std::vector<int> x_pix_vect;

  int avg_y_pix = 0;

  for(int x = (x_center_pix_ - x_offset_pix_); x < (x_center_pix_ + x_offset_pix_); x++)
  {
    int y_top_pix;
    int y_bot_pix;
    bool first = true;
    //for(int y = top_cutoff_pix_; y< cylinder_bottom_pix_; y++)
    for(int y = (y_current_height_pix_ - y_offset_pix_); y < cylinder_bottom_pix_; y++)
    {
      //auto val = (int)fgmask.at<uchar>(y,x);
      //if(val > 254)
        //cv::circle(cv_ptr_disp->image, cv::Point(x, y), 1, CV_RGB(255,0,0));

      if (x < 0 || x > abs_grad_y.cols || y < 0 || y > abs_grad_y.rows){
        //ROS_WARN("x %d, y %d out of bounds", x,y);
        continue;
      }

      int val2 = (int)abs_grad_y.at<uchar>(y,x);

      if(val2 > 0){
        //cv::circle(cv_ptr_disp->image, cv::Point(x, y), 1, CV_RGB(0,255,0));
        y_bot_pix = y;
        if(first){
          y_top_pix = y;
          first = false;

          avg_y_pix += y_top_pix;
          y_pix_vect.push_back(y_top_pix);

          y_pix_vect2.push_back(cv::Point(x_center_pix_, y_top_pix));
          x_pix_vect.push_back(x);

          //int height_pix = cylinder_bottom_pix_ - y_top_pix;

          //cv::circle(cv_ptr_disp->image, cv::Point(x, y), 2, CV_RGB(255,255,0));
        }
      }
    }
  }

  liquid_level_detection::LDHeight ld_height;

  if(y_pix_vect.size()> x_offset_pix_) //atleast half width lines
  {
    int n_dimension =  y_pix_vect.size();
    //Eigen::Matrix<double, 1, n_dimension>  h_top_eigen;
    //Eigen::VectorXd h_top_eigen(n_dimension);

    for(int i =0; i < y_pix_vect.size(); i++)
    {
      int height_pix = cylinder_bottom_pix_ - y_pix_vect.at(i);
      float height_cm = ((height_pix*apriltag_size_)/apriltag_length_pix_)*100.0;
      ld_height.h_top_array.push_back(height_cm);

      ld_height.h += height_cm;

      //h_top_eigen(0,i) = height_cm;
    }
    ld_height.h = ld_height.h/ld_height.h_top_array.size();


    avg_y_pix = avg_y_pix/y_pix_vect.size();
    y_current_height_pix_ = avg_y_pix;

    //KMeans
    cv::Scalar colorTab[] =
    {
      cv::Scalar(0, 0, 255),
      cv::Scalar(0,255,0),
      cv::Scalar(255,100,100),
      cv::Scalar(255,0,255),
      cv::Scalar(255,255,255)
    };
    int clusterCount = 2;
    cv::Mat labels, centers;
    cv::kmeans(y_pix_vect2, clusterCount, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

    //Get the cluster center that is lower
    cv::Point center_min = centers.at<cv::Point2f>(0);
    int lower_ind = 0;
    for(int i = 1; i < clusterCount; i++ )
    {
      cv::Point cent = centers.at<cv::Point2f>(i);
      if(cent.y > center_min.y){
        center_min = cent;
        lower_ind = i;
      }
    }

    //ROS_INFO("lower ind %d", lower_ind);

    //Lower cluster of points
    std::vector<cv::Point2f> y_pix_lower;
    for(int i = 0; i < n_dimension; i++ )
    {
      int clusterIdx = labels.at<int>(i);
      cv::Point ipt = y_pix_vect2.at(i);
      ipt.x = x_pix_vect.at(i);
      //cv::circle(cv_ptr_disp->image,ipt, 2, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA);

      if(clusterIdx == lower_ind){
        y_pix_lower.push_back(ipt);
      }
    }

    //Generate a new color for lower cluster
    //ROS_INFO("lower cluster size %d total %d", y_pix_lower.size(), n_dimension);
    for(int i = 0; i < y_pix_lower.size(); i++ )
    {
      cv::Point ipt = y_pix_lower.at(i);
      //ipt.x = x_pix_vect.at(i);
      cv::circle(cv_ptr_disp->image,ipt, 2, colorTab[4], cv::FILLED, cv::LINE_AA);
    }

    cv::RotatedRect minEllipse;

    /*
    if( y_pix_lower.size() > 5 ){
      minEllipse = cv::fitEllipse(y_pix_lower);
      //cv::ellipse(cv_ptr_disp->image, minEllipse, cv::Scalar(255,0,255), 2, 8 );
    }

    */


    y_cluster_height_pix_ = int(center_min.y);
    int cluster_pix = cylinder_bottom_pix_ - y_cluster_height_pix_;
    float cluster_cm = ((cluster_pix*apriltag_size_)/apriltag_length_pix_)*100.0;
    ld_height.h_clust = cluster_cm;


    //GRANSAC

    //Setup candidate points for RANSAC
    /*
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    for(int i = 0; i < n_dimension; i++ )
    {

      std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(x_pix_vect.at(i), y_pix_vect.at(i));
      CandPoints.push_back(CandPt);

    }*/

/*
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    for(int i = 0; i < y_pix_lower.size(); i++ )
    {

      std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(y_pix_lower.at(i).x, y_pix_lower.at(i).y);
      CandPoints.push_back(CandPt);

    }

    GRANSAC::RANSAC<Line2DModel, 2> Estimator;
    Estimator.Initialize(2, 100); // Threshold, iterations

    std::vector<std::pair <double, double>> height_from_line_vec;

    for(int i = 0; i < 2; i++)
    {

      Estimator.Estimate(CandPoints);

      auto BestInliers = Estimator.GetBestInliers();
      if (BestInliers.size() > 0)
      {
        for (auto& Inlier : BestInliers)
        {
          auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
          cv::Point Pt(floor(RPt->m_Point2D[0]), floor(RPt->m_Point2D[1]));
          cv::circle(cv_ptr_disp->image, Pt, 2, cv::Scalar(150, 255, 0), -1);
        }
      }


      auto BestLine = Estimator.GetBestModel();
      if (BestLine)
      {
        auto BestLinePt1 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[0]);
        auto BestLinePt2 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[1]);
        if (BestLinePt1 && BestLinePt2)
        {
          cv::Point Pt1(BestLinePt1->m_Point2D[0], BestLinePt1->m_Point2D[1]);
          cv::Point Pt2(BestLinePt2->m_Point2D[0], BestLinePt2->m_Point2D[1]);
          GRANSAC::VPFloat slope = (GRANSAC::VPFloat)(Pt2.y - Pt1.y) / (Pt2.x - Pt1.x);

          cv::Point p(0, 0), q(cv_ptr_disp->image.cols, cv_ptr_disp->image.rows);

          p.y = -(Pt1.x - p.x) * slope + Pt1.y;
          q.y = -(Pt2.x - q.x) * slope + Pt2.y;

          cv::line(cv_ptr_disp->image, p, q, cv::Scalar(150, 0, 0), 1, 8, 0);

          double height_from_line_pix = (-(Pt1.x - x_center_pix_) * slope + Pt1.y);

          if(std::fabs(slope) < std::tan(15*M_PI/180.0) )
          {
            height_from_line_vec.push_back(std::make_pair(slope, height_from_line_pix));
            cv::line(cv_ptr_disp->image, p, q, cv::Scalar(0, 255, 255), 1, 8, 0);
          }
        }
      }

      CandPoints = Estimator.GetBestOutliers();
    }

    double max_val = 0.0;
    double best_slope = 0;

    for(int i = 0; i < height_from_line_vec.size(); i++)
    {
      auto pt = height_from_line_vec.at(i);
      if (pt.second > max_val)
      {
        max_val = pt.second;
        best_slope = pt.first;
      }
    }

    cv::Point p(0, 0), q(cv_ptr_disp->image.cols, cv_ptr_disp->image.rows);

    p.y = -(x_center_pix_ - p.x) * best_slope + max_val;
    q.y = -(x_center_pix_ - q.x) * best_slope + max_val;

    cv::line(cv_ptr_disp->image, p, q, cv::Scalar(0, 0, 255), 2, 8, 0);

*/
  }
  else
  {
    //use prev stored height
    int height_pix = cylinder_bottom_pix_ - y_current_height_pix_;
    float height_cm = ((height_pix*apriltag_size_)/apriltag_length_pix_)*100.0;
    ld_height.h = height_cm;

    int cluster_pix = cylinder_bottom_pix_ - y_cluster_height_pix_;
    float cluster_cm = ((cluster_pix*apriltag_size_)/apriltag_length_pix_)*100.0;
    ld_height.h_clust = cluster_cm;
  }

  //temp hack
  if(y_current_height_pix_ > cylinder_bottom_pix_)
    y_current_height_pix_ = cylinder_bottom_pix_;

  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, avg_y_pix), 5, CV_RGB(0,255,255));
  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, y_current_height_pix_), 10, CV_RGB(255,0,255));

  cv::circle(cv_ptr_disp->image, cv::Point(x_center_pix_, y_cluster_height_pix_), 10, CV_RGB(255,255,255));


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
