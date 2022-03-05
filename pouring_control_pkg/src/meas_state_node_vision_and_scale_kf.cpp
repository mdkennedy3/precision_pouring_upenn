/*
 * Written by <Monroe Kennedy III>
 */

#include <pouring_control_pkg/basic_ros_include.h>
#include <pouring_msgs/Hfilt.h>
#include <pouring_msgs/MeasMsg.h>
#include <pouring_msgs/LDHeight.h>
#include <pouring_msgs/ScaleMsg.h>
#include <std_srvs/Trigger.h>

class MeasStateNode
{
  public:
    MeasStateNode(ros::NodeHandle &nh);
    void MainLoop();
    void ScaleCallback(const pouring_msgs::ScaleMsg &scale_msg);
    void VisionCallback(const pouring_msgs::LDHeight &vision_msg);
    bool ResetScale(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    //For KF
    void ProcessUpdate();
    void MeasurementUpdate(const bool &use_vision, const bool &use_scale, const double &vision_volume, const double &scale_volume);

    ros::Subscriber scale_sub;
    ros::Subscriber vision_sub;
    ros::Publisher volume_pub;
  private:
    ros::NodeHandle pnh_;
    double volume_state_est;
    double volume_cov_est; //volume covariance
    double vision_meas_noise;
    double scale_meas_noise;
    double process_noise;

    pouring_msgs::ScaleMsg scale_volume_msg;
    pouring_msgs::LDHeight vision_volume_msg;
    double receiving_beaker_area;
    double lowpass_filter_alpha;

    double min_message_time; //Max time difference for a message to be considered viable
    //Message recieved bools
    bool vision_msg_received, scale_msg_received;
    double vision_volume, scale_volume;
    int seq_counter; //for pub msg
    //for mh_dot
    bool obtained_prev_volume;
    double prev_volume;
    ros::Time prev_time;
    //To reset scale digitally (taring)
    ros::ServiceServer  reset_scale_serv;
    double scale_tare_value;
};

MeasStateNode::MeasStateNode(ros::NodeHandle &nh) : pnh_(nh)
{
  volume_pub = pnh_.advertise<pouring_msgs::MeasMsg>("mh_mh_dot",1);
  scale_sub = pnh_.subscribe("arduino_scale",1,&MeasStateNode::ScaleCallback,this);
  vision_sub = pnh_.subscribe("LDHeight",1,&MeasStateNode::VisionCallback,this);
  //init volume state
  volume_state_est = 0.0;
  volume_cov_est = 10.0;
  //meas noise
  vision_meas_noise = 15.0; //ml per meas  was 0.75  15 was okay, but at times 20ml seemed more accurate
  scale_meas_noise = 1.5; //ml
  //process_noise = 0.2; //ml (per the hz freq)
  process_noise = 0.02; //ml (per the hz freq)
  //Get params
  pnh_.param<double>("mh_low_pass_filter_constant", lowpass_filter_alpha, 0.8);
  // pnh_.param<double>("recieving_container_area", receiving_beaker_area, 41.66);
  pnh_.param<double>("recieving_container_area", receiving_beaker_area, 0.005296); //r=4.106cm, C=25.8cm
  pnh_.param<double>("min_message_time", min_message_time, 0.2); //seconds
  //bool vars for msgs received
  vision_msg_received = false;
  scale_msg_received = false;
  //volumes (initialized here in case scale/vision are never called)
  vision_volume = 0.0;
  scale_volume = 0.0;
  seq_counter = 1;
  //For mh_dot
  obtained_prev_volume = false;
  prev_volume = 0.0;
  prev_time = ros::Time::now();
  //To reset scale digitally (taring)
  reset_scale_serv = pnh_.advertiseService("tare_scale_serv", &MeasStateNode::ResetScale, this);
  scale_tare_value = 0.0;
}



void MeasStateNode::MainLoop()
{
  //1. get current time
  ros::Time curr_time = ros::Time::now();
  //2. get duration from messages
  ros::Duration vision_time_diff = curr_time - vision_volume_msg.header.stamp;
  ros::Duration scale_time_diff = curr_time - scale_volume_msg.header.stamp;
  double vision_time_diff_sec = vision_time_diff.toSec();
  double scale_time_diff_sec = scale_time_diff.toSec();

  //3. Create the volumes (ml)
  if (vision_msg_received == true)
  {
    vision_volume = (receiving_beaker_area*vision_volume_msg.h_clust)*1e6;  //convert from m^3 to  ml
    /*double local_vision_volume = (receiving_beaker_area*vision_volume_msg.h_clust)*1e6;  //convert from m^3 to  ml
    if(std::abs(local_vision_volume - vision_volume)>=1.3)
    {
     vision_volume = local_vision_volume;
    }*/
  }
  if (scale_msg_received == true)
  {
    scale_volume = scale_volume_msg.mass - scale_tare_value;
    /*double scale_volume_local = scale_volume_msg.mass - scale_tare_value;
    if(std::abs(scale_volume - scale_volume_local)>=scale_meas_noise)
    {
      scale_volume = scale_volume_local; 
    }*/
  }

  //4. Do the KF process update step
  ProcessUpdate();

  //5. Using validity of time and flag stating if the message has been used already, pass these the the KF to estimate the volume in the Measurement update Step
  bool use_vision, use_scale;
  if( (vision_time_diff_sec <= min_message_time ) && (scale_time_diff_sec <= min_message_time) && (vision_msg_received == true) && (scale_msg_received == true) ){
    //case everything is available
    use_vision = true;
    use_scale =  true;
    MeasurementUpdate(use_vision, use_scale, vision_volume, scale_volume);
  }else if( (vision_time_diff_sec <= min_message_time ) && (vision_msg_received == true) ){
    //case only vision is available
    use_vision = true;
    use_scale = false;
    MeasurementUpdate(use_vision, use_scale, vision_volume, scale_volume);
  }else if( (scale_time_diff_sec <= min_message_time) && (scale_msg_received == true) ){
    //case only scale is available
    use_vision = false;
    use_scale = true;
    MeasurementUpdate(use_vision, use_scale, vision_volume, scale_volume);

  }else{
    //ROS_WARN("no volume measurements");
  }
  scale_msg_received = false;
  vision_msg_received = false;
  //6. Publish the current volume measurement estimate (ml)
  pouring_msgs::MeasMsg pour_msg;
  pour_msg.header.seq = seq_counter;  seq_counter += 1;
  pour_msg.header.stamp = curr_time;
  pour_msg.mh = volume_state_est;
  //obtained mh_dot
  if (obtained_prev_volume == true)
  {
    //find the dt
    double dt = curr_time.toSec() - prev_time.toSec();
    //find the dv
    double dv = volume_state_est - prev_volume;
    double dv_dt = dv/dt;
    pour_msg.mh_dot = dv_dt; //add to message
  }else{
    //this is the first instance
    obtained_prev_volume = true;
    prev_volume = volume_state_est;
    prev_time = curr_time;
  }
  //Publish:
  volume_pub.publish(pour_msg);
}


void MeasStateNode::ScaleCallback(const pouring_msgs::ScaleMsg &scale_msg)
{
  //scale callback
  scale_volume_msg = scale_msg;
  scale_msg_received = true;
}

void MeasStateNode::VisionCallback(const pouring_msgs::LDHeight &vision_msg)
{
  //vision callback
  vision_volume_msg = vision_msg;
  vision_msg_received = true;
}


void MeasStateNode::ProcessUpdate()
{
  /*
   *This updates the state and covariance
   *x = Ax + Bu
   *S = ASA' + R
   */
  //for estimated volume, A = I, u=0, and R is specified
  //state remains constant:
  // volume_state_est = volume_state_est;
  //covariance is updated
  volume_cov_est = volume_cov_est + process_noise;
}


void MeasStateNode::MeasurementUpdate(const bool &use_vision, const bool &use_scale, const double &vision_volume, const double &scale_volume)
{
  /*
   *This update the kalman gain, for n dim state,
   *[z_vis; z_scale]_{2nx1} = C_{2nxn} x_{nx1} + [qv_n; qs_n]; Hence C = [1;1]  when both are present and c=[1] when only one is present
   *K = S C' (C S C' + Q)^{-1}
   */

    Eigen::MatrixXd Kalman_gain, kalman_arg, covariance_eig(1,1);
    Eigen::VectorXd state_eig(1), C, state_meas;
    //Populate
    covariance_eig << volume_cov_est;
    state_eig << volume_state_est;
    if((use_vision==true) && (use_scale==true)){
      C = Eigen::VectorXd(2);
      state_meas = Eigen::VectorXd(2);

      C << 1,1;
      state_meas << vision_volume, scale_volume;
      Eigen::MatrixXd Q(2,2);
      Q(0,0) = vision_meas_noise;
      Q(1,1) = scale_meas_noise;
      // Kalman_gain
      kalman_arg = C*covariance_eig*C.transpose() + Q;
      Kalman_gain = covariance_eig*C.transpose()*kalman_arg.inverse();
    }else if(use_vision == true){
      C = Eigen::VectorXd(1);
      state_meas = Eigen::VectorXd(1);

      C << 1;
      state_meas << vision_volume;
      Eigen::MatrixXd Q(1,1);
      Q(0,0) =vision_meas_noise;
      // Kalman_gain
      kalman_arg = C*covariance_eig*C.transpose() + Q;
      Kalman_gain = covariance_eig*C.transpose()*kalman_arg.inverse();
    }else if(use_scale==true){

      C = Eigen::VectorXd(1);
      state_meas = Eigen::VectorXd(1);

      C << 1;
      state_meas << scale_volume;
      Eigen::MatrixXd Q(1,1);
      Q(0,0) =scale_meas_noise;
      // Kalman_gain
      kalman_arg = C*covariance_eig*C.transpose() + Q;
      Kalman_gain = covariance_eig*C.transpose()*kalman_arg.inverse();
    }else{
      ROS_WARN("Appropriate condition not found but ended up in meas update!");
    }
  /*
   *Measurement update
   *x = x + K (z - Cx)
   *S = (I - K C) S
   */
    state_eig = state_eig + Kalman_gain*(state_meas - C*state_eig); //state update
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(1,1);
    covariance_eig = (identity -Kalman_gain*C)*covariance_eig;
    //Convert state/cov back to valid form:
    volume_cov_est = covariance_eig(0,0);
    volume_state_est = state_eig(0);
}


bool MeasStateNode::ResetScale(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  scale_tare_value = scale_volume_msg.mass; //simply place current value for difference
  resp.success = true;
  resp.message = "Scale is tared";
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc,argv,"meas_state_node");
  ros::NodeHandle nh("~"); //nh; no tilde for msgs
  MeasStateNode cls_obj(nh);
  ros::Rate loop_rate(30);
  while(ros::ok()){
    ros::spinOnce();
    cls_obj.MainLoop();
    loop_rate.sleep();
  }


  return 0;
}

