/*
 *<Monroe Kennedy III>
 *This script works to optimize coefficients and provide the model for the rest of the pipeline
 */

#include <pouring_unknown_geom/basic_ros_include.h>
#include <pouring_unknown_geom/model_selection_node_header.h>


class ModelSelectionClass
{
public:
  ModelSelectionClass(ros::NodeHandle &nh);
  ros::Publisher model_pub;
  ros::ServiceServer reset_srv;
  ros::Subscriber pouring_msg_sub;
  ros::Subscriber gp_selected_models_sub;
  ros::ServiceClient py_opt_sc;
  //pouring data record
  PouringData pouring_data;
  //params
  std::string volume_solution_mode; //taken from param server: volume_solution_mode
  int param_coef_order; //get from param: volume_profile_poly_degree
  //vars
  std::vector<double> curr_coef; //current list of coefficients
  std::vector<double> coef_domain;
  std::string coef_type;
  double initial_angle; //initial angle of onset of pouring (used to shift volume profiles for training data)
  pouring_msgs::SelectedGPModels gp_selected_models;


  void GPSelectedModelsCallback(const pouring_msgs::SelectedGPModels &data);
  void PouringMsgCallback(const pouring_msgs::PouringMsg &data);
  bool ResetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  void ResetFunction();
  void MainLoop(); //runs main loop at specified hz in main func
private:
  ros::NodeHandle pnh_;
  //Flags:
  bool collected_gp_selected_models;
  bool init_angle_set_bool;
  bool volume_data_shifted_bool;
  int pour_model_msg_index_counter;

};

ModelSelectionClass::ModelSelectionClass(ros::NodeHandle &nh) : pnh_(nh)
{
  reset_srv = pnh_.advertiseService("reset_model_selection_node",&ModelSelectionClass::ResetCallback,this); //reset trigger
  pouring_msg_sub = pnh_.subscribe("pouring_msg",1, &ModelSelectionClass::PouringMsgCallback,this); //pouring msg for volume/angle data
  gp_selected_models_sub = pnh_.subscribe("gp_selected_models",1,&ModelSelectionClass::GPSelectedModelsCallback,this); // gp selected models (is simply passed through)
  model_pub = pnh_.advertise<pouring_msgs::PourModelPred>("Pour_Model_opt",1, true); //publish the model
  //Setup the optimization service client
  ros::service::waitForService("py_profile_opt_srv");
  py_opt_sc = pnh_.serviceClient<pouring_msgs::PyOptimizationServ>("py_profile_opt_srv");
  //Call the setup

  collected_gp_selected_models = false; //only needs to be set once
  coef_domain.push_back(0.0);
  coef_domain.push_back(2.4);
 
  pour_model_msg_index_counter = 0;
  ModelSelectionClass::ResetFunction();
}


void ModelSelectionClass::ResetFunction()
{
  //Reset key variables
  //Call params
  pnh_.param<std::string>("volume_solution_mode",volume_solution_mode,"param");
  pnh_.param<int>("volume_profile_poly_degree",param_coef_order,9);
  //populate the curr coef
  curr_coef = std::vector<double>();
  for(int idx=0; idx<param_coef_order; ++idx)
  {
    curr_coef.push_back(0.0);
  }
  //set default domain
  pnh_.param<std::string>("volume_profile_polynomial_type",coef_type, "power");
  //Reset variables
  pouring_data =  PouringData();
  init_angle_set_bool = false;
  //collected_gp_selected_models = false;  //Unneccessary, as if a new container is pub then this will update
  volume_data_shifted_bool = false; //shifts vol data wrt start angle
  // gp_selected_models = pouring_msgs::SelectedGPModels(); //Don't reset this in case latch topic doesn't echo again, then you have something
}

void ModelSelectionClass::GPSelectedModelsCallback(const pouring_msgs::SelectedGPModels &data)
{
  //Populate gp selected models variable with latest
  if(collected_gp_selected_models==false){
    gp_selected_models = data;
    collected_gp_selected_models = true;
  }
}


void ModelSelectionClass::PouringMsgCallback(const pouring_msgs::PouringMsg &data)
{
  //Populate the variable pouring_data
  pouring_data.v.push_back(data.volume); 
  pouring_data.dv.push_back(data.dvolume); 
  pouring_data.ang.push_back(data.angle);
  pouring_data.stamp.push_back(data.header.stamp);
  //Store initial angle
  if(init_angle_set_bool == false){
    //Store and shift the angle of the models once obtained
    initial_angle = data.angle;
    init_angle_set_bool = true;
  }
  if(init_angle_set_bool==true && volume_data_shifted_bool==false && collected_gp_selected_models==true)
  {
    //shift the volume data
    ShiftVolumeDataFunct(gp_selected_models, initial_angle);//impl in header
    volume_data_shifted_bool = true;
  }
}


bool ModelSelectionClass::ResetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  //Reset necessary params here
  ModelSelectionClass::ResetFunction();
  resp.success = true;
  resp.message = "model_selection_node reset successfully";
  return true;
}


void ModelSelectionClass::MainLoop()
{
  //1. Get PourMsg (only proceed if more than 2 readings are present) [V,th] list
  //ROS_INFO("Inside top main loop");
  //std::cout << " collected gp models" << collected_gp_selected_models << "\n pouring data size "<< pouring_data.v.size() << std::endl;
  if(collected_gp_selected_models==true && pouring_data.v.size() > 1){
    //gp models have been collected and more than 2 volume data points have been stored [and therefore gp selected models have also been shifted]
    //ROS_INFO("conditions met for entering if statement of main loop");
    //2. Get the latest values and coef
    pouring_data;
    gp_selected_models;
    curr_coef;

    //3. Construct the optimization message
    pouring_msgs::PyOptimizationServ py_opt_req;
    for(int idx=0; idx<pouring_data.v.size(); ++idx)
    {
      py_opt_req.request.vol_data.push_back(pouring_data.v[idx]);
      py_opt_req.request.ang_data.push_back(pouring_data.ang[idx]);
    }
    py_opt_req.request.coef_init = curr_coef;

    if (py_opt_sc.call(py_opt_req))
    {
      //ROS_INFO("py_opt_sc callback successful");
      //Now send the update to the publisher and publish
      pouring_msgs::PourModelPred pour_model_msg;
      pour_model_msg.header.seq = pour_model_msg_index_counter; pour_model_msg_index_counter += 1;
      pour_model_msg.header.stamp = ros::Time::now();
      //Coef params
      pour_model_msg.coef_opt = py_opt_req.response.coef_opt;
      pour_model_msg.domain = coef_domain;
      pour_model_msg.residual_avg = py_opt_req.response.resid_avg;
      pour_model_msg.residual_std = py_opt_req.response.resid_std;
      pour_model_msg.type= coef_type;
      //Add the GP models
      pour_model_msg.gp_volume_models = gp_selected_models.gp_volume_models;
      pour_model_msg.volume_solution_mode = volume_solution_mode;
      //Publish
      model_pub.publish(pour_model_msg);
    }
    else
    {
      ROS_WARN("py_opt_sc callback NOT successful");
    }
  }//end olf if statement for collected models and pouring size check
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_selection_node");
  ros::NodeHandle nh("~");
  ModelSelectionClass cls_obj(nh);
  ros::Rate loop_rate(100); //HZ
  while(ros::ok()){
    ros::spinOnce();
    cls_obj.MainLoop();
    loop_rate.sleep();
  }

  return 0;
}
