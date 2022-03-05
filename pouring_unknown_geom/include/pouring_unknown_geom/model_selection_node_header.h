#pragma once

#include <pouring_msgs/PourModelPred.h>
#include <pouring_msgs/PouringMsg.h>
#include <pouring_msgs/SelectedGPModels.h>
#include <pouring_msgs/GPVolumeModel.h>
#include <pouring_msgs/PyOptimizationServ.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerResponse.h>

struct PouringData
{
 std::vector<double> v, dv, ang;
 std::vector<ros::Time> stamp;
};


void ShiftVolumeDataFunct(pouring_msgs::SelectedGPModels &gp_selected_models,const double &initial_angle)
{
  //This function shifts the volume data given an initial angle
  int closest_index;
  for(int jdx=0; jdx<gp_selected_models.gp_volume_models.size(); ++jdx){
    double min_diff = 1e3;//this needs to be inside so it is not persistent btw containers
    pouring_msgs::GPVolumeModel curr_model = gp_selected_models.gp_volume_models[jdx];
    //1. find the angle closest to the initial angle
    for (int idx=0; idx<curr_model.th.size(); ++idx)
    {
      double curr_angle = curr_model.th[idx];
      double diff_abs = std::abs(curr_angle-initial_angle);
      if (diff_abs < min_diff)
      {
        min_diff = diff_abs;
        closest_index = idx;
      }
    }
    //2. Now with the closest index, use the corresponding Volume at that index to shift all of the others
    double V_zero = curr_model.V[closest_index];
    for(int idx=0; idx<curr_model.V.size(); ++idx)
    {
      gp_selected_models.gp_volume_models[jdx].V[idx] = curr_model.V[idx] - V_zero;
    }
  }
}
