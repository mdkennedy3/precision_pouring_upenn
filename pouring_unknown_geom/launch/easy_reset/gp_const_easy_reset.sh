#!/bin/bash

rosservice call /pouring_control/level_detector_node_nn/reset
rosservice call /liquid_level_detection_nn/docker_apriltag_reset
rosservice call /pouring_control/reset_pour_vol_ang 
rosservice call /pouring_control/reset_main_pouring_node 
rosservice call /pouring_control/reset_model_selection_node  
rosservice call /pouring_control/tare_scale_serv
