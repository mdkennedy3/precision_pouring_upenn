#!/bin/bash
mypath=$(pwd)
echo Working directory: $mypath
roslaunch pouring_unknown_geom post_process.launch post_process_pour_curve:=False  post_process_method_comparison:=True method_comp_bags_path:=$mypath
