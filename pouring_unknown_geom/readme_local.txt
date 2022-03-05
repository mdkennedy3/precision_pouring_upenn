On sawyer comp:
roslaunch pouring_unknown_geom camera_sensors_pico_elp.launch 

On GPU nuc:
1. Scan container first: 
roslaunch pouring_unknown_geom pouring_complete_just_scan_no_pour.launch 
2. Run the detector:
roslaunch pouring_unknown_geom pouring_complete_just_pour_no_scan.launch 
3a. to pour motions: 
rosrun pouring_unknown_geom run_simple_pour.py 
3b. to rotate back: 
rosrun pouring_unknown_geom return_to_start_pour.py 
4. reset the pouring scene: 
(in launch/easy_reset/ ./easy_reset.sh)


NN detector debug (issues):
A. make sure invidea is enabled (and logout login)
B. if launch doesn't work: i) run detector.py in its place first ii) then replace back with detector_no_docket.py



