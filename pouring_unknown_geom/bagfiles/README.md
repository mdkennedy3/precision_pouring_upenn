#This is the bagfile directory#

- Topics in these bagfiles include:
     - **/Pour\_Model\_opt**: type iiwa_pouring/PourModelPred 
          - header 
          - coef_opt (float64 list)
          - domain (float64 list)
          - residual_avg (float64)
          - residual_std (float64)
          - type (str)
     - **/pouring\_msg**: type iiwa_pouring/PouringMsg 
          - header, (Header)
          - volume, (float64)
          - dvolume, (float64)
          - angle (each float64) 
     - **/des\_height**: type (float64)
     - **/robot/joint\_states**:  type: sensor_msgs/JointState
     - **/mh\_mh\_dot**:   type: pouring_control_pkg/MeasMsg
          - header, Header
          - mh(float64), 
          - mh_dot(float64)
     - **/scale\_node/scale**: type usb_scale/Scale (contains: 
          - header, Header
          - weight(float64) 
          - units(str)
          - message(str)
     - **/robot/limb/right/joint\_command**: type: intera_core_msgs/JointCommand
     - **/h\_filtered**:  type: pouring_control_pkg/Hfilt (contains 
          - header, Header
          - hfilt(float64)
     - **/camera/apriltags/**: 
     - **/LDHeight**: type: liquid_level_detection/LDHeight (contains 
          - header, Header
          - h(float64) 
          - h_top_array((float64) array)
          - h_clust(float64) 
          - adjusted_image(sensor_msgs/Image))
     - **/LDOutput**: 
     - **/camera/image\_detection**: 
     - **/LDOutput\_fg**: 
     - **/raw\_h\_ml**: type (float64)
     - **/raw\_h\_clust\_ml**: type (float64)
     - **/model_volume_output**: type iiwa_pouring/PouringMsg
          - header, (Header)
          - volume, (float64)
          - dvolume, (float64)
          - angle (each float64)




