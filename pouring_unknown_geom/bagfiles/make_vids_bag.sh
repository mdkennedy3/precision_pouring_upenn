#!/bin/bash

for FILE in $@
do
 if [ -f "$FILE" ]
 then
   roslaunch iiwa_pouring make_video_bags.launch bagname:=$FILE save_data:=true data_name:=$FILE data_trial:=fullbag 
 else
   echo "no file provided"
 fi
done

