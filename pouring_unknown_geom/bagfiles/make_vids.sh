#!/bin/bash

for FILE in $@
do
 if [ -f "${FILE}" ]
 then
   python ./rosbag2video/rosbag2video.py --fps 30 -o ${FILE}_video.mp4 -t /robot/head_display ${FILE}
 else
   echo "no file provided"
 fi
done


