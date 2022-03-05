#!/bin/bash

#FILE=$1
for FILE in $@
do
 if [ -f "$FILE" ]
 then
  scp $FILE manip_control@192.168.0.103:~/Desktop/
 else
  echo "no file provided"
 fi
done
