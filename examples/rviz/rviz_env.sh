#!/bin/bash

# At minimum needs msg files. 
export ROS_PACKAGE_PATH=/tmp

#Local IP of this machine.
export ROS_IP=`ipconfig getifaddr en0`

export ROS_MASTER_URI=http://10.0.0.113:11311
