#!/bin/sh
# Jacqueline Kory Westlund
# January 2018

set -e

cd /home/jakory/projects/ros_catkin_ws/src/rr_audio_entrainer/
. env/bin/activate
. ./libpath_for_praat.sh
cd src/
python entrain_speech_node.py -r -i $ROS_IP -d "/home/jakory/rr1_audio/"
