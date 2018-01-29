#!/bin/sh
# Jacqueline Kory Westlund
# January 2018

set -e

cd "$HOME/projects/ros_catkin_ws/src/rr_audio_entrainer/"
. env/bin/activate
. ./libpath_for_praat.sh
cd src/
python entrain_speech_node.py -r -i $ROS_IP -d "$HOME/rr1_audio/"
