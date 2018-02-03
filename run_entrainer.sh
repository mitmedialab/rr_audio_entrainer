#!/bin/sh
# Jacqueline Kory Westlund
# January 2018

set -e

cd "$(rospack find rr_audio_entrainer)"
. env/bin/activate
. ./libpath_for_praat.sh
cd src/
python entrain_speech_node.py -r -i $ROS_IP -d "$HOME/rr1_audio/"
