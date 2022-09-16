#!/bin/bash
# sudo pip install rocker
# sudo docker build --no-cache --pull -t ros_ez -f includes/ros_Dockerfile .
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )'/ros-ez'
string_for_bashrc='export PATH="$PATH:'$SCRIPT_DIR'"'
if ! $( grep -Fx "$string_for_bashrc" ~/.bashrc )
then
    echo $string_for_bashrc >> ~/.bashrc
fi
