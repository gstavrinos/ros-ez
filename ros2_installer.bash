#!/bin/bash
sudo pip install rocker==0.2.10
sudo docker volume create ros2ez-volume
sudo docker build --no-cache --pull -t ros2_ez -f includes/ros2_Dockerfile .
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )'/ros2-ez'
string_for_bashrc='export PATH="$PATH:'$SCRIPT_DIR'"'
if ! $( grep -Fx "$string_for_bashrc" ~/.bashrc )
then
    echo $string_for_bashrc >> ~/.bashrc
fi
