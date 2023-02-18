#!/bin/bash
original_dir=$(pwd)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
image_name="ros2_ez"
volume="ros2ez-volume"
executable_folder_name="ros2-ez"
dockerfile="ros2_Dockerfile"
if [ "$1" == "1" ]; then
    image_name="ros_ez"
    volume="rosez-volume"
    executable_folder_name="ros-ez"
    dockerfile="ros_Dockerfile"
fi
sudo pip install rocker==0.2.10
cd $SCRIPT_DIR
sudo docker volume create $volume
sudo docker build --no-cache --pull -t $image_name -f $SCRIPT_DIR/$dockerfile . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)
string_for_bashrc='export PATH="$PATH:'$SCRIPT_DIR/"${executable_folder_name}"'"'

if ! $( grep -Fx "$string_for_bashrc" ~/.bashrc )
then
    echo $string_for_bashrc >> ~/.bashrc
fi
cd $original_dir
