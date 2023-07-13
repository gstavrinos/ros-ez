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
elif [ "$1" == "3" ]; then
    image_name="ros_ezm"
    volume="rosezm-volume"
    executable_folder_name="ros-ezm"
    dockerfile="ros_melodic_Dockerfile"
elif [ "$1" == "4" ]; then
    image_name="ros2_ezf"
    volume="ros2ezf-volume"
    executable_folder_name="ros2-ezf"
    dockerfile="ros2_foxy_Dockerfile"
fi
sudo pip install rocker==0.2.10
cd $SCRIPT_DIR
need_rr=0
if ! id -nGz "$USER" | grep -qzxF "docker"
then
    sudo usermod -aG docker $USER
    need_rr=1
fi
sudo docker build --no-cache --pull -t $image_name -f $SCRIPT_DIR/$dockerfile . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)
string_for_bashrc='export PATH="$PATH:'$SCRIPT_DIR/"${executable_folder_name}"'"'

if ! $( grep -Fx "$string_for_bashrc" ~/.bashrc )
then
    echo $string_for_bashrc >> ~/.bashrc
    . ~/.bashrc
fi
if [[ $need_rr -gt 0 ]]
then
    echo "Please reboot your system, and then try using ros-ez!"
fi
cd $original_dir
