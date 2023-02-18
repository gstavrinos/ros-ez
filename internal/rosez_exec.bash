#!/bin/bash
rosws_file="ros2_ws.txt"
rosez_vol="ros2ez-volume"
ros="humble"
ros_image="ros2_ez"
if [ "$1" == "1" ]; then
    rosws_file="ros_ws.txt"
    rosez_vol="rosez-volume"
    ros="noetic"
    ros_image="ros_ez"
fi
shift
gpu_string=$(lspci | grep VGA)
gpu_param=""
if grep -q "force-integrated" <<< "$1" || grep -q "fi" <<< "$1"
then
    gpu_param="--device /dev/dri/card0"
    shift
elif grep -q "nvidia" <<< "$gpu_string" || grep -q "Nvidia" <<< "$gpu_string" || grep -q "NVIDIA" <<< "$gpu_string"
then
    gpu_param="--nvidia"
elif grep -q "intel" <<< "$gpu_string" || grep -q "Intel" <<< "$gpu_string" || grep -q "INTEL" <<< "$gpu_string"
then
    gpu_param="--device /dev/dri/card0"
else
    echo "No Nvidia or Intel GPU found. This case has not been investigated yet. GUI integration might be broken. (Good luck!)"
fi
echo $gpu_param
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
volumes=""
cwd=$(pwd)
while read -r line
do
    wsdir=$(eval echo -e "$line")
    volumes=$volumes"--volume $wsdir:/opt/ros/$(basename $wsdir) "
done < $SCRIPT_DIR/../includes/$rosws_file
cd $cwd
x=""$(rocker --mode dry-run --network host --x11 $gpu_param --volume $rosez_vol:/opt/ros/$ros/ --volume $SCRIPT_DIR/../includes/$rosws_file:/opt/ros/$rosws_file $volumes $SCRIPT_DIR/../internal/entrypoint.bash:/home/rosez_user/.bashrc -- $ros_image:latest | tail -n 1 | sed -e "s/-v .*$rosez_vol:/-v $rosez_vol:/")
xauthf="$((echo \"$x\") | grep -E -o '/tmp/.docker[a-zA-Z0-9_-]+.xauth' | head -1)"
touch $xauthf
/bin/bash -c "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $xauthf nmerge -"
extras="env $ENV /bin/bash"
if [ $# -gt 0 ]
then
    extras=$extras" -c \"source /home/rosez_user/.bashrc && $* \""
fi
x="${x} ${extras}"
userid=$(id -u)
groupid=$(id -g)
x=${x/docker run --rm -it/docker run --rm -it -u $userid:$groupid --ipc=host}
printf "Executing:\n---\n$x\n---\n"
eval "$x"
