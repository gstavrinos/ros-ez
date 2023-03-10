#!/bin/bash
known_params=("force-integrated" "clear-locks")
known_params_short=("fi" "cl")
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
clear_locks=0
bloom_file=/home/$USER/.config/bloom
gitconfig_file=/home/$USER/.gitconfig
ssh_folder=/home/$USER/.ssh
# gpg_keys_folder=/home/$USER/.gnupg
if [ ! -f  $bloom_file ]; then
    touch $bloom_file
fi
if [ ! -f  $gitconfig_file ]; then
    touch $gitconfig_file
fi
if [ ! -d  $ssh_folder ]; then
    mkdir $ssh_folder
fi
# if [ ! -d  $gpg_keys_folder ]; then
    # mkdir $gpg_keys_folder
# fi
# I am not using the index here,
# but I make sure that I cycle through
# the (at most) first i parameters
# to catch all the flags
for i in "${!known_params[@]}"; do
    if [ "${known_params[0]}" == "$1" ] || [ "${known_params_short[0]}" == "$1" ]; then
        gpu_param="--device /dev/dri/card0"
        shift
    elif [ "${known_params[1]}" == "$1" ] || [ "${known_params_short[1]}" == "$1" ]; then
        clear_locks=1
        shift
    else
        break
    fi
done
if [ -z "$gpu_param" ]; then
    if grep -q "nvidia" <<< "$gpu_string" || grep -q "Nvidia" <<< "$gpu_string" || grep -q "NVIDIA" <<< "$gpu_string"
    then
        gpu_param="--nvidia"
    elif grep -q "intel" <<< "$gpu_string" || grep -q "Intel" <<< "$gpu_string" || grep -q "INTEL" <<< "$gpu_string"
    then
        gpu_param="--device /dev/dri/card0"
    else
        echo "No Nvidia or Intel GPU found. This case has not been investigated yet. GUI integration might be broken. (Good luck!)"
    fi
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
# x=""$(rocker --mode dry-run --network host --x11 $gpu_param --volume $rosez_vol:/opt/ros/$ros/ --volume $SCRIPT_DIR/../includes/$rosws_file:/opt/ros/$rosws_file $volumes $SCRIPT_DIR/../internal/entrypoint.bash:/home/rosez_user/.bashrc $bloom_file:/home/rosez_user/.config/bloom $gitconfig_file:/home/rosez_user/.gitconfig $ssh_folder:/home/rosez_user/.ssh $gpg_keys_folder:/home/rosez_user/.gnupg $bloom_file:$bloom_file $gitconfig_file:$gitconfig_file $ssh_folder:$ssh_folder $gpg_keys_folder:$gpg_keys_folder -- $ros_image:latest | tail -n 1 | sed -e "s/-v .*$rosez_vol:/-v $rosez_vol:/")
x=""$(rocker --mode dry-run --network host --x11 $gpu_param --volume $rosez_vol:/opt/ros/$ros/ --volume $SCRIPT_DIR/../includes/$rosws_file:/opt/ros/$rosws_file $volumes $SCRIPT_DIR/../internal/entrypoint.bash:/home/rosez_user/.bashrc $bloom_file:/home/rosez_user/.config/bloom $gitconfig_file:/home/rosez_user/.gitconfig $ssh_folder:/home/rosez_user/.ssh $bloom_file:$bloom_file $gitconfig_file:$gitconfig_file $ssh_folder:$ssh_folder -- $ros_image:latest | tail -n 1 | sed -e "s/-v .*$rosez_vol:/-v $rosez_vol:/")
xauthf="$((echo \"$x\") | grep -E -o '/tmp/.docker[a-zA-Z0-9_-]+.xauth' | head -1)"
touch $xauthf
/bin/bash -c "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $xauthf nmerge -"
cl=""
if [ $clear_locks -gt 0 ]; then
    cl="ros-ez-CL"
fi
extras="env $ENV ROSEZCLEARLOCKS="$cl" /bin/bash"
if [ $# -gt 0 ]; then
    extras=$extras" -c \"source /home/rosez_user/.bashrc && $* \""
fi
x="$x $extras"
userid=$(id -u)
groupid=$(id -g)
x=${x/docker run --rm -it/docker run --rm -it -u $userid:$groupid --ipc=host}
printf "Executing:\n---\n$x\n---\n"
eval "$x"
