#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cwd=$(pwd)
cd $cwd
source $SCRIPT_DIR/helpers.bash
now="$(date +'%Y-%m-%d_%H_%M_%S_%N')"
uptime_date="$(uptime -s | sed "s/[: ]/_/g")"
# Using uptime_date I can see if a lock file is older
# than the current uptime and thus safely delete it.
# This should enable the "a reboot should fix it" behaviour
earliest_possible_lock_file="$lock_prefix$uptime_date$lock_suffix"
lock_file="$lock_prefix$now$lock_suffix"
known_params=("force-integrated" "clear-locks")
known_params_short=("fi" "cl")
rosws_file="ros2_ws.txt"
rosez_vol="ros2ez-volume"
ros="humble"
ros_image="ros2_ez"
gpu_string=$(lspci | grep VGA)
gpu_param=""
clear_locks=0
lockdir=""
if [ "$1" == "1" ]; then
    rosws_file="ros_ws.txt"
    rosez_vol="rosez-volume"
    ros="noetic"
    ros_image="ros_ez"
elif [ "$1" == "3" ]; then
    rosws_file="rosm_ws.txt"
    rosez_vol="rosezm-volume"
    ros="melodic"
    ros_image="ros_ezm"
fi
shift
trap 'signal_handler' $signal_list
volumes=""
lockation=""
while read -r line
do
    wsdir=$(eval echo -e "$line")
    volumes=$volumes"--volume $wsdir:/opt/ros/$(basename $wsdir) "
    if [[ -z $lockation ]]; then
        # lockdir=$(basename wsdir)
        lockation=$wsdir
    fi
done < $SCRIPT_DIR/../includes/$rosws_file
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
if [ $clear_locks -gt 0 ]; then
    echo -e "${colour_orange}I was passed the clear-locks flag. Deleting all lock files...$colour_end"
    rm -f $lockation/$lock_prefix*$lock_suffix
fi
echo $gpu_param
touch $lockation/$lock_file
locked=1
while [ $locked -gt 0 ]; do
    found_lock="$(find $lockation -maxdepth 1 -name "$lock_prefix*$lock_suffix" -print | sort | head -1)"
    # The found_lock should be empty only
    # in the (rare?) case of enabling the 
    # clear-locks (cl) flag from another rosez process
    # ---
    # A new lock_file is created, since the other rosez
    # process will instantly get priority with the cl flag.
    if [ -z "$found_lock" ]; then
        now="$(date +'%Y-%m-%d_%H_%M_%S_%N')"
        lock_file="$lock_prefix$now$lock_suffix"
        sudo touch $lockation/$lock_file
        found_lock=$lockation/$lock_file
    fi
    fl="$(basename $found_lock)"
    echo -e "${colour_blue}Processing: $fl$colour_end"
    if [ "$fl" \< "$earliest_possible_lock_file" ]; then
        echo -e "${colour_orange}Found problematic lock file ($fl)! Deleting it... (Lock file older than uptime)$colour_end"
        sudo rm $found_lock
    elif [ "$fl" \< "$lock_file" ]; then
        echo -e "${colour_orange}Found earlier lock file ($fl) than my own ($lock_file)!\nWaiting...$colour_end"
        sleep 3
    elif [ "$fl" == "$lock_file" ]; then
        echo -e "${colour_green}My lock file ($lock_file) is the earliest found. Continuing...$colour_end"
        locked=0
    fi
done
bloom_file=/home/$USER/.config/bloom
gitconfig_file=/home/$USER/.gitconfig
ssh_folder=/home/$USER/.ssh
if [ ! -f  $bloom_file ]; then
    touch $bloom_file
fi
if [ ! -f  $gitconfig_file ]; then
    touch $gitconfig_file
fi
if [ ! -d  $ssh_folder ]; then
    mkdir $ssh_folder
fi
intermediate_error_handler $?
x=""$(rocker --mode dry-run --network host --x11 $gpu_param --volume $rosez_vol:/opt/ros/$ros/ --volume $SCRIPT_DIR/../includes/$rosws_file:/opt/ros/$rosws_file $volumes $SCRIPT_DIR/../internal/entrypoint.bash:/home/rosez_user/.bashrc $bloom_file:/home/rosez_user/.config/bloom $gitconfig_file:/home/rosez_user/.gitconfig $ssh_folder:/home/rosez_user/.ssh $SCRIPT_DIR/helpers.bash:/home/rosez_user/helpers.bash -- $ros_image:latest | tail -n 1 | sed -e "s/-v .*$rosez_vol:/-v $rosez_vol:/")
intermediate_error_handler $?
xauthf="$((echo \"$x\") | grep -E -o '/tmp/.docker[a-zA-Z0-9_-]+.xauth' | head -1)"
intermediate_error_handler $?
touch $xauthf
intermediate_error_handler $?
/bin/bash -c "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $xauthf nmerge -"
intermediate_error_handler $?
cl=""
extras="env $ENV LOCKFILE=$lock_file /bin/bash"
if [ $# -gt 0 ]; then
    extras=$extras" -c \"source /home/rosez_user/.bashrc && $* \""
fi
x="$x $extras"
userid=$(id -u)
groupid=$(id -g)
x=${x/docker run --rm -it/docker run --rm -it -u $userid --ipc=host --privileged}
intermediate_error_handler $?
printf "Executing:\n---\n$x\n---\n"
eval "$x"
