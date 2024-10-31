#!/bin/bash
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cwd=$(pwd)
cd $cwd
source $SCRIPT_DIR/helpers.bash
get_supported_versions
now="$(date +'%Y-%m-%d_%H_%M_%S_%N')"
uptime_date="$(uptime -s | sed "s/[: ]/_/g")"
# Using uptime_date I can see if a lock file is older
# than the current uptime and thus safely delete it.
# This should enable the "a reboot should fix it" behaviour
earliest_possible_lock_file="$lock_prefix$uptime_date$lock_suffix"
lock_file="$lock_prefix$now$lock_suffix"
rosws_file="ros2h_ws.txt"
rosez_vol="ros2ezh-volume"
ros="humble"
ros_image="ros2_ezh"
gpu_string=$(lspci | grep VGA)
gpu_param=""
clear_locks=0
skip_compilation=0
non_interactive=0
no_sound=0
# lockdir=""
userid=$(id -u)
rosws_file=${workspaces[$($1-1)]}
rosez_vol="${volumes[$($1-1)]}"
ros="${distros[$($1-1)]}"
ros_image="${image_names[$($1-1)]}"
shift
trap 'signal_handler' $signal_list
rosez_volumes=""
lockation=""
while read -r line; do
  wsdir=$(eval echo -e "$line")
  if [ ! -d "$wsdir" ]; then
    echo "$wsdir not found, creating it"
    mkdir -p $wsdir/src
  fi
  rosez_volumes=$rosez_volumes"--volume $wsdir:/opt/ros/$(basename $wsdir) "
  if [[ -z "$lockation" ]]; then
    lockation=$wsdir
  fi
done <$SCRIPT_DIR/../includes/$rosws_file
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
  elif [ "${known_params[2]}" == "$1" ] || [ "${known_params_short[2]}" == "$1" ]; then
    skip_compilation=1
    shift
  elif [ "${known_params[3]}" == "$1" ] || [ "${known_params_short[3]}" == "$1" ]; then
    non_interactive=1
    shift
  elif [ "${known_params[4]}" == "$1" ] || [ "${known_params_short[4]}" == "$1" ]; then
    no_sound=1
    shift
  else
    break
  fi
done
if [ -z "$gpu_param" ]; then
  if grep -q "nvidia" <<<"$gpu_string" || grep -q "Nvidia" <<<"$gpu_string" || grep -q "NVIDIA" <<<"$gpu_string"; then
    gpu_param="--gpus all"
  elif grep -q "intel" <<<"$gpu_string" || grep -q "Intel" <<<"$gpu_string" || grep -q "INTEL" <<<"$gpu_string"; then
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
  echo $found_lock
  if [[ -z "$found_lock" ]]; then
    now="$(date +'%Y-%m-%d_%H_%M_%S_%N')"
    lock_file="$lock_prefix$now$lock_suffix"
    touch $lockation/$lock_file
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
if [ ! -f $bloom_file ]; then
  touch $bloom_file
fi
if [ ! -f $gitconfig_file ]; then
  touch $gitconfig_file
fi
if [ ! -d $ssh_folder ]; then
  mkdir $ssh_folder
fi
intermediate_error_handler $?
sound="-v /run/user/$userid/pulse:/run/user/$userid/pulse --device /dev/snd -e PULSE_SERVER=unix:/run/user/$userid/pulse/native -v /run/user/$userid/pulse/native:/run/user/$userid/pulse/native"
if [ $no_sound -gt 0 ]; then
  sound=""
fi
it="-it"
if [ $non_interactive -gt 0 ]; then
  it=""
fi
xauthf="/tmp/.$ros_image-$now.xauth"
touch $xauthf
intermediate_error_handler $?
/bin/bash -c "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $xauthf nmerge -"
intermediate_error_handler $?
x="docker run --ulimit nofile=1024:524288 --rm $it -u $userid --ipc=host --privileged --network host $gpu_param $sound --group-add dialout --group-add video --group-add audio -v $rosez_vol-bin:/bin -v $rosez_vol-etc:/etc/ -v $rosez_vol-home:/home/ -v $rosez_vol-lib:/lib/ -v $rosez_vol-lib64:/lib64/ -v /mnt:/mnt -v $rosez_vol-opt:/opt/ -v $rosez_vol-root:/root/ -v /run:/run -v $rosez_vol-sbin:/sbin/ -v $rosez_vol-srv:/srv/ -v $rosez_vol-usr:/usr -v $rosez_vol-var:/var -v $rosez_vol:/opt/ros/$ros -v $SCRIPT_DIR/../includes/$rosws_file:/opt/ros/$rosws_file $rosez_volumes -v $SCRIPT_DIR/entrypoint.bash:/home/rosez_user/.bashrc -v /sys:/sys -v /dev:/dev -v $bloom_file:/home/rosez_user/.config/bloom -v $gitconfig_file:/home/rosez_user/.gitconfig -v $ssh_folder:/home/rosez_user/.ssh -v $SCRIPT_DIR/supported_versions.txt:/home/rosez_user/supported_versions.txt -v $SCRIPT_DIR/helpers.bash:/home/rosez_user/helpers.bash -v /:$HOME/.$rosez_vol -e DISPLAY -e TERM -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=$xauthf -v $xauthf:$xauthf -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro $ros_image:latest"
intermediate_error_handler $?
extras="env $ENV LOCKFILE=$lock_file SKIPCOMPILATION=$skip_compilation /bin/bash"
if [ $# -gt 0 ]; then
  extras=$extras" -c \"source /home/rosez_user/.bashrc && $* \""
fi
x="$x $extras"
intermediate_error_handler $?
printf "Executing:\n---\n$x\n---\n"
eval "$x"
