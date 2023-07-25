#!/bin/bash
source /home/rosez_user/helpers.bash
get_supported_versions
lock_file=$LOCKFILE
skip_compilation=$SKIPCOMPILATION
rosversion="unknown"
lockation=""
wstxt=""
for i in $(seq 0 $(( ${#distros[@]}-1 )) ); do
    if [ -f /opt/ros/"${distros[$i]}"/setup.bash ]; then
        rosversion="${distros[$i]}"
        wstxt="${workspaces[$i]}"
    fi
done
read -r lockdir</opt/ros/$wstxt
lockation="/opt/ros/$(basename $lockdir)"
if [ "$ROSEZCLEARLOCKS" == "ros-ez-CL" ]; then
    echo -e "${colour_orange}I was passed the clear-locks flag. Deleting all lock files...$colour_end"
    sudo rm -f $lockation/$lock_prefix*$lock_suffix
    shift
fi
trap 'signal_handler' $signal_list
if [ "$rosversion" != "unknown" ]; then
    # sudo touch $lockation/$lock_file
    . /opt/ros/$rosversion/setup.bash
    if [ "$rosversion" == "humble" ] || [ "$rosversion" == "foxy" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
    while read -r line; do
        bl="$(basename $line)"
        cd /opt/ros/$bl
        if [ $skip_compilation -ne 1 ]; then
            if rosdep check -ir --from-path src --rosdistro $rosversion -y | grep -q 'System dependencies have not been satisfied'; then
                intermediate_error_handler $?
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "sudo apt update" | tee /dev/fd/2)

                intermediate_error_handler $?
            fi
            output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "rosdep install -ir --from-path src --rosdistro $rosversion -y" | tee /dev/fd/2)
            intermediate_error_handler $?
        fi
        if [ "$rosversion" == "humble" ] || [ "$rosversion" == "foxy" ]; then
            if [ $skip_compilation -ne 1 ]; then
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" | tee /dev/fd/2)
                intermediate_error_handler $?
            fi
            . /opt/ros/$bl/install/setup.bash
            intermediate_error_handler $?
        else
            if [ $skip_compilation -ne 1 ]; then
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "catkin_make" | tee /dev/fd/2)
                intermediate_error_handler $?
            fi
            . /opt/ros/$bl/devel/setup.bash
            intermediate_error_handler $?
        fi
    done < /opt/ros/$wstxt
    if [ "$rosversion" == "melodic" ]; then
        . /usr/share/gazebo/setup.sh
    else
        . /usr/share/gazebo/setup.bash
    fi
    intermediate_error_handler $?
    echo -e "${colour_green}Unlocking my lock file ($lock_file) for other processes...$colour_end"
    sudo rm $lockation/$lock_file
    exec "$@"
    trap - $signal_list
fi
