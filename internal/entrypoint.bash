#!/bin/bash
lock_file=$LOCKFILE
skip_compilation=$SKIPCOMPILATION
rosversion="unknown"
lockation=""
wstxt="ros2_ws.txt"
if [ -f /opt/ros/humble/setup.bash ]; then
    rosversion="humble"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    rosversion="foxy"
    wstxt="ros2f_ws.txt"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    rosversion="noetic"
    wstxt="ros_ws.txt"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    rosversion="melodic"
    wstxt="rosm_ws.txt"
fi
read -r lockdir</opt/ros/$wstxt
lockation="/opt/ros/$(basename $lockdir)"
source /home/rosez_user/helpers.bash
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
