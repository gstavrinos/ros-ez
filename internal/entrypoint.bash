#!/bin/bash
signal_list=(INT QUIT TSTP TERM HUP KILL)
colour_end='\033[0m'
colour_red='\033[0;31m'
colour_green='\033[0;32m'
colour_orange='\033[0;33m'
colour_blue='\033[0;34m'
lock_prefix=".rosez-"
lock_suffix=".lock"
now="$(date +'%Y-%m-%d_%H_%M_%S_%N')"
uptime_date="$(uptime -s | sed "s/[: ]/_/g")"
# Using uptime_date I can see if a lock file is older
# than the current uptime and thus safely delete it.
# This should enable the "a reboot should fix it" behaviour
earliest_possible_lock_file="$lock_prefix$uptime_date$lock_suffix"
lock_file="$lock_prefix$now$lock_suffix"
rosversion="unknown"
lockation=""
wstxt="ros2_ws.txt"
if [ -f /opt/ros/humble/setup.bash ]; then
    rosversion="humble"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    rosversion="noetic"
    wstxt="ros_ws.txt"
fi
lockation="/opt/ros/$rosversion"
if [ "$ROSEZCLEARLOCKS" == "ros-ez-CL" ]; then
    echo -e "${colour_orange}I was passed the clear-locks flag. Deleting all lock files...$colour_end"
    sudo rm -f $lockation/$lock_prefix*$lock_suffix
    shift
fi
# This function is used to trap signals
function signal_handler() {
    echo -e "${colour_red}Execution was aborted, deleting my lock_file ($lock_file)$colour_end"
    sudo rm -f $lockation/$lock_file
    exit
}
# This function is used to check intermediate commands'
# exit codes and then terminate if there was an error
function intermediate_error_handler() {
    if [ $1 -gt 0 ]; then
        echo -e "${colour_red}Intermediate process aborted!$colour_end"
        signal_handler
    fi
}
trap 'signal_handler' $signal_list
if [ "$rosversion" != "unknown" ]; then
    sudo touch $lockation/$lock_file
    . /opt/ros/$rosversion/setup.bash
    if [ "$rosversion" == "humble" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
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
    # From now on, "lengthy" commands (like colcon build)
    # are assigned to an (unused) variable, thus bash
    # will report an exit code > 0 ONLY when the command
    # is interrupted. This is a required behaviour, in order
    # to enter the shell (not exit) when a build fails.
    if [ $locked -eq 0 ]; then
        while read -r line; do
            bl="$(basename $line)"
            cd /opt/ros/$bl
            if rosdep check -i --from-path src --rosdistro $rosversion -y | grep -q 'System dependencies have not been satisfied'; then
                intermediate_error_handler $?
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "sudo apt update" | tee /dev/fd/2)

                intermediate_error_handler $?
            fi
            output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "rosdep install -i --from-path src --rosdistro $rosversion -y" | tee /dev/fd/2)
            intermediate_error_handler $?
            if [ "$rosversion" == "humble" ]; then
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" | tee /dev/fd/2)
                intermediate_error_handler $?
                . /opt/ros/$bl/install/setup.bash
                intermediate_error_handler $?
            else
                output=$(script --flush --quiet --return /tmp/ansible-output.txt --command "catkin_make" | tee /dev/fd/2)
                intermediate_error_handler $?
                . /opt/ros/$bl/devel/setup.bash
                intermediate_error_handler $?
            fi
        done < /opt/ros/$wstxt
        . /usr/share/gazebo/setup.bash
        intermediate_error_handler $?
        echo -e "${colour_green}Unlocking my lock file ($lock_file) for other processes...$colour_end"
        sudo rm $lockation/$lock_file
        exec "$@"
        trap - $signal_list
    else
        echo -e "${colour_red}Encountered a very weird error. We shouldn't have come here... Exiting...$colour_end"
        sudo rm $lockation/$lock_file
        exit
    fi
fi
