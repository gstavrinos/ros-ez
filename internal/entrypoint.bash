#!/bin/bash
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
    sudo rm -f $lockation/$lock_prefix*$lock_suffix
    shift
fi
function handler() {
    echo lalala
    sudo rm -f $lockation/$lock_file
    exit
}
trap 'handler' INT QUIT TSTP TERM HUP KILL
if [ "$rosversion" != "unknown" ]; then
    sudo touch $lockation/$lock_file
    . /opt/ros/$rosversion/setup.bash
    if [ "$rosversion" == "humble" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
    locked=1
    while [ $locked -gt 0 ]; do
        found_lock="$(find $lockation -maxdepth 1 -name "$lock_prefix*$lock_suffix" -print | sort | head -1)"
        fl="$(basename $found_lock)"
        if [ "$fl" \< "$earliest_possible_lock_file" ]; then
            echo found problematic lock file. deleting it
            sudo rm $found_lock
        elif [ "$fl" \< "$lock_file" ]; then
            echo found earlier lock
            echo found rosez lock waiting...
            sleep 3
        elif [ "$fl" == "$lock_file" ]; then
            echo unlocking...
            locked=0
        fi
    done
    if [ $locked -eq 0 ]; then
        while read -r line; do
            bl="$(basename $line)"
            cd /opt/ros/$bl
            if rosdep check -i --from-path src --rosdistro $rosversion -y | grep -q 'System dependencies have not been satisfied'; then
                sudo apt update
            fi
            rosdep install -i --from-path src --rosdistro $rosversion -y
            if [ "$rosversion" == "humble" ]; then
                colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
                . /opt/ros/$bl/install/setup.bash
            else
                catkin_make
                . /opt/ros/$bl/devel/setup.bash
            fi
        done < /opt/ros/$wstxt
        . /usr/share/gazebo/setup.bash
        sudo rm $lockation/$lock_file
        exec "$@"
    else
        echo Encountered a very weird error. Exiting...
        sudo rm $lockation/$lock_file
        exit
    fi
fi
