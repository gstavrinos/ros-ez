#!/bin/bash
rosversion="unknown"
wstxt="ros2_ws.txt"
if [ -f /opt/ros/humble/setup.bash ]; then
    rosversion="humble"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    rosversion="noetic"
    wstxt="ros_ws.txt"
fi
if [ "$rosversion" != "unknown" ]; then
    . /opt/ros/$rosversion/setup.bash
    if [ "$rosversion" == "humble" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
    while read -r line
    do
        bl="$(basename $line)"
        wstxt="ros2_ws.txt"
        cd /opt/ros/$bl
        if rosdep check -i --from-path src --rosdistro $rosversion -y | grep -q 'System dependencies have not been satisfied'; then
            apt update
        fi
        rosdep install -i --from-path src --rosdistro $rosversion -y
        if [ "$rosversion" == "humble" ]; then
            colcon build --symlink-install
            . /opt/ros/$bl/install/setup.bash
        else
            catkin_make
            . /opt/ros/$bl/devel/setup.bash
        fi
    done < /opt/ros/$wstxt
    . /usr/share/gazebo/setup.bash
    exec "$@"
fi
