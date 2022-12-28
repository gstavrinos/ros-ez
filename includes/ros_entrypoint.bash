#!/bin/bash
. /opt/ros/noetic/setup.bash
while read -r line
do
    bl="$(basename $line)"
    cd /opt/ros/$bl
    if rosdep check -i --from-path src --rosdistro noetic -y | grep -q 'System dependencies have not been satisfied'; then
        apt update
    fi
    rosdep install -i --from-path src --rosdistro noetic -y
    catkin_make
    . /opt/ros/$bl/devel/setup.bash
done < /opt/ros/ros_ws.txt
exec "$@"
