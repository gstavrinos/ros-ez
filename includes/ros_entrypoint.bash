#!/bin/bash
. /opt/ros/noetic/setup.bash
while read -r line
do
    bl="$(basename $line)"
    cd /opt/ros/$bl
    rosdep install -i --from-path src --rosdistro noetic -y
    catkin_make
    . /opt/ros/$bl/devel/setup.bash
done < /opt/ros/ros_ws.txt
exec "$@"
