#!/bin/bash
. /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while read -r line
do
    bl="$(basename $line)"
    cd /opt/ros/$bl
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --symlink-install
    . /opt/ros/$bl/install/setup.bash
done < /opt/ros/ros2_ws.txt
exec "$@"
