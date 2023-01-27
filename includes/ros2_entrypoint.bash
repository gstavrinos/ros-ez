#!/bin/bash
. /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while read -r line
do
    bl="$(basename $line)"
    cd /opt/ros/$bl
    if rosdep check -i --from-path src --rosdistro humble -y | grep -q 'System dependencies have not been satisfied'; then
        apt update
    fi
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --symlink-install
    . /opt/ros/$bl/install/setup.bash
done < /opt/ros/ros2_ws.txt
. /usr/share/gazebo/setup.bash
exec "$@"
