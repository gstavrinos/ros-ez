#!/bin/bash
. /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /opt/ros/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
. /opt/ros/ros2_ws/install/setup.bash
exec "$@"
