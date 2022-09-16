#!/bin/bash
. /opt/ros/noetic/setup.bash
cd /opt/ros/catkin_ws
rosdep install -i --from-path src --rosdistro noetic -y
catkin_make
. /opt/ros/catkin_ws/install/setup.bash
exec "$@"
