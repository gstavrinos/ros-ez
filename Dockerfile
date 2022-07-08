FROM osrf/ros:humble-desktop-full

LABEL maintainer="George Stavrinos <gstavrinos@protonmail.com>"

RUN mkdir -p /opt/ros/ros2_ws/src && apt-get update && apt-get install -y \
ros-humble-gazebo-ros \
ros-humble-ros2-control \
ros-humble-gazebo-plugins \
ros-humble-ros2controlcli \
ros-humble-control-toolbox \
ros-humble-ros2-controllers \
ros-humble-controller-manager \
ros-humble-effort-controllers \
ros-humble-gazebo-ros2-control \
ros-humble-velocity-controllers \
ros-humble-position-controllers \
ros-humble-joint-state-publisher \
ros-humble-transmission-interface \
ros-humble-joint-state-publisher-gui
