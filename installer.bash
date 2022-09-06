#!/bin/bash
sudo pip install rocker
sudo docker build -t ros2_ez .
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo "export PATH=\"\$PATH:$SCRIPT_DIR\"" >> ~/.bashrc
