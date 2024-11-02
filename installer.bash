#!/bin/bash
original_dir=$(pwd)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
source $SCRIPT_DIR/internal/helpers.bash
image_name=""
dockerfile=""
executable_folder_name=""
get_supported_versions
selected_version=$1
if [[ -z "$1" ]] || [ "$1" == "--interactive" ]; then
  if ! type "dialog" >/dev/null 2>&1; then
    echo "'dialog' (https://linux.die.net/man/1/dialog) is not installed, please install it and run the script again!"
    echo "You can alternatively use the non-interactive version by running the installer with one of the following parameters depending on the ROS version you need:"
    for i in $(seq 0 $((${#distros[@]} - 1))); do
      echo "${rosezv[$i]} for ${distros[$i]}"
    done
    exit 123
  else
    echo "going interactive"
    for i in $(seq 0 $((${#distros[@]} - 1))); do
      choices="$choices\"${rosezv[$i]}\" \"${distros[$i]}\" \"on\" "
    done
    run_dialog_command "--radiolist" "Select a rosez version to install" ${#distros[@]} "$choices"
    clear
    selected_version=$choice
  fi
fi
for i in $(seq 0 $((${#distros[@]} - 1))); do
  if [ "$selected_version" == "${rosezv[$i]}" ]; then
    image_name="${image_names[$i]}"
    dockerfile="${dockerfiles[$i]}"
    executable_folder_name="${executable_folder_names[$i]}"
    break
  fi
done
if [[ -z "$image_name" ]]; then
  echo "Invalid argument passed. You can run the interactive version by using the --interactive flag (or no flags at all)."
  echo "You can alternatively use the non-interactive version by running the installer with one of the following parameters depending on the ROS version you need:"
  for i in $(seq 0 $((${#distros[@]} - 1))); do
    echo "${rosezv[$i]} for ${distros[$i]}"
  done
  exit 234
fi
cd $SCRIPT_DIR
need_rr=0
if ! id -nGz "$USER" | grep -qzxF "docker"; then
  sudo usermod -aG docker $USER
  need_rr=1
fi
sudo docker build --no-cache --pull -t $image_name -f $SCRIPT_DIR/internal/$dockerfile . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)
string_for_bashrc='export PATH="$PATH:'$SCRIPT_DIR/internal/"${executable_folder_name}"'"'

if ! $(grep -Fx "$string_for_bashrc" ~/.bashrc); then
  echo $string_for_bashrc >>~/.bashrc
  . ~/.bashrc
fi
if [[ $need_rr -gt 0 ]]; then
  echo "Please reboot your system, and then try using ros-ez!"
fi
cd $original_dir
