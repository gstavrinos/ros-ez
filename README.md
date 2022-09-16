# ros-ez-docker-gui
Docker 'n' rocker for quick and easy access to ROS2 (humble) and ROS1 (noetic) along with their GUI applications like Gazebo and rViz without the need for a local installation

## Requirements

* `docker`
* `pip`
* `rocker` (automatically installed using pip in the installer script)
* `Linux` (not necessarily Debian-based)
* A `ros2_ws` directory located at `$HOME` for the ROS2 workspace
* A `catkin_ws` directory located at `$HOME` for the ROS1 workspace

## Instructions
* Make sure your system satisfies all the requirements. The installer script tries to remain distro-agnostic, thus does not install anything apart from `rocker` through `pip`).
* Run the `ros_installer.bash` for ROS1 or `ros2_installer.bash` for ROS2 installation.
* Run `. ~/.bashrc` or open a new terminal.
* Run `ros2ez` for ROS2 or `rosez` for ROS1 followed by the command you want to run else you will be thrown in a shell inside the image (useful if autocomplete is required).

#### Examples

`ros2ez ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=racer_01/cmd_vel`

or

`rosez rosrun rviz rviz`

#### Tips!
* If you encounter errors regarding your graphics card while trying to run the `rosez` or `ros2ez` script, try adding the `fi` or `force-integrated` flag, which will force usage of the integrated graphics card of your computer. This is, of course, not optimal, so update your drivers and try running the script again. Make sure that the flag is the first argument of your command. If this fails too, open an issue. For instance, the examples above can be executed with the integrated graphics card by running `ros2ez fi ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=racer_01/cmd_vel` and `rosez fi rosrun rviz rviz` respectively.

## Tested platforms
* EndeavourOS
* Fedora 35
* Ubuntu 22.04
* MX Linux 19.1

## TODOs
* Make workspaces customizable (not just `ros2_ws`/`catkin_ws`).
* Make adding dependencies in the docker images easier.

