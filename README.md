<img src=media/rosez.png width="444px"/>

Docker hacks for quick and easy access to ROS2 (humble) and ROS1 (noetic) along with their GUI applications like Gazebo and rViz without the need for a local installation

## Requirements

* `docker`
* `Linux` (not necessarily Debian-based)
* `dialog` (optionally for interactive terminal UI)

## Instructions
* Make sure your system satisfies all the requirements. The installer script tries to remain distro-agnostic, thus does not install anything.
* Run the `installer.bash` script with a single argument with the rosez version you need to install. e.g. `./installer.bash ros2ez`. You can also run it interactively with no arguments. For ros-ez versions < `1.7.0`, you must use one of the `ros_*installer.bash` scripts. There are multiple, one for each supported ROS distribution.
* Run `. ~/.bashrc` or open a new terminal.
* Run `ros2ez` for ROS2 or `rosez` for ROS1 followed by the command you want to run else you will be thrown in a shell inside the image (useful if autocomplete is required).

#### Examples

`ros2ez ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=racer_01/cmd_vel`

or

`rosez rosrun rviz rviz`

#### Tips/Features/Changelog!
* `NVIDIA` users should install the [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) that massively boosts performance inside docker containers.
* If you encounter errors regarding your graphics card while trying to run the `rosez` or `ros2ez` script, try adding the `fi` or `force-integrated` flag, which will force usage of the integrated graphics card of your computer. This is, of course, not optimal, so update your drivers and try running the script again. Make sure that the flag is the first argument of your command. If this fails too, open an issue. For instance, the examples above can be executed with the integrated graphics card by running `ros2ez fi ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=racer_01/cmd_vel` and `rosez fi rosrun rviz rviz` respectively.
* You can change or add default workspaces by editing the `includes/ros*_ws.txt` file. One directory per line.
* Migrating from `v1.0.0` to newer versions will most probably introduce some permission issues, since newer versions try to stay inside the user's permissions instead of root. A chown of your workspace(s) to your user should fix the problem.
* [For versions >= `v1.1.0`] User configurable and system files have been separated, so you (most probably) should not mess with the `internals` folder.
* [For versions <= `v1.1.0`] Running multiple `ros*ez` commands concurrently (in separate terminals) can introduce race conditions, especially in workspace building (colcon (ROS2) or catkin (ROS1)). Refrain from running `ros*ez` commands concurrently, but wait for build commands to finish.
* [For versions >= `v1.2.0`] The new concurrency handling system allows for execution of multiple `ros*ez` commands concurrently (in separate terminals). The system uses lock files and tries to handle them effectively to avoid dangling ones. In the (rare?) case of a dangling lock file (essentially eternally waiting for a lock file to unlock), you should do one of the following solutions: The first one is to reboot your machine. The next time you boot up, your lock file will be older than your system's uptime, and be automatically removed. The second one is to use the `clear-locks` or `cl` flag during `ros*ez` execution. Keep in mind that if you generally use the `force-integrated` or `fi` flag, you willneed it here too. For instance, the next commands will remove **ALL** locks and then run the specified command `ros2ez fi cl ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=racer_01/cmd_vel` or for ros1 `rosez cl fi rosrun rviz rviz`. Note that the order of the flags is not important, but **have** to be placed before the required ros command.
* [For versions >= `v1.2.1`] Users are now able to use their local git configuration (ssh keys etc) and bloom release, from within the ros\*ez environment.
* [For versions >= `v1.2.2`] There was a problem that was introduced in `v1.2.1` regarding volumes and permissions for user configuration files in ssh, git and bloom. It is now fixed.
* [For versions >= `v1.2.3`] The installer script now checks if the user is in the docker group, and if not adds them and prompts for reboot at the end of the installation.
* [For versions >= `v1.3.0`] A ROS Melodic version is now available (with the `rosezm` command) and a much more robust locking mechanism is now in effect. If you experienced frequent lockouts or race conditions, this version should be more stable.
* [For versions >= `v1.4.0`] All supported ROS versions (melodic/noetic/humble) now work on a completely persistent filesystem to save apt, library and other installations + all other user-defined changes on the container. Additionally, this version also supports audio, enabling users to play sounds through the `rosez` containers.
* [For versions >= `v1.5.0`] The `sc` or `skip-compilation` flag is now supported, which completely bypasses rosdep and catkin/colcon builds for faster startup.
* [For versions >= `v1.6.0`] A ROS2 Foxy version is now available (with the `ros2ezf` command). Its image comes with built-in `ros1_bridge` support for *ez* ROS1-ROS2 integration.
* [For versions >= `v1.6.1`] A deletion script has been included inside the `internal/deeper/` folder. It takes the version-to-delete as an argument. For example, `bash delete_version.bash ros2ezf` deletes the Foxy version for rosez.
* [For versions >= `v1.7.0`] A new script to easily create systemd services was added. It can be used interactively using `dialog` or non-interactively by passing it 9 arguments. Running the `create_rosez_systemd_service.bash` with less than 9 args (but more than 0) will trigger a help message. Additionally the installation scripts have been merged into one, `installer.bash`. You can now use it interactively, or pass a single argument with the rosez version you need to install. e.g. `./installer.bash ros2ez`
* [For versions >= `v2.0.0`] Docker devices like sound and graphics are now manually handled. This version is **NOT** the same as the previous ones. This is a pretty substantial change, thus getting a major release. Externally nothing should change for the end user, but internally a lot has changed. OSRF's `rocker` is no longer utilized, making docker handling more versatile (but also fragile).

## Tested platforms
* EndeavourOS
* Fedora 35
* Ubuntu 22.04
* MX Linux 19.1
