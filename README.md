# ROS2 Workspace
This is my (George Kouretas') ROS2 workspace aimed at developing a rehabilitative robotic system using a UR10E robot and real-time biofeedback signals.

## Setup
This workspace is developed using ROS2 `humble`. 

To install, follow the provided directions found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

In addition to the base installation, `ros2_control` is required and can be installed with the following commands:
```bash
$ sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
$ sudo apt-get install ros-humble-xacro
```

Additionally, the real-time or low latency kernel is recommended to be installed when using the UR driver. Instructions for this can be found [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html#real-time-setup).


For URSim, docker must be installed (instructions for Ubuntu 22.04 can be found [here](https://www.cherryservers.com/blog/install-docker-ubuntu-22-04)). To set up the URSim, the steps found [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/ursim_docker.html) may be performed.

## Configuration
To setup the workspace, run the following git command to checkout a branch and update all the active submodules:
```bash
git checkout tag/working-branch && git submodule update --init --recursive
```

## Build
If not already installed, install colcon for building the workspace with the following command:
```bash
$ sudo apt install python3-colcon-common-extensions
```

From the workspace folder, run the following command to build the complete workspace:
```bash
$ colcon build --symlink-install
$ source install/setup.bash
```

# ROS2 Workspace
## Helpful links
### Visualization
[rqt](http://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html)
### Universal robots
[ROS2 Driver Repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
[UR Simulator Setup Instructions](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)
[RT Platform Setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html#real-time-setup)

Click [here](http://localhost:6080/vnc_auto.html) to access interface