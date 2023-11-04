```markdown
# ROS2 Fork of Unitree Robotics Guide

## Overview

This repository contains a ROS2 fork of the original `unitree_robotics/unitree_guide` package, adapted to work with the latest ROS2 ecosystem. It is intended for use with the Unitree GO1 robot and includes the necessary launch files, configurations, and dependency information for ROS2.

The package aims to provide an example for robotics enthusiasts and researchers looking to integrate Unitree robots with ROS2.

## Current Status: Prototype

As of the current release, the package can successfully launch the GO1 robot within a ROS2 environment. However, the control parameters are not properly configured, leading to unpredictable behavior when attempting to control the robot.

**Important Note:** This package is in a very early prototype stage. If you launch the robot with this package, the GO1 robot may exhibit erratic behavior due to incorrect parameter settings.

## Getting Started

To get started with this package, clone the repository into your ROS2 workspace:

```bash
git clone https://github.com/Wataru-Oshima-Tokyo/unitree_guide2.git
```

Then, compile the workspace:

```bash
cd path_to_your_ros2_workspace
colcon build 
source install/setup.bash
```

To launch the robot, use the provided launch files:

(terminal1)
```bash
ros2 launch unitree_guide2 launch_world.launch.py 
```
(terminal2)
```bash
ros2 launch unitree_ros2_guide spawn_go1.launch.py 
```

## Contributions

This package is open for contributions! If you have experience with Unitree robots and ROS2, your help would be highly appreciated to improve the parameter configurations. Here's how you can contribute:

- **Issue Reporting**: If you find any issues, please report them using the GitHub Issues feature.
- **Pull Requests**: Feel free to fork the repository, make improvements, especially to the parameter configurations, and submit a pull request.
- **Discussion**: Start or join a discussion about the best practices for configuring Unitree robots in ROS2.

## Troubleshooting

If you encounter issues with the package, please check the GitHub Issues to see if it's a known problem or start a new issue providing as much detail as possible about your environment and the problem you are experiencing.

## License

This package inherits the license from the original `unitree_robotics/unitree_guide`. Please refer to the LICENSE file for more details.

## Acknowledgements

This ROS2 adaptation is based on the work done by Unitree Robotics. We thank them for their contributions to the robotics community and for providing a platform to learn and experiment with quadruped robots.
```
