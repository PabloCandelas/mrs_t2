# mrs_t2

This repository contains ROS 2 (Humble) packages for MRS T2 experiments, exercises, and prototypes.

## Build

From the workspace root (`~/ros2_ws`):

```bash
cd ~/ros2_ws
colcon build --symlink-install --parallel-workers 6
source install/setup.bash

colcon build --symlink-install --packages-select <your_package_name> --parallel-workers 6

