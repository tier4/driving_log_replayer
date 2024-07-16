# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `log_evaluator`.

## Requirements

- CPU amd64
- Ubuntu 22.04
- ROS humble
- Python 3.10
- NVIDIA GPU (required if running perception)
- [zstd](https://github.com/facebook/zstd)
  - sudo apt install zstd

## How to build

1. Navigate to the Autoware workspace:

   ```shell
   cd autoware
   ```

2. Add dependency packages:

   ```shell
   nano simulator.repos
   # add repositories below.
   ```

   ```shell
     simulator/perception_eval:
       type: git
       url: https://github.com/tier4/autoware_perception_evaluation.git
       version: main
     simulator/log_evaluator:
       type: git
       url: https://github.com/tier4/log_evaluator.git
       version: main
     simulator/vendor/ros2_numpy:
       type: git
       url: https://github.com/Box-Robotics/ros2_numpy.git
       version: humble
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
   ```

3. Import Simulator dependencies:

   ```shell
   vcs import src < simulator.repos
   ```

4. Update rosdep:

   ```shell
   rosdep update
   ```

5. Install dependent ROS packages:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. Build the workspace:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
