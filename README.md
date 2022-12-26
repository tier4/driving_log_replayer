# Driving Log Replayer for ROS2 Autoware.Universe

<b><font color="#FF0000">This branch is local use only, please do not use this branch in Autoware Evaluator</font></b>

Driving Log Replayer is a ROS package that evaluates the functionality of Autoware.Universe

## Requirements

- ROS2 humble
- [Python 3.10](https://www.python.org/)
- [pipx](https://pipxproject.github.io/pipx/)
  - **Please do not install pipx using apt**
  - See the above link, and install pipx using pip
- venv
  - sudo apt install python3-venv

### Optional

If you want to change the rosbag format from ros1 to ros2.

- [rosbags](https://gitlab.com/ternaris/rosbags)
  - pip3 install rosbags

## Installation

You need to install driving_log_replayer and driving_log_replayer_cli package.

### How to install driving_log_replayer package

Use colcon build

### How to install driving_log_replayer_cli package

Use pipx

```shell
# install
pipx install git+https://github.com/tier4/driving_log_replayer.git

# upgrade
pipx upgrade driving_log_replayer_cli

# uninstall
pipx uninstall driving_log_replayer_cli
```

## Usage

refer [document](https://tier4.github.io/driving_log_replayer/)
