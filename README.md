# Driving Log Replayer for ROS 2 Autoware.Universe

Driving Log Replayer is a ROS package that evaluates the functionality of Autoware.Universe

## Requirements

- ROS 2 humble
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

### Shell Completion

Execute the following command so that you can complete the command in the shell.

#### bash

```shell
_DLR_COMPLETE=bash_source dlr > $HOME/.dlr-complete.bash
_DLR_COMPLETE=bash_source dlr > $HOME/.dlr-analyzer-complete.bash

echo "source $HOME/.dlr-complete.bash" >> ~/.bashrc
echo "source $HOME/.dlr-analyzer-complete.bash" >> ~/.bashrc
```

#### fish

```shell
_DLR_COMPLETE=fish_source dlr > $HOME/.config/fish/completions/dlr.fish
_DLR_ANALYZER_COMPLETE=fish_source dlr-analyzer > $HOME/.config/fish/completions/dlr-analyzer.fish
```

## Usage

refer [document](https://tier4.github.io/driving_log_replayer/)

## (For Developer) Release Process

This package uses `catkin_pkg` to manage releases.

Refer [this page](https://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage)

### Release command

Can only be executed by users with repository maintainer privileges

```shell
# create change log
catkin_generate_changelog
# edit CHANGELOG.rst
# update package version in pyproject.toml
# edit ReleaseNotes.md
# commit and create pull request
# merge pull request
catkin_prepare_release
# When you type the command, it automatically updates CHANGELOG.rst and creates a git tag
git checkout main
git merge develop
git push origin main
```
