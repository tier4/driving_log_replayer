# Driving Log Replayer for ROS 2 Autoware.Universe

Driving Log Replayer is a ROS package that evaluates the functionality of Autoware.Universe

This repository is currently in maintenance mode.
Bug fixes only, no new features will be added.
Please use v2 below.

<https://github.com/tier4/driving_log_replayer_v2>

## Requirements

- ROS 2 humble
- [Python 3.10](https://www.python.org/)
- [pipx](https://pipxproject.github.io/pipx/)
  - pipx is installed automatically in Autoware setup.

### Optional

If you want to change the rosbag format from ros1 to ros2.

- [rosbags](https://gitlab.com/ternaris/rosbags)
  - `pip3 install rosbags`

## Installation

You need to install driving_log_replayer and driving_log_replayer_cli package.

### How to install driving_log_replayer package

Use colcon build

```shell
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to driving_log_replayer
```

### How to install driving_log_replayer_cli package

Use pipx **Do not use pip**

```shell
# install
pipx install git+https://github.com/tier4/driving_log_replayer.git

# upgrade
pipx upgrade driving-log-replayer

# uninstall
pipx uninstall driving-log-replayer
```

### Why pipx, not pip

For ros, driving_log_replayer uses numpy 1.22.0
See requirements.txt

On the other hand, cli uses pandas, and the version of numpy required by pandas is different from the version we want to use in ros.
If you install with pip, the pandas-dependent numpy is installed under $HOME/.local/lib/python3.10/site-packages.
This will cause version mismatch, so you need to install cli on an independent venv using pipx.

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
